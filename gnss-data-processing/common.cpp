#include "common.h"
#include <cmath>

DateTime::DateTime(int year, int month, int day, int hour, int minute, int second)
    : _year(year), _month(month), _day(day), _hour(hour), _minute(minute), _second(second) {}

DateTime DateTime::operator- (DateTime const &dt) const
{
    return DateTime(_year - dt._year, _month - dt._month, _day - dt._day, _hour - dt._hour, _minute - dt._minute, _second - dt._second);
}

int DateTime::timeSpanAsSecondsInSingleDay() const
{
    if (_year || _month || _day)
        return -1;
    else
        return fabs(_hour * 3600 + _minute * 60 + _second);
}

bool DateTime::isCloseTo (DateTime::cptr dt) const
{
    int timeSpan = (*this - *dt).timeSpanAsSecondsInSingleDay();
    if (timeSpan > 0 && timeSpan <= 7200)
        return true;
    else
        return false;
}

Coordinates::Coordinates(double X, double Y, double Z)
    : _X(X), _Y(Y), _Z(Z) {}

Coordinates::Coordinates(Vector3d const& vec)
    : _X(vec[0]), _Y(vec[1]), _Z(vec[2]) {}


Vector3d Coordinates::toXYZ() const
{
    Vector3d vec;
    vec << _X, _Y, _Z;
    return vec;
}

Vector3d Coordinates::toBLH() const
{
    double const normXY = Vector2d(_X, _Y).norm();

    double L = atan(_Y / _X) + PI;
    double B = atan(_Z / normXY);

    double const ITER_TOL = 1E-8;
    for(int i = 0; i <= 50; ++i)
    {
        if(i >= 50)
            return Vector3d(NAN, NAN, NAN); // Iteration convergence fails.

        double BPrev = B;
        B = atan((_Z + getN(B) * pow(Reference::e, 2) * sin(B)) / normXY);

        if (fabs(B - BPrev) < ITER_TOL)
            break;
    }

    double H = normXY / cos(B) - getN(B);

    return Vector3d(B, L, H);
}

double Coordinates::getN(double B)
{
    return Reference::a / sqrt(1 - pow(Reference::e * sin(B), 2));
}

Vector3d Coordinates::toNEU(Coordinates const& stationCoord) const
{
    Vector3d BLH = stationCoord.toBLH();
    double B0 = BLH[0], L0 = BLH[1], H0 = BLH[2];
    double N0 = getN(B0);

    double dX = (N0 + H0) * cos(B0) * cos(L0);
    double dY = (N0 + H0) * cos(B0) * sin(L0);
    double dZ = (N0 * (1 - pow(Reference::e, 2)) + H0) * sin(B0);

    return transThsToTes(B0, L0).transpose() * (toXYZ() - Vector3d(dX, dY, dZ));
}

MatrixXd Coordinates::transThsToTes(double B, double L)
{
    double epsY = PI / 2 - B;
    double epsZ = PI - L;

    MatrixXd Rz(3, 3), Ry(3, 3), Py(3, 3);
    Py << 1, 0, 0, 0, -1, 0, 0, 0, 1;
    Ry << cos(epsY), 0, -sin(epsY), 0, 1, 0, sin(epsY), 0, cos(epsY);
    Rz << cos(epsZ), -sin(epsZ), 0, sin(epsZ), cos(epsZ), 0, 0, 0, 1;

    return Rz * Ry * Py;
}

Vector3d Coordinates::errorInXYZ(Coordinates const& preciseValue) const
{
    double dX = _X - preciseValue._X;
    double dY = _Y - preciseValue._Y;
    double dZ = _Z - preciseValue._Z;

    return Vector3d(dX, dY, dZ);
}

Vector3d Coordinates::errorInNEU(Coordinates const& preciseValue) const
{
    Vector3d BLH = preciseValue.toBLH();
    double B0 = BLH[0], L0 = BLH[1];

    return transThsToTes(B0, L0).transpose() * errorInXYZ(preciseValue);
}
