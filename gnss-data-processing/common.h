#pragma once
#include <memory>
#include <string>
#include <vector>
#include "../Eigen/Dense" // External libs for matrix manipulation

using namespace std;
using namespace Eigen;

#define PI 3.14159265358979
#define EPS 1e-8

namespace Reference
{
double const mu = 3.9860050E14;
double const omegaE = 7.2921151467E-5;

double const a = 6378137.0;
double const f = 1 / 298.257223563;
double const e = sqrt(2 * Reference::f - pow(Reference::f, 2));

double const c = 299792458;
};

enum SatType
{
    UNKNOWN = 0,
    SAT_G,
    SAT_C,
    SAT_E,
    SAT_S
};

class DateTime
{
public:
    int _year, _month, _day, _hour, _minute, _second;

    ~DateTime() = default;
    DateTime() = default;
    DateTime(int year, int month, int day, int hour, int minute, int second);
    DateTime operator- (DateTime const& dt) const;
    int timeSpanAsSecondsInSingleDay() const;
    bool isCloseTo(DateTime const& dt) const;
};

class Coordinates
{
public:
    double _X, _Y, _Z;

    ~Coordinates() = default;
    Coordinates() = default;
    Coordinates(double X, double Y, double Z);
    Coordinates(Vector3d const& vec);

    Vector3d toXYZ() const;
    Vector3d toBLH() const;
    Vector3d toNEU(Coordinates const& originCoord) const;
    Vector3d errorInXYZ(Coordinates const& preciseValue) const;
    Vector3d errorInNEU(Coordinates const& preciseValue) const;

    static MatrixXd rotationXyzToNeu(double B, double L);
    static double getN(double B);
};
