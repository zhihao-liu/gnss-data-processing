#include <fstream>
#include <cmath>

#include "observation.h"

ObservationData::ObservationData(string const& filePath)
{
    string rinexType = filePath.substr(filePath.length() - 1, 1);
    if (rinexType != "o" && rinexType != "O")
        return;

    ifstream file(filePath.c_str());
    if (!file) return;

    string line;

    while (!file.eof())
    {
        getline(file, line);
        _header._infoLines.push_back(line);
        if (line.substr(60, 13) == "END OF HEADER")
            break;
        if (line.substr(60, 19) == "APPROX POSITION XYZ")
        {
            double x, y, z;
            x = StringConverter::toDouble(line.substr(1, 13));
            y = StringConverter::toDouble(line.substr(15, 13));
            z = StringConverter::toDouble(line.substr(29, 13));
            _header._approxPosition = Coordinates(x, y, z);
        }
    }

    shared_ptr<ObservationRecord> record;
    while (!file.eof())
    {
        getline(file, line);
        if (line == "")
            break;

        record.reset(new ObservationRecord());

        int year = StringConverter::toInt(line.substr(1, 5));
        int month = StringConverter::toInt(line.substr(6, 3));
        int day = StringConverter::toInt(line.substr(9, 3));
        int hour = StringConverter::toInt(line.substr(12, 3));
        int minute = StringConverter::toInt(line.substr(15, 3));
        int second = StringConverter::toInt(line.substr(18, 11));
        record->_receiverTime = DateTime(year, month, day, hour, minute, second);
        record->_statusFlag = StringConverter::toInt(line.substr(29, 3));
        record->_sumSat = StringConverter::toInt(line.substr(32, 3));

        for (int i = 0; i < record->_sumSat; ++i)
        {
            getline(file, line);
            if (line.substr(0, 1) != "G")
                continue; // Temporarily ignore satellites of systems other than GPS.

            record->_listSatPRN.push_back(line.substr(0, 3));
            record->_pseudorange_C1C.push_back(StringConverter::toDouble(line.substr(3, 14)));
            record->_pseudorange_C2P.push_back(StringConverter::toDouble(line.substr(19, 14)));
            record->_phase_L1C.push_back(StringConverter::toDouble(line.substr(51, 14)));
            record->_phase_L2P.push_back(StringConverter::toDouble(line.substr(67, 14)));
        }

        _records.push_back(record);

        // Reduce the number of records for provisional testing.
//                if (_observationRecords.size() >= 5000)
//                    break;
    }

    file.close();
}

shared_ptr<Coordinates> ObservationRecord::computeReceiverPosition(NavigationData const& navigationData, Coordinates const& approxRecCoord, ObservableType observableType) const
{
    double const ITER_TOL = 1E-8;
    double const BLUNDER_PICKER = 0.5E6;
    double const CUTOFF_ELEVATION = 10.0 / 180 * PI;

    Vector3d recCoord = approxRecCoord.toXYZ();
    double recClockError = 0;

    for (int itrAdjust = 0; itrAdjust <= 100; ++itrAdjust)
    {
        if (itrAdjust >= 100)
            return nullptr; // Iteration convergence fails.

        MatrixXd designMat(_listSatPRN.size(), 4);
        VectorXd observableVec(_listSatPRN.size());
        VectorXd weightVec(_listSatPRN.size());

        int countDisposed = 0; // Count the number of records with some data lost.

        for (int satIndex = 0; satIndex < int(_listSatPRN.size()); ++satIndex)
            // Observation for each single satellite generates a equation.
        {
            double observableValue = getObservable(observableType, satIndex);

            if (observableValue < EPS)
                // Ignore records with data partly lost.
            {
                ++countDisposed;
                continue;
            }

            shared_ptr<NavigationRecord> closeRecord = navigationData.findCloseRecord(_receiverTime, _listSatPRN.at(satIndex));
            if (closeRecord == nullptr)
                return nullptr; // No corresponding empheris found.

            double recTimeF = GpsWeekSecond(_receiverTime)._second;
            double estimatedTimeDelay = 0.075;

            MatrixXd rotationCorrection(3, 3);
            double angle = closeRecord->_OmegaDOT * (observableValue / PhysicalConstants::c);
            rotationCorrection << cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1;
            Vector3d satCoord;

            double satTimeF = 0;
            for (int itrTime = 0; itrTime <= 100; ++itrTime)
            {
                if (itrTime >= 100)
                    return nullptr; // Iteration convergence fails.

                double satTimePrev = satTimeF;
                satTimeF = recTimeF - recClockError - estimatedTimeDelay;
                satCoord = closeRecord->computeSatellitePosition(&satTimeF).toXYZ();
                satCoord = rotationCorrection * satCoord;

                if (fabs(satTimeF - satTimePrev) <= ITER_TOL)
                    break;

                estimatedTimeDelay = (satCoord - recCoord).norm() / PhysicalConstants::c;
            }

            Vector3d satCoordNEU = Coordinates(satCoord).toNEU(approxRecCoord);
            double satElevation = asin(satCoordNEU[2] / satCoordNEU.norm());
            if (satElevation <= CUTOFF_ELEVATION)
                // Ignore records of low elevation.
            {
                ++countDisposed;
                continue;
            }

            double rho = (recCoord - satCoord).norm();
            if (fabs(rho - observableValue) > BLUNDER_PICKER)
                // Ignore records with probable blunders.
            {
                ++countDisposed;
                continue;
            }

            double aX = (recCoord[0] - satCoord[0]) / rho;
            double aY = (recCoord[1] - satCoord[1]) / rho;
            double aZ = (recCoord[2] - satCoord[2]) / rho;

            double closeTocF = GpsWeekSecond(closeRecord->_Toc)._second;
            double satClockError = closeRecord->_a0 + closeRecord->_a1 * (satTimeF - closeTocF) + closeRecord->_a2 * pow((satTimeF - closeTocF), 2);

            designMat.row(satIndex - countDisposed) << aX, aY, aZ, 1;
            observableVec[satIndex - countDisposed] = observableValue - rho + PhysicalConstants::c * satClockError;
            weightVec[satIndex - countDisposed] = pow(sin(satElevation), 2);
        }

        if (countDisposed > 0)
        {
            if (_listSatPRN.size() - countDisposed < 4)
                return nullptr; // No enough observations to solve the equation.

            MatrixXd temp;
            temp = designMat;
            designMat = temp.block(0, 0, temp.rows() - countDisposed, temp.cols());
            temp = observableVec;
            observableVec = temp.block(0, 0, temp.size() - countDisposed, 1);
            temp = weightVec;
            weightVec = temp.block(0, 0, temp.size() - countDisposed, 1);
        }

        MatrixXd weightMat(weightVec.asDiagonal());
        VectorXd solution = (designMat.transpose() * weightMat * designMat).inverse() * designMat.transpose() * weightMat * observableVec;
        recCoord[0] += solution[0];
        recCoord[1] += solution[1];
        recCoord[2] += solution[2];
        recClockError = solution[3] / PhysicalConstants::c;


        if (solution.block<3, 1>(0, 0).norm() < ITER_TOL)
            break;
    }

    return shared_ptr<Coordinates>(new Coordinates(recCoord));
}

double ObservationRecord::getObservable(ObservableType observableType, int index) const
{
    switch (observableType)
    {
    case ObservableType::GpsL1:
        return _pseudorange_C1C.at(index);
        break;

    case ObservableType::GpsL2:
        return _pseudorange_C2P.at(index);
        break;

    case ObservableType::GpsIfCombination:
    {
        double p1 = _pseudorange_C1C.at(index);
        double p2 = _pseudorange_C2P.at(index);
        double f1Sqr = pow(SatlliteProperties::frequencyGpsL1, 2);
        double f2Sqr = pow(SatlliteProperties::frequencyGpsL2, 2);

        return (f1Sqr * p1 - f2Sqr * p2) / (f1Sqr - f2Sqr);
    }
        break;

    case ObservableType::GpsWlCombination:
    {
        double p1 = _pseudorange_C1C.at(index);
        double p2 = _pseudorange_C2P.at(index);
        double f1 = SatlliteProperties::frequencyGpsL1;
        double f2 = SatlliteProperties::frequencyGpsL2;

        return (f1 * p1 - f2 * p2) / (f1 - f2);
    }

    default:
        return 0;
        break;
    }
}
