#include <fstream>
#include <cmath>

#include "observation.h"

ObservationData::ObservationData(string filePath)
    :_header(new ObservationHeader())
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
        _header->_infoLines.push_back(line);
        if (line.substr(60, 13) == "END OF HEADER")
            break;
        if (line.substr(60, 19) == "APPROX POSITION XYZ")
        {
            double x, y, z;
            x = atof(line.substr(1, 13).c_str());
            y = atof(line.substr(15, 13).c_str());
            z = atof(line.substr(29, 13).c_str());
            _header->approxPosition.reset(new Coordinates(x, y, z));
        }
    }

    ObservationRecord::ptr record;
    while (!file.eof())
    {
        getline(file, line);
        if (line == "")
            break;

        record.reset(new ObservationRecord());

        int year = atoi(line.substr(1, 5).c_str());
        int month = atoi(line.substr(6, 3).c_str());
        int day = atoi(line.substr(9, 3).c_str());
        int hour = atoi(line.substr(12, 3).c_str());
        int minute = atoi(line.substr(15, 3).c_str());
        int second = atoi(line.substr(18, 11).c_str());
        record->_receiverTime.reset(new DateTime(year, month, day, hour, minute, second));
        record->_statusFlag = atoi(line.substr(29, 3).c_str());
        record->_sumSat = atoi(line.substr(32, 3).c_str());

        for (int i = 0; i < record->_sumSat; ++i)
        {
            getline(file, line);
            if (line.substr(0, 1) != "G")
                continue; // Temporarily ignore satellites of systems other than GPS.

            record->_listSatPRN.push_back(line.substr(0, 3));
            record->_pseudorange_C1C.push_back(atof(line.substr(3, 14).c_str()));
            record->_pseudorange_C2P.push_back(atof(line.substr(19, 14).c_str()));
            record->_phase_L1C.push_back(atof(line.substr(51, 14).c_str()));
            record->_phase_L2P.push_back(atof(line.substr(67, 14).c_str()));
        }

        _observationRecords.push_back(record);

        // Reduce the number of records for provisional testing.
        if(_observationRecords.size() >= 100)
            break;
    }

    file.close();
}

Coordinates::ptr ObservationRecord::computeReceiverPosition(NavigationData::cptr navigationData, Coordinates::cptr approxRecCoord) const
{
    double const ITER_TOL = 10e-8;

    Vector3d recCoord = approxRecCoord->toXYZ();
    double recClockError = 0;

    for (int itrAdjust = 0; itrAdjust <= 100; ++itrAdjust)
    {
        if (itrAdjust >= 100)
            return nullptr; // Iteration convergence fails.

        MatrixXd designMat(_listSatPRN.size(), 4);
        VectorXd observableVec(_listSatPRN.size());
        VectorXd weightVec(_listSatPRN.size());

        int countDisposed = 0; // Count the number of records with some data lost.

        for(int satIndex = 0; satIndex < int(_listSatPRN.size()); ++satIndex)
            // Observation for each satellite can generates a equation.
        {
            if(_pseudorange_C1C.at(satIndex) < EPS)
                // Ignore records with data partly lost.
            {
                ++countDisposed;
                continue;
            }

            NavigationRecord::cptr closeRecord = navigationData->findCloseRecord(_receiverTime, _listSatPRN.at(satIndex));
            if(!closeRecord)
                return nullptr; // No corresponding empheris found.

            double recTimeF = WeekSecond(_receiverTime)._remainingSecond;
            double estimatedTimeDelay = 0.075;

            MatrixXd rotationCorrection(3, 3);
            double angle = closeRecord->_OmegaDOT * (_pseudorange_C1C.at(satIndex) / Reference::c);
            rotationCorrection << cos(angle), sin(angle), 0, -sin(angle), cos(angle), 0, 0, 0, 1;
            Vector3d satCoord;

            double satTimeF = 0;
            for(int itrTime = 0; itrTime <= 100; ++itrTime)
            {
                if (itrTime >= 100)
                    return nullptr; // Iteration convergence fails.

                double satTimePrev = satTimeF;
                satTimeF = recTimeF - recClockError - estimatedTimeDelay;
                satCoord = closeRecord->computeSatellitePosition(&satTimeF)->toXYZ();
                satCoord = rotationCorrection * satCoord;

                if(abs(satTimeF - satTimePrev) <= ITER_TOL)
                    break;

                estimatedTimeDelay = (satCoord - recCoord).norm() / Reference::c;
            }

            double satElevation = asin(satCoord[2] / satCoord.norm());
            if(satElevation <= 15.0 / 180 * PI)
                // Ignore records of low elevation.
            {
                ++countDisposed;
                continue;
            }

            double rho = (recCoord - satCoord).norm();
            double aX = (recCoord[0] - satCoord[0]) / rho;
            double aY = (recCoord[1] - satCoord[1]) / rho;
            double aZ = (recCoord[2] - satCoord[2]) / rho;

            double satClockError = closeRecord->_a0 + closeRecord->_a1 * (recTimeF - satTimeF) + closeRecord->_a2 * pow((recTimeF - satTimeF), 2);

            designMat.row(satIndex - countDisposed) << aX, aY, aZ, 1;
            observableVec[satIndex - countDisposed] = _pseudorange_C1C.at(satIndex) - rho + Reference::c * satClockError;
            weightVec[satIndex - countDisposed] = pow(sin(satElevation), 2);
        }

        if(countDisposed > 0)
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
        recClockError = solution[3] / Reference::c;


        if(solution.block<3, 1>(0, 0).norm() < ITER_TOL)
            break;
    }

    return Coordinates::ptr(new Coordinates(recCoord));
}
