#include <fstream>
#include <cmath>

#include "navigation.h"


WeekSecond::WeekSecond(DateTime::cptr dt)
{
    int year = dt->_year;
    int month = dt->_month;
    int day = dt->_day;
    int hour = dt->_hour;
    int minute = dt->_minute;
    int second = dt->_second;
    if (month <= 2)
    {
        year = year - 1;
        month = month + 12;
    }

    double JD = floor(365.25 * year) + floor(30.6001 * (month + 1)) + day + (hour + minute / 60.0 + second / 3600.0) / 24 + 1720981.5;
    _completeWeek = floor((JD - 2444244.5) / 7);
    _remainingSecond = (JD - 2444244.5 - 7 * _completeWeek) * 86400;
}

NavigationData::NavigationData(string filePath)
    :_header(new NavigationHeader())
{
    string rinexType = filePath.substr(filePath.length() - 1, 1);
    if (rinexType != "p" && rinexType != "P")
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
    }

    NavigationRecord::ptr lastRecord;
    NavigationRecord::ptr record;
    while (!file.eof())
    {
        getline(file, line);
        if (line == "")
            break;

        record.reset(new NavigationRecord());

        record->_satPRN = line.substr(0, 3);

        string typeMark = record->_satPRN.substr(0, 1);
        SatType satType = UNKNOWN;
        if (typeMark == "G")
            satType = SAT_G;
        else if (typeMark == "C")
            satType = SAT_C;
        else if (typeMark == "E")
            satType = SAT_E;
        else if (typeMark == "S")
            satType = SAT_S;

        switch (satType)
        {
        case SAT_G:
        {
            int year = atoi(line.substr(3, 5).c_str());
            int month = atoi(line.substr(8, 3).c_str());
            int day = atoi(line.substr(11, 3).c_str());
            int hour = atoi(line.substr(14, 3).c_str());
            int minute = atoi(line.substr(17, 3).c_str());
            int second = atoi(line.substr(20, 3).c_str());
            record->_Toc.reset(new DateTime(year, month, day, hour, minute, second));

            record->_a0 = atof(line.substr(23, 19).c_str());
            record->_a1 = atof(line.substr(42, 19).c_str());
            record->_a2 = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_IODE = atof(line.substr(4, 19).c_str());
            record->_Crs = atof(line.substr(23, 19).c_str());
            record->_DeltaN = atof(line.substr(42, 19).c_str());
            record->_M0 = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_Cuc = atof(line.substr(4, 19).c_str());
            record->_e = atof(line.substr(23, 19).c_str());
            record->_Cus = atof(line.substr(42, 19).c_str());
            record->_sqrtA = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_Toe = atof(line.substr(4, 19).c_str());
            record->_Cic = atof(line.substr(23, 19).c_str());
            record->_Omega0 = atof(line.substr(42, 19).c_str());
            record->_Cis = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_I0 = atof(line.substr(4, 19).c_str());
            record->_Crc = atof(line.substr(23, 19).c_str());
            record->_omega = atof(line.substr(42, 19).c_str());
            record->_OmegaDOT = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_IDOT = atof(line.substr(4, 19).c_str());
            record->_CodesL2Channel = atof(line.substr(23, 19).c_str());
            record->_GpsWeek = atof(line.substr(42, 19).c_str());
            record->_L2DataFlag = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_SvAccuracy = atof(line.substr(4, 19).c_str());
            record->_SvHealth = atof(line.substr(23, 19).c_str());
            record->_Tgd = atof(line.substr(42, 19).c_str());
            record->_IODC = atof(line.substr(61, 19).c_str());

            getline(file, line);
            record->_TransmissionTime = atof(line.substr(4, 19).c_str());
            record->_FitInterval = atof(line.substr(23, 19).c_str());

            // Ignore more than one records of the same satellite within 2 hours.
            if (lastRecord)
                if (record->_satPRN == lastRecord->_satPRN && record->_Toc->isCloseTo(lastRecord->_Toc))
                    continue;

            _navigationRecords.push_back(record);
            lastRecord = record;
        }
            break;

        case SAT_C:
        case SAT_E:
        {
            for (int i = 0; i < 8; ++i)
                getline(file, line);
        }
            break;

        case SAT_S:
        {
            for (int i = 0; i < 4; ++i)
                getline(file, line);
        }
            break;

        default:
            break;
        }
    }

    file.close();
}

Coordinates::ptr NavigationRecord::computeSatellitePosition(double const* ti) const
// Calculate the SatellitePosition
// using internal parameters of the NavigationRecord
// and the specified ti.
// If ti is not specified,
// the approximate TOC in the NavigationRecord will be used.
{
    WeekSecond tocWeekSecond(_Toc);

    if(!ti)
        ti = &tocWeekSecond._remainingSecond;

    double tk = *ti - _Toe;
    if (abs(tk) > 7200)
        return nullptr;

    double A = pow(_sqrtA, 2);
    double n0 = pow(Reference::mu / pow(A, 3), 0.5);
    double ni = n0 + _DeltaN;
    double Mk = _M0 + ni * tk;

    //Iterative calculation for eccentric anomaly
    double Ek1 = Mk;
    double Ek0 = 0.0;
    while (abs(Ek1 - Ek0) > 1.0E-12)
    {
        Ek0 = Ek1;
        Ek1 = Mk + _e * sin(Ek0);
    }
    double Ek = Ek1;

    //Calculate the true anomaly at the certain epoch.
    double tan_half_vk = pow((1 + _e) / (1 - _e), 0.5) * tan(Ek / 2.0);
    double vk = atan(tan_half_vk) * 2.0;

    double uk = vk + _omega;

    //Corrections for second harmonic perturbations.
    double cos2uk = cos(2 * uk);
    double sin2uk = sin(2 * uk);
    double delta_uk = _Cuc * cos2uk + _Cus * sin2uk;
    double delta_rk = _Crc * cos2uk + _Crs * sin2uk;
    double delta_ik = _Cic * cos2uk + _Cis * sin2uk;

    double ui = uk + delta_uk;
    double ri = A * (1 - _e * cos(Ek)) + delta_rk;
    double ii = _I0 + delta_ik;

    double lambda = _Omega0 + (_OmegaDOT - Reference::omegaE) * tk - Reference::omegaE * _Toe;

    double Xi = ri * (cos(ui) * cos(lambda) - sin(ui) * cos(ii) * sin(lambda));
    double Yi = ri * (cos(ui) * sin(lambda) + sin(ui) * cos(ii) * cos(lambda));
    double Zi = ri * (sin(ui) * sin(ii));

    return Coordinates::ptr(new Coordinates(Xi, Yi, Zi));
}

NavigationRecord::cptr NavigationData::findCloseRecord(DateTime::cptr dt, string satPRN) const
{
    for(int i = 0; i < int(_navigationRecords.size()); ++i)
        if(satPRN == _navigationRecords.at(i)->_satPRN &&
                dt->isCloseTo(_navigationRecords.at(i)->_Toc))
            return _navigationRecords.at(i);
    return nullptr;
}
