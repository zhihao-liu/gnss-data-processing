#include <fstream>
#include <cmath>

#include "navigation.h"


GpsWeekSecond::GpsWeekSecond(DateTime const& dt)
{
    int year = dt._year;
    int month = dt._month;
    int day = dt._day;
    int hour = dt._hour;
    int minute = dt._minute;
    int second = dt._second;

    if (month <= 2)
    {
        year = year - 1;
        month = month + 12;
    }

    double JD = floor(365.25 * year) + floor(30.6001 * (month + 1)) + day + (hour + minute / 60.0 + second / 3600.0) / 24 + 1720981.5;
    _week = floor((JD - 2444244.5) / 7);
    _second = round((JD - 2444244.5 - 7 * _week) * 86400);
}

NavigationData::NavigationData(string const& filePath)
{
    string rinexType = filePath.substr(filePath.length() - 1, 1);
    if (rinexType != "p" && rinexType != "P" &&
            rinexType != "n" && rinexType != "N")
        return;

    ifstream file(filePath);
    if (!file) return;

    string line;

    while (!file.eof())
    {
        getline(file, line);
        _header._infoLines.push_back(line);
        if (line.substr(60, 13) == "END OF HEADER")
            break;
    }

    shared_ptr<NavigationRecord> lastRecord;
    shared_ptr<NavigationRecord> record;
    while (!file.eof())
    {
        getline(file, line);
        if (line == "")
            break;

        record.reset(new NavigationRecord());

        record->_satPRN = line.substr(0, 3);

        string typeMark = record->_satPRN.substr(0, 1);
        SatType satType = SatType::Unknown;
        if (typeMark == "G")
            satType = SatType::Gps;
        else if (typeMark == "C")
            satType = SatType::Bds;
        else if (typeMark == "E")
            satType = SatType::Galileo;
        else if (typeMark == "S")
            satType = SatType::Sbas;

        switch (satType)
        {
        case SatType::Gps:
        {
            int year = StringConverter::toInt(line.substr(3, 5));
            int month = StringConverter::toInt(line.substr(8, 3));
            int day = StringConverter::toInt(line.substr(11, 3));
            int hour = StringConverter::toInt(line.substr(14, 3));
            int minute = StringConverter::toInt(line.substr(17, 3));
            int second = StringConverter::toInt(line.substr(20, 3));
            record->_Toc = DateTime(year, month, day, hour, minute, second);

            record->_a0 = StringConverter::toDouble(line.substr(23, 19));
            record->_a1 = StringConverter::toDouble(line.substr(42, 19));
            record->_a2 = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_IODE = StringConverter::toDouble(line.substr(4, 19));
            record->_Crs = StringConverter::toDouble(line.substr(23, 19));
            record->_DeltaN = StringConverter::toDouble(line.substr(42, 19));
            record->_M0 = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_Cuc = StringConverter::toDouble(line.substr(4, 19));
            record->_e = StringConverter::toDouble(line.substr(23, 19));
            record->_Cus = StringConverter::toDouble(line.substr(42, 19));
            record->_sqrtA = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_Toe = StringConverter::toDouble(line.substr(4, 19));
            record->_Cic = StringConverter::toDouble(line.substr(23, 19));
            record->_Omega0 = StringConverter::toDouble(line.substr(42, 19));
            record->_Cis = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_I0 = StringConverter::toDouble(line.substr(4, 19));
            record->_Crc = StringConverter::toDouble(line.substr(23, 19));
            record->_omega = StringConverter::toDouble(line.substr(42, 19));
            record->_OmegaDOT = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_IDOT = StringConverter::toDouble(line.substr(4, 19));
            record->_CodesL2Channel = StringConverter::toDouble(line.substr(23, 19));
            record->_GpsWeek = StringConverter::toDouble(line.substr(42, 19));
            record->_L2DataFlag = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_SvAccuracy = StringConverter::toDouble(line.substr(4, 19));
            record->_SvHealth = StringConverter::toDouble(line.substr(23, 19));
            record->_Tgd = StringConverter::toDouble(line.substr(42, 19));
            record->_IODC = StringConverter::toDouble(line.substr(61, 19));

            getline(file, line);
            record->_TransmissionTime = StringConverter::toDouble(line.substr(4, 19));
            record->_FitInterval = StringConverter::toDouble(line.substr(23, 19));

            // Ignore more than one records of the same satellite within 2 hours.
            if (lastRecord)
                if (record->_satPRN == lastRecord->_satPRN && record->_Toc.isCloseTo(lastRecord->_Toc))
                    continue;

            _records.push_back(record);
            lastRecord = record;
        }
            break;

        case SatType::Bds:
        case SatType::Galileo:
        {
            for (int i = 0; i < 8; ++i)
                getline(file, line);
        }
            break;

        case SatType::Sbas:
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

Coordinates NavigationRecord::computeSatellitePosition(double const* ti) const
// Calculate the SatellitePosition
// using internal parameters of the NavigationRecord
// and the specified ti.
// If ti is not specified,
// the approximate TOC in the NavigationRecord will be used.
{
    GpsWeekSecond tocWeekSecond(_Toc);

    if (ti == nullptr)
        ti = &tocWeekSecond._second;

    double tk = *ti - _Toe;

//    if (fabs(tk) > 7200)
//        return nullptr;

    double A = pow(_sqrtA, 2);
    double n0 = pow(PhysicalConstants::mu / pow(A, 3), 0.5);
    double ni = n0 + _DeltaN;
    double Mk = _M0 + ni * tk;

    //Iterative calculation for eccentric anomaly
    double Ek1 = Mk;
    double Ek0 = 0.0;
    while (fabs(Ek1 - Ek0) > 1.0E-12)
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

    double lambda = _Omega0 + (_OmegaDOT - PhysicalConstants::omegaE) * tk - PhysicalConstants::omegaE * _Toe;

    double Xi = ri * (cos(ui) * cos(lambda) - sin(ui) * cos(ii) * sin(lambda));
    double Yi = ri * (cos(ui) * sin(lambda) + sin(ui) * cos(ii) * cos(lambda));
    double Zi = ri * (sin(ui) * sin(ii));

    return Coordinates(Xi, Yi, Zi);
}

shared_ptr<NavigationRecord> NavigationData::findCloseRecord(DateTime const& dt, string const& satPRN) const
{
    for (auto item : _records)
        if (satPRN == item->_satPRN && dt.isCloseTo(item->_Toc))
            return item;

    return nullptr;
}
