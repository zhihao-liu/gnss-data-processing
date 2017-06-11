#pragma once

#include "common.h"

using namespace std;


class NavigationHeader
{
public:
    vector<string> _infoLines;

    ~NavigationHeader() = default;
    NavigationHeader() = default;
};

class NavigationRecord
{
public:
    string _satPRN;
    DateTime _Toc;
	double
		_a0, _a1, _a2,
		_IODE, _Crs, _DeltaN, _M0,
		_Cuc, _e, _Cus, _sqrtA,
		_Toe, _Cic, _Omega0, _Cis,
		_I0, _Crc, _omega, _OmegaDOT,
		_IDOT, _CodesL2Channel, _GpsWeek, _L2DataFlag,
		_SvAccuracy, _SvHealth, _Tgd, _IODC,
		_TransmissionTime, _FitInterval;

    ~NavigationRecord() = default;
    NavigationRecord() = default;
    Coordinates computeSatellitePosition(double const* ti = nullptr) const;
};

class GpsWeekSecond
{
public:
    double _week, _second;

    ~GpsWeekSecond() = default;
    GpsWeekSecond() = default;
    GpsWeekSecond(DateTime const& dt);
};

class NavigationData
{
public:
    NavigationHeader _header;
    vector< shared_ptr<NavigationRecord> > _navigationRecords;

    ~NavigationData() = default;
    NavigationData() = default;
    NavigationData(string const& filePath);
    shared_ptr<NavigationRecord> findCloseRecord(DateTime const& dt, string const& satPRN) const;
};
