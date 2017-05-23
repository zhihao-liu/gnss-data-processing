#pragma once

#include <vector>
#include <string>

#include "common.h"

using namespace std;


class NavigationHeader
{
public:
    typedef shared_ptr<NavigationHeader> ptr;

	vector<string> _infoLines;
};

class NavigationRecord
{
public:
    typedef shared_ptr<NavigationRecord> ptr;
    typedef shared_ptr<NavigationRecord const> cptr;

	string _satPRN;
    DateTime::ptr _Toc;
	double
		_a0, _a1, _a2,
		_IODE, _Crs, _DeltaN, _M0,
		_Cuc, _e, _Cus, _sqrtA,
		_Toe, _Cic, _Omega0, _Cis,
		_I0, _Crc, _omega, _OmegaDOT,
		_IDOT, _CodesL2Channel, _GpsWeek, _L2DataFlag,
		_SvAccuracy, _SvHealth, _Tgd, _IODC,
		_TransmissionTime, _FitInterval;
	
    NavigationRecord() {}
    Coordinates::ptr computeSatellitePosition(double const* ti = nullptr) const;
};

class WeekSecond
{
public:
    typedef shared_ptr<WeekSecond> ptr;

    double _completeWeek, _remainingSecond;

    WeekSecond(DateTime::cptr dt);
};

class NavigationData
{
public:
    typedef shared_ptr<NavigationData> ptr;
    typedef shared_ptr<NavigationData const> cptr;

    NavigationHeader::ptr _header;
    vector<NavigationRecord::ptr> _navigationRecords;

    NavigationData(string filePath);
    NavigationRecord::cptr findCloseRecord(DateTime::cptr dt, string satPRN) const;
};
