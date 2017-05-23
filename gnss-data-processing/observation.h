#pragma once

#include <vector>
#include <string>

#include "common.h"
#include "navigation.h"

using namespace std;


class ObservationHeader
{
public:
    typedef shared_ptr<ObservationHeader> ptr;

	vector<string> _infoLines;
    Coordinates::ptr approxPosition;
};

class ObservationRecord
{
public:
    typedef shared_ptr<ObservationRecord> ptr;

    DateTime::ptr _receiverTime;
	int _statusFlag;
	int _sumSat;
    vector<string> _listSatPRN;
	vector<double> _pseudorange_C1C;
	vector<double> _pseudorange_C2P;
	vector<double> _phase_L1C;
	vector<double> _phase_L2P;

    ObservationRecord() {}
    Coordinates::ptr computeReceiverPosition(NavigationData::cptr navigationData, Coordinates::cptr approxRecCoord) const;
};

class ObservationData
{
public:
    typedef shared_ptr<ObservationData> ptr;

    ObservationHeader::ptr _header;
    vector<ObservationRecord::ptr> _observationRecords;

	ObservationData(string filePath);
};
