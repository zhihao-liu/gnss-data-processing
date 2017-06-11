#pragma once

#include "common.h"
#include "navigation.h"

using namespace std;


class ObservationHeader
{
public:
    vector<string> _infoLines;
    Coordinates _approxPosition;

    ~ObservationHeader() = default;
    ObservationHeader() = default;
};

class ObservationRecord
{
public:
    DateTime _receiverTime;
	int _statusFlag;
	int _sumSat;
    vector<string> _listSatPRN;
    vector<double> _pseudorange_C1C;
    vector<double> _pseudorange_C2P;
    vector<double> _phase_L1C;
    vector<double> _phase_L2P;

    ~ObservationRecord() = default;
    ObservationRecord() = default;
    shared_ptr<Coordinates> computeReceiverPosition(NavigationData const& navigationData, Coordinates const& approxRecCoord) const;
};

class ObservationData
{
public:
    ObservationHeader _header;
    vector< shared_ptr<ObservationRecord> > _observationRecords;

    ~ObservationData() = default;
    ObservationData() = default;
    ObservationData(string const& filePath);
};
