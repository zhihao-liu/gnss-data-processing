#include <QCoreApplication>
#include <vector>
#include <iostream>

#include "navigation.h"
#include "observation.h"

using namespace std;


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    NavigationData::ptr brdm1980(new NavigationData
                                 ("/users/liuzhihao/workstation/programs/gnss-data-processing/sample data/brdm1980.16p"));
    ObservationData::ptr hcad1980(new ObservationData
                                  ("/users/liuzhihao/workstation/programs/gnss-data-processing/sample data/hcad1980.16o"));

    vector<Coordinates::ptr> results;
    vector<shared_ptr<Vector3d>> errorsInXYZ, errorsInNEU;
    Coordinates preciseValue(-2823793.9960, 4656028.3870, 3309791.2420);
    for (int i = 0; i < int(hcad1980->_observationRecords.size()); ++i)
    {
        Coordinates::ptr coord = hcad1980->_observationRecords.at(i)->
                computeReceiverPosition(brdm1980, hcad1980->_header->approxPosition);
        results.push_back(coord);
        if(coord != nullptr)
        {
            errorsInXYZ.push_back(shared_ptr<Vector3d>(new Vector3d(coord->errorInXYZ(preciseValue))));
            errorsInNEU.push_back(shared_ptr<Vector3d>(new Vector3d(coord->errorInNEU(preciseValue))));
        }
        else
        {
            errorsInXYZ.push_back(nullptr);
            errorsInNEU.push_back(nullptr);
        }
    }

    getchar();

    return a.exec();
}
