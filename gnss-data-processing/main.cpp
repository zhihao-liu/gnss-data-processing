#include <QCoreApplication>
#include <vector>
#include <iostream>
#include <fstream>

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
    ofstream fout("/users/liuzhihao/desktop/results.csv");

    vector<Coordinates::ptr> results;
    vector<shared_ptr<Vector3d>> errorsInXYZ, errorsInNEU;
    Coordinates preciseValue(-2823793.9960, 4656028.3870, 3309791.2420);
    for (long i = 0; i < long(hcad1980->_observationRecords.size()); ++i)
    {
        Coordinates::ptr coord = hcad1980->_observationRecords.at(i)->computeReceiverPosition(brdm1980, hcad1980->_header->approxPosition);
        results.push_back(coord);
        if(coord != nullptr)
        {
            fout << coord->_X << "," << coord->_Y << "," << coord->_Z <<  ",";
            errorsInXYZ.push_back(shared_ptr<Vector3d>(new Vector3d(coord->errorInXYZ(preciseValue))));            
            fout << (*errorsInXYZ.back())[0] << "," << (*errorsInXYZ.back())[1] << "," << (*errorsInXYZ.back())[2] << ",";
            errorsInNEU.push_back(shared_ptr<Vector3d>(new Vector3d(coord->errorInNEU(preciseValue))));            
            fout << (*errorsInNEU.back())[0] << "," << (*errorsInNEU.back())[1] << "," << (*errorsInNEU.back())[2] << endl;
        }
        else
        {
            errorsInXYZ.push_back(nullptr);
            errorsInNEU.push_back(nullptr);
        }
    }

    cout << "OUTPUT FINISHED" << endl;

    getchar();

    return a.exec();
}
