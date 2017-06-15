#include "mainwindow.h"
#include <QApplication>
#include <fstream>
#include <iostream>
#include "common.h"
#include "navigation.h"
#include "observation.h"

using namespace std;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

//int main()
//{
//    int count = 0;

//    NavigationData navData("C:/Workstation/Programs/gnss-data-processing/sample data/CMDT.16n");
//    ObservationData obsData("C:/Workstation/Programs/gnss-data-processing/sample data/CMDT.16o");

//    ofstream fout("C:/Workstation/Programs/gnss-data-processing/results/results.csv");

//    for (auto item : obsData._observationRecords)
//    {
//        ++count;
//        shared_ptr<Coordinates> recCoord = item->computeReceiverPosition(navData, obsData._header._approxPosition, ObservableType::GpsL1);
//        if (recCoord != nullptr)
//        {
//            Coordinates errXYZ = recCoord->errorInXYZ(obsData._header._approxPosition);
//            Coordinates errNEU = recCoord->errorInNEU(obsData._header._approxPosition);
//            fout << count << ",";
//            fout << errXYZ._X << "," << errXYZ._Y << "," << errXYZ._Z << ",";
//            fout << errNEU._X << "," << errNEU._Y << "," << errNEU._Z << endl;
//        }
//    }

//    cout << "OUTPUT COMPLETED" << endl;

//    getchar();

//    return 0;
//}
