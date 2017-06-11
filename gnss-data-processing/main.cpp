#include "mainwindow.h"
#include <QApplication>
#include <fstream>
#include <iostream>
#include "common.h"
#include "navigation.h"
#include "observation.h"

using namespace std;

//int main(int argc, char *argv[])
//{
//    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();

//    return a.exec();
//}

int main()
{
    int count = 0;

    NavigationData navData("/Users/liuzhihao/Workstation/Programs/gnss-data-processing/sample data/brdm1980.16p");
    ObservationData obsData("/Users/liuzhihao/Workstation/Programs/gnss-data-processing/sample data/hcad1980.16o");

    ofstream fout("/Users/liuzhihao/Desktop/results.csv");

    for (auto item : obsData._observationRecords)
    {
        shared_ptr<Coordinates> recCoord = item->computeReceiverPosition(navData, obsData._header._approxPosition);
        if (recCoord != nullptr)
        {
            Coordinates errXYZ = recCoord->errorInXYZ(obsData._header._approxPosition);
            Coordinates errNEU = recCoord->errorInNEU(obsData._header._approxPosition);
            fout << count << ",";
            fout << errXYZ._X << "," << errXYZ._Y << "," << errXYZ._Z << ",";
            fout << errNEU._X << "," << errNEU._Y << "," << errNEU._Z << endl;
        }
    }

    cout << "OUTPUT COMPLETED" << endl;

    getchar();

    return 0;
}
