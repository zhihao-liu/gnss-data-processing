#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>
#include <vector>
#include "common.h"
#include "navigation.h"
#include "observation.h"

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    unique_ptr<Ui::MainWindow> ui;

    void setMsg(QString const& msg);

    unique_ptr<NavigationData> navigationData;
    unique_ptr<ObservationData> observationData;
    vector< shared_ptr<Coordinates> > coordinateResults;
    vector< shared_ptr<Vector3d> > errorsInXYZ;
    vector< shared_ptr<Vector3d> > errorsInNEU;

private slots:
    void selectNavFilePath();
    void selectObsFilePath();
    void readFileRecords();
    void computeSpp();
    void selectResultPath();
    void saveResultFile();
};

#endif // MAINWINDOW_H
