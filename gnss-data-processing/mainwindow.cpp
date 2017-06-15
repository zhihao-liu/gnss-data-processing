#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <fstream>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->radioButton_L1->setChecked(true);

    connect(ui->pushButton_BrowseNav, SIGNAL(released()), this, SLOT(selectNavFilePath()));
    connect(ui->pushButton_BrowseObs, SIGNAL(released()), this, SLOT(selectObsFilePath()));
    connect(ui->pushButton_Read, SIGNAL(released()), this, SLOT(readFileRecords()));
    connect(ui->pushButton_Compute, SIGNAL(released()), this, SLOT(computeSpp()));
    connect(ui->pushButton_BrowseOutput, SIGNAL(released()), this, SLOT(selectResultPath()));
    connect(ui->pushButton_SaveOutput, SIGNAL(released()), this, SLOT(saveResultFile()));
}

MainWindow::~MainWindow() {}

void MainWindow::selectNavFilePath()
{
    QFileDialog fileDialog(this, "Open Navigation File");
    if (!fileDialog.exec())
        return;

    ui->lineEdit_NavPath->setText(fileDialog.selectedFiles().at(0));
}

void MainWindow::selectObsFilePath()
{
    QFileDialog fileDialog(this, "Open Observation File");
    if (!fileDialog.exec())
        return;

    ui->lineEdit_ObsPath->setText(fileDialog.selectedFiles().at(0));
}

void MainWindow::selectResultPath()
{
    QFileDialog fileDialog(this, "Save Result File", "", "*.csv");
    if (!fileDialog.exec())
        return;

    ui->lineEdit_OutputPath->setText(fileDialog.selectedFiles().at(0) + ".csv");
}

void MainWindow::readFileRecords()
{
    string navPath = ui->lineEdit_NavPath->text().toStdString();
    string obsPath = ui->lineEdit_ObsPath->text().toStdString();

    setMsg("Reading records...");
    qApp->processEvents();

    navigationData.reset(new NavigationData(navPath));
    observationData.reset(new ObservationData(obsPath));

    setMsg("Records have been successfully read.");
}

void MainWindow::computeSpp()
{
    coordinateResults.clear();
    errorsInXYZ.clear();
    errorsInNEU.clear();

    ObservableType observableType;

    if (ui->radioButton_L1->isChecked())
        observableType = ObservableType::GpsL1;
    else if (ui->radioButton_L2->isChecked())
        observableType = ObservableType::GpsL2;
    else if (ui->radioButton_IF->isChecked())
        observableType = ObservableType::GpsIfCombination;
    else if (ui->radioButton_WL->isChecked())
        observableType = ObservableType::GpsWlCombination;
    else
    {
        setMsg("Please select one of the observable types.");
        return;
    }

    setMsg("Processing computation...");
    qApp->processEvents();

    double const INF = 1E8;
    double maxX = 0, maxY = 0, maxZ = 0, maxN = 0, maxE = 0, maxU = 0;
    double minX = INF, minY = INF, minZ = INF, minN = INF, minE = INF, minU = INF;

    for (auto item : observationData->_records)
    {
        Coordinates approxPosition = observationData->_header._approxPosition;
        shared_ptr<Coordinates> receiverPosition = item->computeReceiverPosition(*navigationData, approxPosition, observableType);
        coordinateResults.push_back(receiverPosition);
        if (receiverPosition == nullptr)
        {
            errorsInXYZ.push_back(nullptr);
            errorsInNEU.push_back(nullptr);
        }
        else
        {
            Vector3d errorInXYZ = receiverPosition->errorInXYZ(approxPosition);
            Vector3d errorInNEU = receiverPosition->errorInNEU(approxPosition);
            errorsInXYZ.push_back(shared_ptr<Vector3d>(new Vector3d(errorInXYZ)));
            errorsInNEU.push_back(shared_ptr<Vector3d>(new Vector3d(errorInNEU)));
            if (fabs(errorInXYZ[0]) > maxX) maxX = errorInXYZ[0];
            if (fabs(errorInXYZ[1]) > maxY) maxY = errorInXYZ[1];
            if (fabs(errorInXYZ[2]) > maxZ) maxZ = errorInXYZ[2];
            if (fabs(errorInXYZ[0]) < minX) minX = errorInXYZ[0];
            if (fabs(errorInXYZ[1]) < minY) minY = errorInXYZ[1];
            if (fabs(errorInXYZ[2]) < minZ) minZ = errorInXYZ[2];
            if (fabs(errorInNEU[0]) > maxN) maxN = errorInNEU[0];
            if (fabs(errorInNEU[1]) > maxE) maxE = errorInNEU[1];
            if (fabs(errorInNEU[2]) > maxU) maxU = errorInNEU[2];
            if (fabs(errorInNEU[0]) < minN) minN = errorInNEU[0];
            if (fabs(errorInNEU[1]) < minE) minE = errorInNEU[1];
            if (fabs(errorInNEU[2]) < minU) minU = errorInNEU[2];
        }
    }

    setMsg("Computation is completed.");
    setMsg("Errors in different directions:");
    setMsg(QString("Maximum: dX = ") + QString::number(maxX) + ", dY = " + QString::number(maxY) + ", dZ = " + QString::number(maxZ));
    setMsg(QString("\tdN = ") + QString::number(maxN) + ", dE = " + QString::number(maxE) + ", dU = " + QString::number(maxU));
    setMsg(QString("Minimum: dX = ") + QString::number(minX) + ", dY = " + QString::number(minY) + ", dZ = " + QString::number(minZ));
    setMsg(QString("\tdN = ") + QString::number(minN) + ", dE = " + QString::number(minE) + ", dU = " + QString::number(minU));
    setMsg("You can now save the results to a CSV file.");
}

void MainWindow::setMsg(QString const& msg)
{
    ui->listWidget_Msg->addItem(msg);
}

void MainWindow::saveResultFile()
{
    setMsg("Saving results...");
    qApp->processEvents();

    ofstream fout(ui->lineEdit_OutputPath->text().toStdString());
    fout << "X," << "Y," << "Z," << "dX," << "dY," << "dZ," << "dN," << "dE," << "dU" << endl;
    for (int i = 0; i < coordinateResults.size(); ++i)
    {
        shared_ptr<Coordinates> coord = coordinateResults.at(i);
        shared_ptr<Vector3d> errXYZ = errorsInXYZ.at(i);
        shared_ptr<Vector3d> errNEU = errorsInNEU.at(i);
        if (coord != nullptr)
        {
            fout << coord->_X << "," << coord->_Y << "," << coord->_Z << ",";
            fout << (*errXYZ)[0] << "," << (*errXYZ)[1] << "," << (*errXYZ)[2] << ",";
            fout << (*errNEU)[0] << "," << (*errNEU)[1] << "," << (*errNEU)[2] << endl;
        }
    }
    fout.close();

    setMsg("Results have been successfully saved.");
}
