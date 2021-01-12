/*
    postprocessform.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <memory>
#include <math.h>

#include <QTime>
#include <QSettings>
#include <QMessageBox>
#include <QtMath>

#include "postprocessform.h"
#include "ui_postprocessform.h"
#include "transformmatrixgenerator.h"

struct
{
    QString name;
    double values[4][4];
} static const transformationPresets[] =
{
    { "XYZ = +N+E+D or NED -> +X+Y+Z (default \"no conversion\")",
        {
            {  1,  0,  0,  0 },
            {  0,  1,  0,  0 },
            {  0,  0,  1,  0 },
            {  0,  0,  0,  1 },
        }
    },
    { "XYZ = EDS = +E+D-N or NED -> -Z+X+Y (Processing's default left-handed)",
        {
            {  0,  1,  0,  0 },
            {  0,  0,  1,  0 },
            { -1,  0,  0,  0 },
            {  0,  0,  0,  1 },
        }
    },
    { "XYZ = EUS = +E-D-N or NED -> -Z+X-Y (Godot's \"North = -Z\")",
    {
        {  0,  1,  0,  0 },
        {  0,  0, -1,  0 },
        { -1,  0,  0,  0 },
        {  0,  0,  0,  1 },
    }
},

};


PostProcessingForm::PostProcessingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PostProcessingForm)
{
    ui->setupUi(this);

    QSettings settings;

    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Fallback", "900").toDouble());
    ui->lineEdit_TagIndicatingBeginningOfNewObject->setText(settings.value("PostProcessing_TagIndicatingBeginningOfNewObject", "New object").toString());
    ui->lineEdit_TagIndicatingBeginningOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingBeginningOfObjectPoints", "LMB").toString());
    ui->lineEdit_TagIndicatingEndOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingEndOfObjectPoints", "RMB").toString());

    ui->spinBox_MaxLogLines->setValue(settings.value("PostProcessing_MaxLogLines", "1000").toInt());

    ui->checkBox_Stylus_PointCloud_IncludeNormals->setChecked(settings.value("PostProcessing_Stylus_PointCloud_IncludeNormals", false).toBool());

    ui->spinBox_ExpectedITOWAlignment->setValue(settings.value("PostProcessing_ExpectedITOWAlignment", "125").toInt());
    ui->spinBox_ITOWAutoAlignThreshold->setValue(settings.value("PostProcessing_ITOWAutoAlignThreshold", "5").toInt());
    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Correction", "0").toDouble());
    ui->checkBox_ReportITOWAutoAlign->setChecked(settings.value("PostProcessing_ReportITOWAutoAlign", false).toBool());
    ui->checkBox_ReportMissingITOWs->setChecked(settings.value("PostProcessing_ReportMissingITOWs", false).toBool());
    ui->checkBox_ReportUnalignedITOWS->setChecked(settings.value("PostProcessing_ReportUnalignedITOWS", false).toBool());

    ui->doubleSpinBox_Translation_N->setValue(settings.value("PostProcessing_Translation_N", "0").toDouble());
    ui->doubleSpinBox_Translation_E->setValue(settings.value("PostProcessing_Translation_E", "0").toDouble());
    ui->doubleSpinBox_Translation_D->setValue(settings.value("PostProcessing_Translation_D", "0").toDouble());

    ui->doubleSpinBox_Stylus_Movie_Camera_N->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_N", "-1").toDouble());
    ui->doubleSpinBox_Stylus_Movie_Camera_E->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_E", "0").toDouble());
    ui->doubleSpinBox_Stylus_Movie_Camera_D->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_D", "-0.05").toDouble());

    ui->doubleSpinBox_Stylus_Movie_LookAt_N->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_N", "0").toDouble());
    ui->doubleSpinBox_Stylus_Movie_LookAt_E->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_E", "0").toDouble());
    ui->doubleSpinBox_Stylus_Movie_LookAt_D->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_D", "-0.05").toDouble());

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            QString settingKey = "PostProcessing_Transform_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            QString defaultValue = "0";

            if (row == column)
            {
                defaultValue = "1";
            }

            ui->tableWidget_TransformationMatrix->item(row, column)->setText(settings.value(settingKey, defaultValue).toString());
        }
    }

    // Just some valid values
    const double defaultAntennaLocations[3][3] = {
        { 1, 0, 0 },
        { -1, -1, 0 },
        { -1, 1, 0 }
    };

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            QString settingKey = "PostProcessing_AntennaLocations_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            QString defaultValue = QString::number(defaultAntennaLocations[row][column], 'g', 3);

            ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->setText(settings.value(settingKey, defaultValue).toString());
        }
    }

    ui->doubleSpinBox_Stylus_Movie_FPS->setValue(settings.value("PostProcessing_Stylus_Movie_FPS", "30").toDouble());

    ui->plainTextEdit_LOSolver_TransformMatrixScript->setPlainText(settings.value("PostProcessing_LOSolver_TransformMatrixScript", ui->plainTextEdit_LOSolver_TransformMatrixScript->toPlainText()).toString());
    ui->comboBox_LOSolver_Movie_TimeStamps->setCurrentIndex(settings.value("PostProcessing_LOSolver_Movie_Timestamps", ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex()).toInt());

    ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->setPlainText(settings.value("PostProcessing_Lidar_TransformMatrixScript_BeforeRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->toPlainText()).toString());
    ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->setPlainText(settings.value("PostProcessing_Lidar_TransformMatrixScript_AfterRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->toPlainText()).toString());

    ui->checkBox_Lidar_PointCloud_IncludeNormals->setChecked(settings.value("PostProcessing_Lidar_PointCloud_IncludeNormals").toBool());

    ui->spinBox_Lidar_TimeShift->setValue(settings.value("PostProcessing_Lidar_TimeShift", "80").toInt());

    ui->doubleSpinBox_Lidar_Filtering_StartAngle->setValue(settings.value("PostProcessing_Lidar_Filtering_StartAngle", "90").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_EndAngle->setValue(settings.value("PostProcessing_Lidar_Filtering_EndAngle", "270").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_Quality_Pre->setValue(settings.value("PostProcessing_Lidar_Filtering_Quality_Pre", "0.5").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_Quality_Post->setValue(settings.value("PostProcessing_Lidar_Filtering_Quality_Post", "0.5").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Near->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceLimit_Near", "0.1").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Far->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceLimit_Far", "5").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceDeltaLimit->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceDeltaLimit", "0").toDouble());
    ui->doubleSpinBox_Lidar_Filtering_RelativeDistanceSlopeLimit->setValue(settings.value("PostProcessing_Lidar_Filtering_RelativeDistanceSlopeLimit", "0").toDouble());

    ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_N", "0").toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_E", "0").toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_D", "0").toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Radius->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Radius", "100000000").toDouble());
}

PostProcessingForm::~PostProcessingForm()
{
    QSettings settings;

    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Fallback", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfNewObject", ui->lineEdit_TagIndicatingBeginningOfNewObject->text());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfObjectPoints", ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text());
    settings.setValue("PostProcessing_TagIndicatingEndOfObjectPoints", ui->lineEdit_TagIndicatingEndOfObjectPoints->text());

    settings.setValue("PostProcessing_MaxLogLines", ui->spinBox_MaxLogLines->value());

    settings.setValue("PostProcessing_Stylus_PointCloud_IncludeNormals", ui->checkBox_Stylus_PointCloud_IncludeNormals->isChecked());

    settings.setValue("PostProcessing_ExpectedITOWAlignment", ui->spinBox_ExpectedITOWAlignment->value());
    settings.setValue("PostProcessing_ITOWAutoAlignThreshold", ui->spinBox_ITOWAutoAlignThreshold->value());
    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Correction", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value());
    settings.setValue("PostProcessing_ReportITOWAutoAlign", ui->checkBox_ReportITOWAutoAlign->isChecked());
    settings.setValue("PostProcessing_ReportMissingITOWs", ui->checkBox_ReportMissingITOWs->isChecked());
    settings.setValue("PostProcessing_ReportUnalignedITOWS", ui->checkBox_ReportUnalignedITOWS->isChecked());

    settings.setValue("PostProcessing_Translation_N", ui->doubleSpinBox_Translation_N->value());
    settings.setValue("PostProcessing_Translation_E", ui->doubleSpinBox_Translation_E->value());
    settings.setValue("PostProcessing_Translation_D", ui->doubleSpinBox_Translation_D->value());

    settings.setValue("PostProcessing_Stylus_Movie_Camera_N", ui->doubleSpinBox_Stylus_Movie_Camera_N->value());
    settings.setValue("PostProcessing_Stylus_Movie_Camera_E", ui->doubleSpinBox_Stylus_Movie_Camera_E->value());
    settings.setValue("PostProcessing_Stylus_Movie_Camera_D", ui->doubleSpinBox_Stylus_Movie_Camera_D->value());

    settings.setValue("PostProcessing_Stylus_Movie_LookAt_N", ui->doubleSpinBox_Stylus_Movie_LookAt_N->value());
    settings.setValue("PostProcessing_Stylus_Movie_LookAt_E", ui->doubleSpinBox_Stylus_Movie_LookAt_E->value());
    settings.setValue("PostProcessing_Stylus_Movie_LookAt_D", ui->doubleSpinBox_Stylus_Movie_LookAt_D->value());

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            QString settingKey = "PostProcessing_Transform_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            settings.setValue(settingKey, ui->tableWidget_TransformationMatrix->item(row, column)->text());
        }
    }

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            QString settingKey = "PostProcessing_AntennaLocations_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            settings.setValue(settingKey, ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->text());
        }
    }

    settings.setValue("PostProcessing_Stylus_Movie_FPS", ui->doubleSpinBox_Stylus_Movie_FPS->value());

    // NOTE: Directories to log files are saved in syncLogFileDialogDirectories-function "on the fly"

    settings.setValue("PostProcessing_Directory_Dialog_Transformation_Load", fileDialog_Transformation_Load.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_Transformation_Save", fileDialog_Transformation_Save.directory().path());

    settings.setValue("PostProcessing_Directory_Dialog_AntennaLocations_Load", fileDialog_AntennaLocations_Load.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_AntennaLocations_Save", fileDialog_AntennaLocations_Save.directory().path());

    settings.setValue("PostProcessing_Directory_Dialog_PointCloud", fileDialog_PointCloud.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_Stylus_MovieScript", fileDialog_Stylus_MovieScript.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_LOSolver_Script", fileDialog_LOSolver_Script.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_Lidar_Script", fileDialog_Lidar_Script.directory().path());

    settings.setValue("PostProcessing_Directory_Dialog_Operations_Load", fileDialog_Operations_Load.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_Operations_Save", fileDialog_Operations_Save.directory().path());

    settings.setValue("PostProcessing_LOSolver_TransformMatrixScript", ui->plainTextEdit_LOSolver_TransformMatrixScript->toPlainText());
    settings.setValue("PostProcessing_LOSolver_Movie_Timestamps", ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex());

    settings.setValue("PostProcessing_Lidar_TransformMatrixScript_BeforeRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->toPlainText());
    settings.setValue("PostProcessing_Lidar_TransformMatrixScript_AfterRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->toPlainText());

    settings.setValue("PostProcessing_Lidar_PointCloud_IncludeNormals", ui->checkBox_Lidar_PointCloud_IncludeNormals->checkState() == Qt::Checked);

    settings.setValue("PostProcessing_Lidar_TimeShift", ui->spinBox_Lidar_TimeShift->value());

    settings.setValue("PostProcessing_Lidar_Filtering_StartAngle", ui->doubleSpinBox_Lidar_Filtering_StartAngle->value());
    settings.setValue("PostProcessing_Lidar_Filtering_EndAngle", ui->doubleSpinBox_Lidar_Filtering_EndAngle->value());
    settings.setValue("PostProcessing_Lidar_Filtering_Quality_Pre", ui->doubleSpinBox_Lidar_Filtering_Quality_Pre->value());
    settings.setValue("PostProcessing_Lidar_Filtering_Quality_Post", ui->doubleSpinBox_Lidar_Filtering_Quality_Post->value());
    settings.setValue("PostProcessing_Lidar_Filtering_DistanceLimit_Near", ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Near->value());
    settings.setValue("PostProcessing_Lidar_Filtering_DistanceLimit_Far", ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Far->value());
    settings.setValue("PostProcessing_Lidar_Filtering_DistanceDeltaLimit", ui->doubleSpinBox_Lidar_Filtering_DistanceDeltaLimit->value());
    settings.setValue("PostProcessing_Lidar_Filtering_RelativeDistanceSlopeLimit", ui->doubleSpinBox_Lidar_Filtering_RelativeDistanceSlopeLimit->value());

    settings.setValue("PostProcessing_Lidar_BoundingSphere_Center_N", ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value());
    settings.setValue("PostProcessing_Lidar_BoundingSphere_Center_E", ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value());
    settings.setValue("PostProcessing_Lidar_BoundingSphere_Center_D", ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value());
    settings.setValue("PostProcessing_Lidar_BoundingSphere_Radius", ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value());

    delete ui;
}

void PostProcessingForm::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    if (!onShowInitializationsDone)
    {
        fileDialog_UBX.setFileMode(QFileDialog::ExistingFiles);

        QStringList roverFilters;

        roverFilters << "UBX log files (*.ubx)"
                << "Raw log files (*.raw)"
                << "Any files (*)";

        fileDialog_UBX.setNameFilters(roverFilters);

        fileDialog_Tags.setFileMode(QFileDialog::ExistingFiles);

        QStringList tagFilters;

        tagFilters << "Tag-files (*.tags)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Tags.setNameFilters(tagFilters);

        fileDialog_Distances.setFileMode(QFileDialog::ExistingFiles);

        QStringList distanceFilters;

        distanceFilters << "Distance-files (*.distances)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Distances.setNameFilters(distanceFilters);

        fileDialog_Sync.setFileMode(QFileDialog::ExistingFiles);

        QStringList syncFilters;

        syncFilters << "Sync-files (*.sync)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Sync.setNameFilters(syncFilters);

        fileDialog_Lidar.setFileMode(QFileDialog::ExistingFiles);

        QStringList lidarFilters;

        lidarFilters << "Lidar-files (*.lidar)"
                << "Any files (*)";

        fileDialog_Lidar.setNameFilters(lidarFilters);

        fileDialog_All.setFileMode(QFileDialog::ExistingFiles);

        QStringList allFilesFilters;

        allFilesFilters << "UBX log files (*.ubx)"
                << "Raw log files (*.raw)"
                << "Tag-files (*.tags)"
                << "Txt-files (*.txt)"
                << "Sync-files (*.sync)"
                << "lidar-files (*.lidar)"
                << "Any files (*)";

        fileDialog_All.setNameFilters(allFilesFilters);

        fileDialog_PointCloud.setFileMode(QFileDialog::Directory);

        fileDialog_Stylus_MovieScript.setFileMode(QFileDialog::AnyFile);
        fileDialog_Stylus_MovieScript.setDefaultSuffix("MovieScript");

        QStringList movieScriptFilters;

        movieScriptFilters << "moviescript files (*.moviescript)"
                << "Any files (*)";

        fileDialog_Stylus_MovieScript.setNameFilters(movieScriptFilters);


        fileDialog_Transformation_Load.setFileMode(QFileDialog::ExistingFile);

        QStringList transformationFilters;

        transformationFilters << "Transformation files (*.Transformation)"
                << "Any files (*)";

        fileDialog_Transformation_Load.setNameFilters(transformationFilters);


        fileDialog_Transformation_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_Transformation_Save.setDefaultSuffix("Transformation");

        fileDialog_Transformation_Save.setNameFilters(transformationFilters);


        fileDialog_AntennaLocations_Load.setFileMode(QFileDialog::ExistingFile);

        QStringList antennaLocationsFilters;

        antennaLocationsFilters << "Antenna locations-files (*.AntennaLocations)"
                << "Any files (*)";

        fileDialog_AntennaLocations_Load.setNameFilters(antennaLocationsFilters);


        fileDialog_AntennaLocations_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_AntennaLocations_Save.setDefaultSuffix("AntennaLocations");

        fileDialog_AntennaLocations_Save.setNameFilters(antennaLocationsFilters);


        fileDialog_LOSolver_Script.setFileMode(QFileDialog::AnyFile);
        fileDialog_LOSolver_Script.setDefaultSuffix("LOScript");

        QStringList loScriptFilters;

        loScriptFilters << "location/orientation script files (*.loscript)"
                << "Any files (*)";

        fileDialog_LOSolver_Script.setNameFilters(loScriptFilters);


        fileDialog_Lidar_Script.setFileMode(QFileDialog::AnyFile);
        fileDialog_Lidar_Script.setDefaultSuffix("LidarScript");

        QStringList lidarScriptFilters;

        lidarScriptFilters << "lidar script files (*.lidarscript)"
                << "Any files (*)";

        fileDialog_Lidar_Script.setNameFilters(lidarScriptFilters);


        fileDialog_Operations_Load.setFileMode(QFileDialog::ExistingFile);

        QStringList OperationsFilters;

        OperationsFilters << "Operations-files (*.Operations)"
                << "Any files (*)";

        fileDialog_Operations_Load.setNameFilters(OperationsFilters);

        fileDialog_Operations_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_Operations_Save.setDefaultSuffix("Operations");

        fileDialog_Operations_Save.setNameFilters(OperationsFilters);



        for (unsigned int presetIndex = 0; presetIndex < (sizeof(transformationPresets) / sizeof(transformationPresets[0])); presetIndex++)
        {
            ui->comboBox_Presets->addItem(transformationPresets[presetIndex].name);
        }

        onShowInitializationsDone = true;
    }

    QSettings settings;

    fileDialog_All.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Logs").toString()));

    syncLogFileDialogDirectories(fileDialog_All.directory().path(), false);
    fileDialog_Transformation_Load.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Transformation_Load").toString()));
    fileDialog_Transformation_Save.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Transformation_Save").toString()));

    fileDialog_AntennaLocations_Load.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_AntennaLocations_Load").toString()));
    fileDialog_AntennaLocations_Save.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_AntennaLocations_Save").toString()));

    fileDialog_PointCloud.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_PointCloud").toString()));
    fileDialog_Stylus_MovieScript.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Stylus_MovieScript").toString()));
    fileDialog_LOSolver_Script.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_LOSolver_Script").toString()));
    fileDialog_Lidar_Script.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Lidar_Script").toString()));

    fileDialog_Operations_Load.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Operations_Load").toString()));
    fileDialog_Operations_Save.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Operations_Save").toString()));
}

void PostProcessingForm::addLogLine(const QString& line)
{
    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Log->setMaximumBlockCount(ui->spinBox_MaxLogLines->value());
//    ui->plainTextEdit_Log->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Log->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Log->appendPlainText(timeString + ": " + line);

    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverA_clicked()
{
    rovers[0].relposnedMessages.clear();
    addLogLine("Rover A RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverB_clicked()
{
    rovers[1].relposnedMessages.clear();
    addLogLine("Rover B RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverC_clicked()
{
    rovers[2].relposnedMessages.clear();
    addLogLine("Rover C RELPOSNED-data cleared.");
}


void PostProcessingForm::on_pushButton_ClearTagData_clicked()
{
    tags.clear();
    addLogLine("Tag data cleared.");
}

void PostProcessingForm::addRELPOSNEDData_Rover(const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        if (fileDialog_UBX.exec())
        {
            QStringList fileNames = fileDialog_UBX.selectedFiles();

            if (fileNames.size() != 0)
            {
                fileDialog_UBX.setDirectory(QFileInfo(fileNames[0]).path());
                syncLogFileDialogDirectories(fileDialog_UBX.directory().path(), true);
            }

            addRELPOSNEDData_Rover(fileNames, roverId);
        }
    }
}

void PostProcessingForm::addRELPOSNEDData_Rover(const QStringList fileNames, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        addLogLine("Reading files into rover " + getRoverIdentString(roverId) + " relposned-data...");

        currentRELPOSNEDReadingData.relposnedMessages =  &rovers[roverId].relposnedMessages;

        addRELPOSNEDFileData(fileNames);

        currentRELPOSNEDReadingData.relposnedMessages = nullptr;
    }
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverA_clicked()
{
    addRELPOSNEDData_Rover(0);
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverB_clicked()
{
    addRELPOSNEDData_Rover(1);
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverC_clicked()
{
    addRELPOSNEDData_Rover(2);
}

void PostProcessingForm::addRELPOSNEDFileData(const QStringList& fileNames)
{
    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile ubxFile;
        ubxFile.setFileName(fileName);
        if (ubxFile.open(QIODevice::ReadOnly))
        {
            QDataStream stream(&ubxFile);

            int64_t fileLength = ubxFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
            }
            else
            {
                // Interpreting of the RELPOSNED-data is done using UBloxDataStreamProcessor.
                // Therefore the actual messages are received using slots.

                auto fileData = std::make_unique<unsigned char[]>(static_cast<std::size_t>(fileLength));

                stream.readRawData(reinterpret_cast<char *>(fileData.get()), static_cast<int>(fileLength));

                UBloxDataStreamProcessor ubloxProcessor;

                currentRELPOSNEDReadingData.init();

                QObject::connect(&ubloxProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                                 this, SLOT(ubloxProcessor_nmeaSentenceReceived(const NMEAMessage&)));

                QObject::connect(&ubloxProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                                 this, SLOT(ubloxProcessor_ubxMessageReceived(const UBXMessage&)));

                QObject::connect(&ubloxProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                                 this, SLOT(ubloxProcessor_rtcmMessageReceived(const RTCMMessage&)));

                QObject::connect(&ubloxProcessor, SIGNAL(ubxParseError(const QString&)),
                                 this, SLOT(ubloxProcessor_ubxParseError(const QString&)));

                QObject::connect(&ubloxProcessor, SIGNAL(nmeaParseError(const QString&)),
                                 this, SLOT(ubloxProcessor_nmeaParseError(const QString&)));

                QObject::connect(&ubloxProcessor, SIGNAL(unidentifiedDataReceived(const QByteArray&)),
                                 this, SLOT(ubloxProcessor_unidentifiedDataReceived(const QByteArray&)));

                for (currentRELPOSNEDReadingData.currentFileByteIndex = 0;
                     currentRELPOSNEDReadingData.currentFileByteIndex < fileLength;
                     currentRELPOSNEDReadingData.currentFileByteIndex++)
                {
                    // Handle data byte-by-byte using UBloxDataStreamProcessor.
                    // It will send signals when necessary
                    ubloxProcessor.process(static_cast<char>(fileData[static_cast<std::size_t>(currentRELPOSNEDReadingData.currentFileByteIndex)]), 0);
                }

                if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
                {
                    addLogLine("Warning: Duplicate iTOWS found at the end of file. Number of messages: " + QString::number(currentRELPOSNEDReadingData.duplicateITOWCounter) +
                               ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOW) + "..." +
                               QString::number(currentRELPOSNEDReadingData.lastReadITOW) +
                               ". Bytes " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex) +
                               "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex) +
                               ". Only previous messages preserved.");

                    currentRELPOSNEDReadingData.firstDuplicateITOW = -1;
                    currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = -1;
                    currentRELPOSNEDReadingData.duplicateITOWCounter = 0;
                }

                unsigned int numOfUnprocessedBytes = ubloxProcessor.getNumOfUnprocessedBytes();

                if (numOfUnprocessedBytes != 0)
                {
                    addLogLine("Warning: Unprocessed bytes at the end of the file: " + QString::number(numOfUnprocessedBytes));
                }

                addLogLine("File \"" + fileInfo.fileName() + "\" processed. Message counts: " +
                           "RELPOSNED: " + QString::number(currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_Total) +
                           " (" + QString::number(currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_UniqueITOWs) + " unique iTOWS)" +
                           ", UBX: " + QString::number(currentRELPOSNEDReadingData.messageCount_UBX) +
                           ", NMEA: " + QString::number(currentRELPOSNEDReadingData.messageCount_NMEA) +
                           ", RTCM: " + QString::number(currentRELPOSNEDReadingData.messageCount_RTCM) +
                           ". Discarded bytes: " + QString::number(currentRELPOSNEDReadingData.discardedBytesCount) +
                           " (" + QString::number(currentRELPOSNEDReadingData.discardedBytesCount * 100. / fileLength) + "%).");
            }

            ubxFile.close();
        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}

void PostProcessingForm::RELPOSNEDReadingData::init()
{
    messageCount_UBX = 0;
    messageCount_NMEA = 0;
    messageCount_RTCM = 0;
    messageCount_UBX_RELPOSNED_Total = 0;
    messageCount_UBX_RELPOSNED_UniqueITOWs = 0;

    lastReadITOW = -1;
    firstDuplicateITOW = -1;
    firstDuplicateITOWByteIndex = -1;
    duplicateITOWCounter = 0;

    currentFileByteIndex = 0;
    lastHandledDataByteIndex = 0;
    discardedBytesCount = 0;
}

void PostProcessingForm::ubloxProcessor_nmeaSentenceReceived(const NMEAMessage& nmeaSentence)
{
    // NMEA-messages are not utilized, but count them anyway
    Q_UNUSED(nmeaSentence);
    currentRELPOSNEDReadingData.messageCount_NMEA++;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    currentRELPOSNEDReadingData.messageCount_UBX++;

    unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        // Casting of UBX-message to RELPOSNED was successful

        if ((currentRELPOSNEDReadingData.lastReadITOW != -1) && ((relposned.iTOW % expectedITOWAlignment) != 0))
        {
            unsigned int iTOWautoAlignThreshold = ui->spinBox_ITOWAutoAlignThreshold->value();
            int autoAlignedITOW = 0;
            bool iTOWAutoAligned = false;

            if ((relposned.iTOW % expectedITOWAlignment) <= iTOWautoAlignThreshold)
            {
                // Negative correction (f. exp 1001 > 1000)
                autoAlignedITOW = relposned.iTOW - (relposned.iTOW % expectedITOWAlignment);
                iTOWAutoAligned = true;
            }
            else if ((expectedITOWAlignment - (relposned.iTOW % expectedITOWAlignment)) <= iTOWautoAlignThreshold)
            {
                // Positive correction (f. exp 999 > 1000)
                autoAlignedITOW = relposned.iTOW - (relposned.iTOW % expectedITOWAlignment) + expectedITOWAlignment;
                iTOWAutoAligned = true;
            }

            if (iTOWAutoAligned)
            {
                if (ui->checkBox_ReportITOWAutoAlign->isChecked())
                {
                    addLogLine("Warning: iTOW auto-aligned to expected interval (" +
                               QString::number(expectedITOWAlignment) +" ms). original iTOW: " + QString::number(relposned.iTOW) +
                               ", auto-aligned: " + QString::number(autoAlignedITOW) +
                               " (adjustment: " + QString::number(int(autoAlignedITOW) - int(relposned.iTOW)) + ")"
                               ". Bytes " + QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1) +
                               "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex));
                }

                relposned.iTOW = autoAlignedITOW;
            }
            else
            {
                if (ui->checkBox_ReportUnalignedITOWS->isChecked())
                {
                    addLogLine("Warning: iTOW not aligned or auto-alignable to expected interval (" +
                               QString::number(expectedITOWAlignment) +" ms). iTOW: " + QString::number(relposned.iTOW) +
                               ". Bytes " + QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1) +
                               "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex));
                }
            }
        }

        if ((ui->checkBox_ReportMissingITOWs->isChecked()) &&
                ((currentRELPOSNEDReadingData.lastReadITOW != -1) &&
                 (static_cast<unsigned int>(relposned.iTOW - currentRELPOSNEDReadingData.lastReadITOW) > expectedITOWAlignment)))
        {
            int missingITOWS = (relposned.iTOW - currentRELPOSNEDReadingData.lastReadITOW - 1) / expectedITOWAlignment;

            addLogLine("Warning: iTOWs not consecutive with expected interval (" +
                       QString::number(expectedITOWAlignment) +" ms). Number of missing iTOWs: " + QString::number(missingITOWS) +
                       ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.lastReadITOW + 1) + "..." +
                       QString::number(relposned.iTOW - 1) +
                       ". Bytes " + QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1) +
                       "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex));
        }

        currentRELPOSNEDReadingData.lastReadITOW = relposned.iTOW;
        currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_Total++;

        if (currentRELPOSNEDReadingData.relposnedMessages->find(relposned.iTOW) != currentRELPOSNEDReadingData.relposnedMessages->end())
        {
            // RELPOSNED-message with the same iTOW already existed
            if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
            {
                // This was not the first already existing RELPOSNED-message with duplicate iTOW -> Increase counter
                currentRELPOSNEDReadingData.duplicateITOWCounter++;
            }
            else
            {
                // This is the first RELPOSNED-message with duplicate iTOW -> Store starting values
                currentRELPOSNEDReadingData.firstDuplicateITOW = relposned.iTOW;
                currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1;
                currentRELPOSNEDReadingData.duplicateITOWCounter = 1;
            }
        }
        else
        {
            if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
            {
                // Duplicate iTOW(s) were found before this message
                addLogLine("Warning: Duplicate iTOWS found. Number of messages: " + QString::number(currentRELPOSNEDReadingData.duplicateITOWCounter) +
                           ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOW) + "..." +
                           QString::number(relposned.iTOW - 1) +
                           ". Bytes " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex) +
                           "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex) +
                           ". Only previous messages preserved.");

                currentRELPOSNEDReadingData.firstDuplicateITOW = -1;
                currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = -1;
                currentRELPOSNEDReadingData.duplicateITOWCounter = 0;
            }

            currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_UniqueITOWs++;
            if (currentRELPOSNEDReadingData.relposnedMessages)
            {
                currentRELPOSNEDReadingData.relposnedMessages->operator[](relposned.iTOW) = relposned;
            }
        }
    }

    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_rtcmMessageReceived(const RTCMMessage& rtcmMessage)
{
    // RTCM-messages are not utilized, but count them anyway
    Q_UNUSED(rtcmMessage);
    currentRELPOSNEDReadingData.messageCount_RTCM++;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_ubxParseError(const QString& errorString)
{
    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: UBX parse error: \"" + errorString + "\". " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_nmeaParseError(const QString& errorString)
{
    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: NMEA parse error: \"" + errorString + "\". " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_unidentifiedDataReceived(const QByteArray& data)
{
    Q_UNUSED(data);

    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: Unidentified data. " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}



void PostProcessingForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Log->clear();
}

void PostProcessingForm::addTagData(const QStringList& fileNames)
{
    addLogLine("Reading tags...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile tagFile;
        tagFile.setFileName(fileName);
        if (tagFile.open(QIODevice::ReadOnly))
        {
            int numberOfTags = 0;

            QTextStream textStream(&tagFile);

            int64_t fileLength = tagFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                tagFile.close();
                continue;
            }

            QString headerLine = textStream.readLine();

            bool uptimeColumnExists;

            if (!headerLine.compare("Time\tiTOW\tTag\tText", Qt::CaseInsensitive))
            {
                addLogLine("Warning: File's \"" + fileInfo.fileName() + "\" doesn't have \"Uptime\"-column (old format). Using iTOWS as uptimes. Distances and sync-data may not be valid.");
                uptimeColumnExists = false;
            }
            else if (!headerLine.compare("Time\tiTOW\tTag\tText\tUptime", Qt::CaseInsensitive))
            {
                uptimeColumnExists = true;
            }
            else
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have supported header. Skipped.");
                tagFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;
            int firstDuplicateTagLine = 0;
            int lastDuplicateTagLine = 0;

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if ((subItems.count() < 4) ||
                        ((subItems.count() < 5) && uptimeColumnExists))
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                Tag newTag;

                bool iTOWConvOk;
                newTag.iTOW = subItems[1].toInt(&iTOWConvOk);

                if (!iTOWConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 2 (iTOW) to integer. Line skipped.");
                    continue;
                }

                if (subItems[2].length() == 0)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Empty tag. Line skipped.");
                    continue;
                }

                bool uptimeConvOk;
                qint64 uptime;

                if (uptimeColumnExists)
                {
                    uptime = subItems[4].toLongLong(&uptimeConvOk);

                    if (!uptimeConvOk)
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (uptime) to 64-bit integer. Line skipped.");
                        continue;
                    }
                }
                else
                {
                    // Old format -> Use iTOW as uptime
                    uptime = newTag.iTOW;
                }

                newTag.sourceFile = fileName;
                newTag.sourceFileLine = lineNumber;
                newTag.ident = subItems[2];
                newTag.text = subItems[3];

                if (tags.find(uptime) != tags.end())
                {
                    QList<Tag> simultaneousItems = tags.values(uptime);

                    bool skip = false;

                    for (int i = 0; i < simultaneousItems.size(); i++)
                    {
                        if (simultaneousItems.at(i).ident == newTag.ident)
                        {
                            discardedLines++;
                            if (!firstDuplicateTagLine)
                            {
                                firstDuplicateTagLine = lineNumber;
                            }

                            lastDuplicateTagLine = lineNumber;
                            skip = true;
                            continue;
                        }
                    }

                    if (skip)
                    {
                        continue;
                    }
                }

                if (firstDuplicateTagLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateTagLine) + "-" +
                               QString::number(lastDuplicateTagLine) +
                               ": Duplicate tag(s). Line(s) skipped.");

                    firstDuplicateTagLine = 0;
                }

                // QT pre 5.15: tags.insertMulti(uptime, newTag);
                tags.insert(uptime, newTag);

                numberOfTags ++;
            }

            if (firstDuplicateTagLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateTagLine) + "-" +
                           QString::number(lastDuplicateTagLine) +
                           ": Duplicate tag(s). Line(s) skipped.");

                firstDuplicateTagLine = 0;
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid tags: " +
                       QString::number(numberOfTags) +
                       ", total lines: " + QString::number(lineNumber) +
                       ", discarded lines: " + QString::number(discardedLines) + ".");
        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}


void PostProcessingForm::on_pushButton_AddTagData_clicked()
{
    if (fileDialog_Tags.exec())
    {
        QStringList fileNames = fileDialog_Tags.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Tags.setDirectory(QFileInfo(fileNames[0]).path());
            syncLogFileDialogDirectories(fileDialog_Tags.directory().path(), true);
        }

        addTagData(fileNames);
    }

}

void PostProcessingForm::on_pushButton_Stylus_GeneratePointClouds_clicked()
{
    generatePointClouds(SOURCE_LASERDISTANCEMETER_OR_CONSTANT);
}

void PostProcessingForm::generatePointClouds(const PointCloudDistanceSource source)
{
    Eigen::Transform<double, 3, Eigen::Affine> transform_NEDToXYZ;
    Eigen::Transform<double, 3, Eigen::Affine> transform_Lidar_Generated_BeforeRotation;
    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarGenerated_AfterRotation;
    LOInterpolator loInterpolator_Lidar(this);
    RPLidarPlausibilityFilter::Settings lidarFilteringSettings;

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    switch (source)
    {
    case SOURCE_LASERDISTANCEMETER_OR_CONSTANT:
        break;

    case SOURCE_LIDAR:
    {
        if (!generateLidarTransformMatrices(transform_Lidar_Generated_BeforeRotation, transform_LidarGenerated_AfterRotation))
        {
            return;
        }

        if (!updateLOSolverReferencePointLocations(loInterpolator_Lidar.loSolver))
        {
            return;
        }

        getLidarFilteringSettings(lidarFilteringSettings);

        break;
    }
    default:
        Q_ASSERT(false);
        // addLogLine("Unsupported point cloud distance source (not implemented in SW so fix it!)");
        break;
    }

    if (fileDialog_PointCloud.exec())
    {
        QDir dir = fileDialog_PointCloud.directory();

        if (!dir.exists())
        {
            addLogLine("Error: Directory \"" + dir.path() + "\" doesn't exist. Point cloud files not created.");
            return;
        }

        fileDialog_PointCloud.setDirectory(dir);

        addLogLine("Processing...");

        // Some locals to prevent excessive typing:
        QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();

//        QMultiMap<qint64, Tag_New>::const_iterator currentTagIterator;

        bool objectActive = false;

        qint64 beginningUptime = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QFile* outFile = nullptr;
        QTextStream* outStream = nullptr;

        qint64 uptime = -1;
        Tag beginningTag;

        while (tags.upperBound(uptime) != tags.end())
        {
            uptime = tags.upperBound(uptime).key();

            QList<Tag> tagItems = tags.values(uptime);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const Tag& currentTag = tagItems[i];

                if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
                {
                    // Tag type: new object

                    if (objectActive)
                    {
                        // Object already active -> Close existing stream and file

                        if (outStream)
                        {
                            delete outStream;
                            outStream = nullptr;
                        }
                        if (outFile)
                        {
                            addLogLine("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsWritten));
                            outFile->close();
                            delete outFile;
                            outFile = nullptr;
                        }
                        objectActive = false;
                    }

                    if (currentTag.text.length() == 0)
                    {
                        // Empty name for the new object not allowed

                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": New object without a name. Ending previous object, but not beginning new nor creating a new file. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        continue;
                    }

                    QString fileName = QDir::cleanPath(dir.path() + "/" + currentTag.text + ".xyz");

                    outFile = new QFile(fileName);

                    if (outFile->exists())
                    {
                        // File already exists -> Not allowed

                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        delete outFile;
                        outFile = nullptr;
                        continue;
                    }

                    addLogLine("Creating file \"" + fileName + "\"...");

                    if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text))
                    {
                        // Creating the file failed

                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": File \"" + fileName + "\" can't be created. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        delete outFile;
                        outFile = nullptr;
                        continue;
                    }

                    outStream = new QTextStream(outFile);
                    objectActive = true;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                    pointsWritten = 0;
                }
                else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: Begin points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Duplicate beginning tag. Skipped.");
                        continue;
                    }

                    // Just store the beginning uptime-value and tag. Writing of the points is done in ending tag-branch
                    beginningUptime = uptime;
                    beginningTag = currentTag;
                }
                else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: end points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag without beginning tag. Skipped.");
                        continue;
                    }

                    const Tag& endingTag = currentTag;

                    if (endingTag.sourceFile != beginningTag.sourceFile)
                    {
                        addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                                   beginningTag.sourceFile + "\", line " +
                                   QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                                   endingTag.sourceFile + "\", line " +
                                   QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                        continue;
                    }

                    bool generatingOk = false;
                    int prevPointsWritten = pointsWritten;

                    switch (source)
                    {
                    case SOURCE_LASERDISTANCEMETER_OR_CONSTANT:
                        generatingOk = generatePointCloudPointSet_Stylus(beginningTag, endingTag, beginningUptime, uptime, outStream, transform_NEDToXYZ, pointsWritten);
                        break;

                    case SOURCE_LIDAR:
                        generatingOk = generatePointCloudPointSet_Lidar(beginningTag, endingTag, beginningUptime, uptime, outStream, transform_NEDToXYZ, transform_Lidar_Generated_BeforeRotation, transform_LidarGenerated_AfterRotation, loInterpolator_Lidar, lidarFilteringSettings, pointsWritten);
                        break;

                    default:
                        Q_ASSERT(false);
                        //addLogLine("Unsupported point cloud distance source (not implemented in SW so fix it!)");
                        break;
                    }

                    if (generatingOk)
                    {
                        int pointsBetweenTags = pointsWritten - prevPointsWritten;

                        if (pointsBetweenTags == 0)
                        {
                            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                                       QString::number(beginningTag.sourceFileLine) +
                                       ", uptime " + QString::number(beginningUptime) +
                                       ", iTOW " + QString::number(beginningTag.iTOW) + ", ending tag line " +
                                       QString::number(endingTag.sourceFileLine) +
                                       ", uptime " + QString::number(endingTag.iTOW) +
                                       ", iTOW " + QString::number(endingTag.iTOW) +
                                       ", File \"" + endingTag.sourceFile + "\""
                                       " No points between tags.");
                        }
                    }

                    beginningUptime = -1;
                }
            }
        }

        if (beginningUptime != -1)
        {
            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
                       QString::number(beginningTag.sourceFileLine) +
                       ", iTOW " + QString::number(beginningUptime) +
                       ", iTOW " + QString::number(beginningTag.iTOW) +
                       " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
        }

        if (outStream)
        {
            delete outStream;
        }

        if (outFile)
        {
            addLogLine("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsWritten));
            outFile->close();
            delete outFile;
        }

        addLogLine("Point cloud files generated.");
    }
}


void PostProcessingForm::on_pushButton_StartReplay_clicked()
{
    bool convOk;
    firstUptimeToReplay = ui->lineEdit_Uptime_Min->text().toLongLong(&convOk);
    lastReplayedUptime_ms = firstUptimeToReplay - 1;
    if (!convOk)
    {
        addLogLine("Invalid uptime range for replay, min.");
        ui->lineEdit_Uptime_Min->setFocus();
        return;
    }

    lastUptimeToReplay = ui->lineEdit_Uptime_Max->text().toLongLong(&convOk);
    if (!convOk)
    {
        addLogLine("Invalid uptime range for replay, max.");
        ui->lineEdit_Uptime_Max->setFocus();
        return;
    }

    if (firstUptimeToReplay >= lastUptimeToReplay)
    {
        addLogLine("Invalid uptime range for replay, min>max.");
        ui->lineEdit_Uptime_Min->setFocus();
        return;
    }

    if (getLastUptime() < 0)
    {
        addLogLine("No data to replay.\nData for both rovers empty or no valid data found\n(tags are synced to rovers' iTOWs).");
    }
    else
    {
        addLogLine("Replay started.");
        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(false);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(true);
        ui->lineEdit_Uptime_Min->setEnabled(false);
        ui->lineEdit_Uptime_Max->setEnabled(false);

        stopReplayRequest = false;

        // emit initial distance (/ fallback if no distances in file)
        DistanceItem initialDistanceItem;
        initialDistanceItem.distance = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        initialDistanceItem.type = DistanceItem::CONSTANT;
        emit replayData_Distance(1, initialDistanceItem);

        handleReplay(true);
    }
}

void PostProcessingForm::on_replayTimerTimeout()
{
    handleReplay(false);
}


void PostProcessingForm::handleReplay(bool firstRound)
{
    qint64 nextUptime_ms = getNextUptime(lastReplayedUptime_ms);

    if (stopReplayRequest)
    {
        addLogLine("Replay stopped. Last replayed uptime: " + QString::number(lastReplayedUptime_ms));
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(true);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->lineEdit_Uptime_Min->setEnabled(true);
        ui->lineEdit_Uptime_Max->setEnabled(true);

        stopReplayRequest = false;
    } else if ((nextUptime_ms >= 0) && (lastReplayedUptime_ms <= lastUptimeToReplay))
    {
        for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
        {
            if (rovers[i].roverSyncData.find(nextUptime_ms) != rovers[i].roverSyncData.end())
            {
                RoverSyncItem syncItem = rovers[i].roverSyncData[nextUptime_ms];

                if (rovers[i].relposnedMessages.find(syncItem.iTOW) != rovers[i].relposnedMessages.end())
                {
                    // Make local copy to add time stamp / frame duration.

                    UBXMessage_RELPOSNED relposnedMessage = rovers[i].relposnedMessages[syncItem.iTOW];

                    relposnedMessage.messageStartTime = nextUptime_ms;
                    relposnedMessage.messageEndTime = nextUptime_ms + syncItem.frameTime;

                    emit replayData_Rover(relposnedMessage, i);
                }
                else
                {
                    addLogLine("Warning: File \"" + syncItem.sourceFile + "\", line " +
                               QString::number(syncItem.sourceFileLine)+
                               ",  uptime " + QString::number(nextUptime_ms) +
                               ",  iTOW " + QString::number(syncItem.iTOW) +
                               ": No matching rover " + getRoverIdentString(i) + "-data found. Skipped.");
                }
            }
        }

        if (distances.find(nextUptime_ms) != distances.end())
        {
            emit replayData_Distance(nextUptime_ms, distances[nextUptime_ms]);
        }

        if (lidarRounds.find(nextUptime_ms) != lidarRounds.end())
        {
            emit replayData_Lidar(lidarRounds[nextUptime_ms].distanceItems, lidarRounds[nextUptime_ms].startTime, lidarRounds[nextUptime_ms].endTime);
        }

        if (tags.find(nextUptime_ms) != tags.end())
        {
            QList<Tag> tagItems = tags.values(nextUptime_ms);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                emit replayData_Tag(nextUptime_ms, tagItems[i]);
            }
        }

        lastReplayedUptime_ms = nextUptime_ms;

        // Go on to next uptime

        nextUptime_ms = getNextUptime(nextUptime_ms);

        if (nextUptime_ms >= 0)
        {
            qint64 timerTotalError_ns = 0;

            if (firstRound)
            {
                cumulativeRequestedWaitTime_ns = 0;
                replayTimeElapsedTimer.start();
            }
            else
            {
                timerTotalError_ns = static_cast<qint64>(replayTimeElapsedTimer.nsecsElapsed()) - cumulativeRequestedWaitTime_ns;
            }

            qint64 uptimeDifference_ms = nextUptime_ms - lastReplayedUptime_ms;

            if (uptimeDifference_ms > (ui->doubleSpinBox_LimitInterval->value() * 1000))
            {
                addLogLine("Warning: Time between messages limited to max value (" +
                           QString::number(ui->doubleSpinBox_LimitInterval->value(), 'g', 3) + " s) between uptimes " +
                           QString::number(lastReplayedUptime_ms) + " and " +
                           QString::number(nextUptime_ms) + ".");
                uptimeDifference_ms = ui->doubleSpinBox_LimitInterval->value() * 1000;
            }

            qint64 expectedWaitTime_ns;

            if (ui->doubleSpinBox_ReplaySpeed->value() >= 1000)
            {
                // Max speed
                expectedWaitTime_ns = 0;
            }
            else
            {
                expectedWaitTime_ns = static_cast<qint64>((1000000. * uptimeDifference_ms) / ui->doubleSpinBox_ReplaySpeed->value());
            }

            if ((timerTotalError_ns >= 1e9) && (ui->doubleSpinBox_ReplaySpeed->value() <= 1))
            {
                addLogLine("Warning: Replay timer total error exceeded 1s (computer was in sleep or otherwise laggy?), timer reset.");
                timerTotalError_ns = 0;
                cumulativeRequestedWaitTime_ns = 0;
                replayTimeElapsedTimer.restart();
            }
            else if (timerTotalError_ns >= 1e9)
            {
                // Limit maximum error to 1 s if computer can't keep up with the pace when replaying overspeed
                timerTotalError_ns = 1e9;
            }

            cumulativeRequestedWaitTime_ns += expectedWaitTime_ns;

            qint64 waitTime_ns = expectedWaitTime_ns - timerTotalError_ns;

            if (waitTime_ns < 0)
            {
                waitTime_ns = 0;
            }

            int waitTime_ms = static_cast<int>(waitTime_ns / 1000000);

            qint64 replayRange_Min = getFirstUptime();
            if (firstUptimeToReplay > replayRange_Min)
            {
                replayRange_Min = firstUptimeToReplay;
            }

            qint64 replayRange_Max = getLastUptime();
            if (lastUptimeToReplay < replayRange_Max)
            {
                replayRange_Max = lastUptimeToReplay;
            }

            int progress = (lastReplayedUptime_ms - replayRange_Min) * 100 / (replayRange_Max - replayRange_Min);

            ui->progress_ReplayProgress->setValue(progress);

            QTimer::singleShot(waitTime_ms, this, SLOT(on_replayTimerTimeout()));
        }
        else
        {
            // Replay is finished

#if 0
            // List tags with iTOW larger than any of the rover iTOWs

            QMap<UBXMessage_RELPOSNED::ITOW, Tag>::const_iterator orphanedTagIterator = tags.upperBound(lastReplayedITOW);

            while (orphanedTagIterator != tags.end())
            {
                const UBXMessage_RELPOSNED::ITOW orphanedTagITOW = orphanedTagIterator.key();
                const Tag& orphanedTag = orphanedTagIterator.value();

                addLogLine("Warning: Tag in file \"" + orphanedTag.sourceFile + "\", line " +
                           QString::number(orphanedTag.sourceFileLine)+
                           ", iTOW " + QString::number(orphanedTagITOW) +
                           ": File \"" + orphanedTag.sourceFile + "\": iTOW larger than any rover's iTOW. Tag ignored.");

                orphanedTagIterator++;
            }
#endif

            addLogLine("Replay finished.");
            ui->progress_ReplayProgress->setValue(0);
            ui->pushButton_StartReplay->setEnabled(true);
            ui->pushButton_ContinueReplay->setEnabled(false);
            ui->pushButton_StopReplay->setEnabled(false);
            ui->lineEdit_Uptime_Min->setEnabled(true);
            ui->lineEdit_Uptime_Max->setEnabled(true);

            if (ui->checkBox_Looping->isChecked())
            {
                addLogLine("Looping replay...");
                on_pushButton_StartReplay_clicked();
            }
        }
    }
    else
    {
        addLogLine("Replay finished unexpectedly. Did you clear some data during replay/pause?");
        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->lineEdit_Uptime_Min->setEnabled(true);
        ui->lineEdit_Uptime_Max->setEnabled(true);
    }

}

qint64 PostProcessingForm::getFirstUptime()
{
    qint64 firstUptime = std::numeric_limits<qint64>::max();

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if ((!rovers[i].roverSyncData.isEmpty()) && (rovers[i].roverSyncData.firstKey() < firstUptime))
        {
            firstUptime = rovers[i].roverSyncData.firstKey();
        }
    }

    if ((!distances.isEmpty()) && (distances.firstKey() < firstUptime))
    {
        firstUptime = distances.firstKey();
    }

    if ((!lidarRounds.isEmpty()) && (lidarRounds.firstKey() < firstUptime))
    {
        firstUptime = lidarRounds.firstKey();
    }

    if ((!tags.isEmpty()) && (tags.firstKey() < firstUptime))
    {
        firstUptime = tags.firstKey();
    }

    if (firstUptime == std::numeric_limits<qint64>::max())
    {
        firstUptime = -1;
    }

    return firstUptime;
}

qint64 PostProcessingForm::getLastUptime()
{
    qint64 lastUptime = -1;

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if ((!rovers[i].roverSyncData.isEmpty()) && (rovers[i].roverSyncData.lastKey() > lastUptime))
        {
            lastUptime = rovers[i].roverSyncData.lastKey();
        }
    }

    if ((!distances.isEmpty()) && (distances.lastKey() > lastUptime))
    {
        lastUptime = distances.lastKey();
    }

    if ((!lidarRounds.isEmpty()) && (lidarRounds.lastKey() > lastUptime))
    {
        lastUptime = lidarRounds.lastKey();
    }

    if ((!tags.isEmpty()) && (tags.lastKey() > lastUptime))
    {
        lastUptime = tags.lastKey();
    }

    return lastUptime;
}

qint64 PostProcessingForm::getNextUptime(const qint64 uptime)
{
    qint64 nextUptime = std::numeric_limits<qint64>::max();

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if ((!rovers[i].roverSyncData.empty()) && (rovers[i].roverSyncData.upperBound(uptime) != rovers[i].roverSyncData.end()) &&
                (rovers[i].roverSyncData.upperBound(uptime).key() < nextUptime))
        {
            nextUptime = rovers[i].roverSyncData.upperBound(uptime).key();
        }
    }

    if ((!tags.empty()) && (tags.upperBound(uptime) != tags.end()) &&
            ((tags.upperBound(uptime).key() < nextUptime) || (nextUptime == -1)))
    {
        nextUptime = tags.upperBound(uptime).key();
    }

    if ((!distances.empty()) && (distances.upperBound(uptime) != distances.end()) &&
            ((distances.upperBound(uptime).key() < nextUptime) || (nextUptime == -1)))
    {
        nextUptime = distances.upperBound(uptime).key();
    }

    if ((!lidarRounds.empty()) && (lidarRounds.upperBound(uptime) != lidarRounds.end()) &&
            ((lidarRounds.upperBound(uptime).key() < nextUptime) || (nextUptime == -1)))
    {
        nextUptime = lidarRounds.upperBound(uptime).key();
    }

    if (nextUptime == std::numeric_limits<qint64>::max())
    {
        return -1;
    }
    else
    {
        return nextUptime;
    }
}


void PostProcessingForm::on_pushButton_StopReplay_clicked()
{
    stopReplayRequest = true;
}

void PostProcessingForm::on_pushButton_ContinueReplay_clicked()
{
    if (getLastUptime() < 0)
    {
        addLogLine("No data to replay.\nData for both rovers empty or no valid data found\n(tags are synced to rovers' iTOWs).");
    }
    else
    {
        addLogLine("Replay continued.");
//        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(false);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(true);
        ui->lineEdit_Uptime_Min->setEnabled(false);
        ui->lineEdit_Uptime_Max->setEnabled(false);

        stopReplayRequest = false;
        handleReplay(true);
    }
}

void PostProcessingForm::on_pushButton_Stylus_Movie_GenerateScript_clicked()
{
    Eigen::Transform<double, 3, Eigen::Affine> transform;

    if (!generateTransformationMatrix(transform))
    {
        return;
    }

    Eigen::Matrix3d transform_NoTranslation = transform.linear();

    if (fileDialog_Stylus_MovieScript.exec())
    {
        QStringList fileNameList = fileDialog_Stylus_MovieScript.selectedFiles();

        if (fileNameList.size() != 0)
        {
            fileDialog_Stylus_MovieScript.setDirectory(QFileInfo(fileNameList[0]).path());
        }

        if (fileNameList.length() != 1)
        {
            addLogLine("Movie script: Multiple file selection not supported. Script not created.");
            return;
        }

        QFile movieScriptFile;

        movieScriptFile.setFileName(fileNameList[0]);

        if (movieScriptFile.exists())
        {
            QMessageBox msgBox;
            msgBox.setText("File already exists.");
            msgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

            msgBox.setDefaultButton(cancelButton);

            msgBox.exec();

            if (msgBox.clickedButton() != overwriteButton)
            {
                addLogLine("Movie script not created.");
                return;
            }
        }

        if (!movieScriptFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Can't open movie script file.");
            return;
        }

        QTextStream textStream(&movieScriptFile);

        textStream << "// Lines\tiTOW\tX\tY\tZ\taccX\tAccY\tAccZ\tObject\n";

        addLogLine("Processing line sets...");

        // Some locals to prevent excessive typing:
        QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Min = ui->spinBox_Stylus_Movie_ITOW_Points_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Max = ui->spinBox_Stylus_Movie_ITOW_Points_Max->value();
        unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

//        QMap<qint64, Tag_New>::const_iterator currentTagIterator;

        bool objectActive = false;

        qint64 beginningUptime = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QString objectName = "N/A";

        qint64 uptime = -1;
        Tag beginningTag;

        while (tags.upperBound(uptime) != tags.end())
        {
            uptime = tags.upperBound(uptime).key();

            QList<Tag> tagItems = tags.values(uptime);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const Tag& currentTag = tagItems[i];

                if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
                {
                    objectActive = false;

                    if (currentTag.text.length() == 0)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": New object without a name. Ending previous object, but not beginning new nor creating a new line. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;
                        objectName = "N/A";

                        continue;
                    }

                    addLogLine("Object \"" + currentTag.text + "\"...");

                    objectActive = true;
                    objectName = currentTag.text;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                    pointsWritten = 0;
                }
                else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Duplicate beginning tag. Skipped.");
                        continue;
                    }

                    beginningUptime = uptime;
                    beginningTag = currentTag;
                }
                else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag without beginning tag. Skipped.");
                        continue;
                    }

                    const Tag& endingTag = currentTag;
    //                const Tag_New beginningTag = tags_New[beginningUptime];

                    if (endingTag.sourceFile != beginningTag.sourceFile)
                    {
                        addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                                   beginningTag.sourceFile + "\", line " +
                                   QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                                   endingTag.sourceFile + "\", line " +
                                   QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                        continue;
                    }

                    bool constDistancesOnly = true;

                    QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(beginningUptime);

                    while ((distIter != distances.end()) && (distIter.key() < uptime))
                    {
                        if (distIter.value().type == DistanceItem::Type::MEASURED)
                        {
                            constDistancesOnly = false;
                            break;
                        }

                        distIter++;
                    }

                    int pointsBetweenTags = 0;

                    if (constDistancesOnly)
                    {
                        distIter = distances.upperBound(beginningUptime);

                        if (distIter != distances.end())
                        {
                            distIter--;

                            if ((distIter == distances.end()) ||
                                    (distIter.value().type != DistanceItem::Type::CONSTANT))
                            {
                                addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                           QString::number(currentTag.sourceFileLine)+
                                           ", uptime " + QString::number(uptime) +
                                           ", iTOW " + QString::number(currentTag.iTOW) +
                                           ": Points between tags having only constant distances without preceeding constant distance. Skipped.");
                                continue;
                            }

                            stylusTipDistanceFromRoverA = distIter.value().distance;
                        }

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.upperBound(rovers[0].relposnedMessages.upperBound(beginningUptime).value().iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.upperBound(rovers[1].relposnedMessages.upperBound(beginningUptime).value().iTOW);

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = rovers[0].relposnedMessages.upperBound(currentTag.iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = rovers[1].relposnedMessages.upperBound(currentTag.iTOW);

                        while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                            (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                        {
                            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                                   (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
                            {
                                relposIterator_RoverA++;
                            }

                            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                                   (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
                            {
                                relposIterator_RoverB++;
                            }

                            if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                                (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                            {
                                if ((relposIterator_RoverA.key() >= iTOWRange_Lines_Min) &&
                                (relposIterator_RoverA.key() <= iTOWRange_Lines_Max))
                                {
                                    Eigen::Vector3d roverAPosNED(
                                            relposIterator_RoverA.value().relPosN,
                                            relposIterator_RoverA.value().relPosE,
                                            relposIterator_RoverA.value().relPosD);

                                    Eigen::Vector3d roverBPosNED(
                                            relposIterator_RoverB.value().relPosN,
                                            relposIterator_RoverB.value().relPosE,
                                            relposIterator_RoverB.value().relPosD);

                                    Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                                    Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                                    Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

                                    Eigen::Vector3d stylusTipAccNED(
                                                relposIterator_RoverA.value().accN,
                                                relposIterator_RoverA.value().accE,
                                                relposIterator_RoverA.value().accD);

                                    // Use accuracies of rover A (used for stylus tip accuracy)
                                    // Could calculate some kind of "worst case" scenario using both rovers,
                                    // but probably errors are mostly common to both of them.
                                    Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

                                    QString lineOut;

                                    if (pointsBetweenTags == 0)
                                    {
                                        lineOut = "LStart";

                                    }
                                    else
                                    {
                                        lineOut = "LCont";
                                    }

                                    lineOut +=
                                            "\t" + QString::number(relposIterator_RoverA.key()) +
                                            "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                            "\t" + objectName;

                                    textStream << (lineOut + "\n");

                                    pointsWritten++;

                                    pointsBetweenTags++;
                                }
                                relposIterator_RoverA++;
                                relposIterator_RoverB++;
                            }
                        }
                    } // if (constDistancesOnly)
                    else
                    {
                        // Distances found, sync point creation to them.
                        // Rover coordinates are interpolated according to distance timestamps.

                        distIter = distances.upperBound(beginningUptime);

                        while ((distIter != distances.end()) && (distIter.key() < uptime))
                        {
                            if (distIter.value().type == DistanceItem::Type::CONSTANT)
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Constant distance between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }
                            else if (distIter.value().type == DistanceItem::Type::MEASURED)
                            {
                                // Try to find next and previous rover coordinates
                                // for this uptime

                                qint64 distanceUptime = distIter.key();
                                // TODO: Add/subtract fine tune sync value here if needed

                                QMap<qint64, RoverSyncItem>::const_iterator roverAUptimeIter = rovers[0].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverA;

                                if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverAUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverAUptimeIter--;
                                    if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverAUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(upperSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[0].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[0].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                QMap<qint64, RoverSyncItem>::const_iterator roverBUptimeIter = rovers[1].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverB;

                                if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverBUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverBUptimeIter--;
                                    if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverBUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(upperSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[1].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[1].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                stylusTipDistanceFromRoverA = distIter.value().distance;

                                // TODO: Refine this quick hack or at least make the distance configurable!
                                // Skip distances that are too far away
                                // (measurement module seems to emit "outliers" now and then)
                                if (stylusTipDistanceFromRoverA > 2)
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Distance between RoverA and tip too high (" +
                                               QString::number(stylusTipDistanceFromRoverA) +
                                               " m). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                Eigen::Vector3d roverAPosNED(
                                        interpolated_RoverA.relPosN,
                                        interpolated_RoverA.relPosE,
                                        interpolated_RoverA.relPosD);

                                Eigen::Vector3d roverBPosNED(
                                        interpolated_RoverB.relPosN,
                                        interpolated_RoverB.relPosE,
                                        interpolated_RoverB.relPosD);

                                Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

                                Eigen::Vector3d stylusTipAccNED(
                                            interpolated_RoverA.accN,
                                            interpolated_RoverA.accE,
                                            interpolated_RoverA.accD);

                                // Use accuracies of rover A (used for stylus tip accuracy)
                                // Could calculate some kind of "worst case" scenario using both rovers,
                                // but probably errors are mostly common to both of them.
                                Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

                                QString lineOut;

                                if (pointsBetweenTags == 0)
                                {
                                    lineOut = "LStart";
                                }
                                else
                                {
                                    lineOut = "LCont";
                                }

                                lineOut +=
                                        "\t" + QString::number(interpolated_RoverA.iTOW) +
                                        "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                        "\t" + objectName;

                                textStream << (lineOut + "\n");

                                pointsWritten++;
                                pointsBetweenTags++;
                            }
                            else
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Unknown distance type between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }

                            distIter++;
                        }
                    }


                    if (pointsBetweenTags == 0)
                    {
                        addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                                   QString::number(beginningTag.sourceFileLine) +
                                   ", iTOW " + QString::number(rovers[0].roverSyncData.upperBound(beginningUptime).value().iTOW) + ", ending tag line " +
                                   QString::number(endingTag.sourceFileLine) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ", File \"" + endingTag.sourceFile + "\""
                                   " No points between tags.");
                    }

                    beginningTag.iTOW = -1;
                    beginningUptime = -1;
                }
            }
        }

        if (beginningTag.iTOW != -1)
        {
            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
                       QString::number(beginningTag.sourceFileLine) +
                       ", iTOW " + QString::number(beginningTag.iTOW) +
                       " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
        }

        stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();

        addLogLine("Processing script...");
        textStream << "// Frame type\tiTOW"
                      "\tTip_X\tTip_Y\tTip_Z"
                      "\tRoverA_X\tRoverA_Y\tRoverA_Z"
                      "\tRoverB_X\tRoverB_Y\tRoverB_Z"
                      "\tTip_acc_X\tTip_Acc_Y\tTip_Acc_Z"
                      "\tRoverA_acc_X\tRoverA_Acc_Y\tRoverA_Acc_Z"
                      "\tRoverB_acc_X\tRoverB_Acc_Y\tRoverB_Acc_Z"
                      "\tCamera_X\tCamera_Y\tCamera_Z"
                      "\tLookAt_X\tLookAt_Y\tLookAt_X\tTipPositionValidity\n";

        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = ui->spinBox_Stylus_Movie_ITOW_Script_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = ui->spinBox_Stylus_Movie_ITOW_Script_Max->value();

        double fps = ui->doubleSpinBox_Stylus_Movie_FPS->value();

        iTOWRange_Script_Min -= iTOWRange_Script_Min % expectedITOWAlignment; // Round to previous aligned ITOW

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.lowerBound(iTOWRange_Script_Min);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.lowerBound(iTOWRange_Script_Min);

        UBXMessage_RELPOSNED::ITOW startingITOW = (relposIterator_RoverA.value().iTOW > relposIterator_RoverB.value().iTOW) ?
                    relposIterator_RoverA.value().iTOW : relposIterator_RoverB.value().iTOW;

        startingITOW -= startingITOW % expectedITOWAlignment; // Should not be needed, but just to be sure...

        int frameCounter = 0;

        // Some variables for camera:
        double cameraNShift = ui->doubleSpinBox_Stylus_Movie_Camera_N->value();
        double cameraEShift = ui->doubleSpinBox_Stylus_Movie_Camera_E->value();
        double cameraDShift = ui->doubleSpinBox_Stylus_Movie_Camera_D->value();

        double lookAtNShift = ui->doubleSpinBox_Stylus_Movie_LookAt_N->value();
        double lookAtEShift = ui->doubleSpinBox_Stylus_Movie_LookAt_E->value();
        double lookAtDShift = ui->doubleSpinBox_Stylus_Movie_LookAt_D->value();

        UBXMessage_RELPOSNED::ITOW iTOW = (frameCounter * 1000) / fps + startingITOW;

        UBXMessage_RELPOSNED::ITOW lastRoverANagITOW = -1;
        UBXMessage_RELPOSNED::ITOW lastRoverBNagITOW = -1;

        while ((iTOW <= iTOWRange_Script_Max) &&
               (rovers[0].relposnedMessages.upperBound(iTOW) != rovers[0].relposnedMessages.end()) &&
               (rovers[1].relposnedMessages.upperBound(iTOW) != rovers[1].relposnedMessages.end()))
        {
            QString frameType;

            UBXMessage_RELPOSNED relposned_RoverA;
            UBXMessage_RELPOSNED relposned_RoverB;

            qint64 uptime = -1;

            if ((rovers[0].relposnedMessages.find(iTOW) != rovers[0].relposnedMessages.end()) &&
                    (rovers[1].relposnedMessages.find(iTOW) != rovers[1].relposnedMessages.end()))
            {
                frameType = "F_Key";
                relposned_RoverA = rovers[0].relposnedMessages.find(iTOW).value();
                relposned_RoverB = rovers[1].relposnedMessages.find(iTOW).value();

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA = rovers[0].reverseSync.find(iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB = rovers[1].reverseSync.find(iTOW);

                if ((reverseIter_RoverA == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = iTOW;
                }

                if ((reverseIter_RoverB == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = iTOW;
                }

                if ((reverseIter_RoverA != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverB != rovers[1].reverseSync.end()))
                {
                    uptime = (reverseIter_RoverA.value() + reverseIter_RoverB.value()) / 2;
                }
            }
            else
            {
                frameType = "F_Interp";

                UBXMessage_RELPOSNED interpAStart = (rovers[0].relposnedMessages.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpAEnd = rovers[0].relposnedMessages.upperBound(iTOW).value();

                UBXMessage_RELPOSNED interpBStart = (rovers[1].relposnedMessages.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpBEnd = rovers[1].relposnedMessages.upperBound(iTOW).value();

                relposned_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(interpAStart, interpAEnd, iTOW);
                relposned_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(interpBStart, interpBEnd, iTOW);

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_Start = rovers[0].reverseSync.find(interpAStart.iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_End = rovers[0].reverseSync.find(interpAEnd.iTOW);

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_Start = rovers[1].reverseSync.find(interpBStart.iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_End = rovers[1].reverseSync.find(interpBEnd.iTOW);

                if ((reverseIter_RoverA_Start == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != interpAStart.iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(interpAStart.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = interpAStart.iTOW;
                }

                if ((reverseIter_RoverA_End == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != interpAEnd.iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(interpAEnd.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = interpAEnd.iTOW;
                }

                if ((reverseIter_RoverB_Start == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != interpBStart.iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(interpBStart.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = interpBStart.iTOW;
                }

                if ((reverseIter_RoverB_End == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != interpBEnd.iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(interpBEnd.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = interpBEnd.iTOW;
                }

                if ((reverseIter_RoverA_Start != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverA_End != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverB_Start != rovers[1].reverseSync.end()) &&
                        (reverseIter_RoverB_End != rovers[1].reverseSync.end()))
                {
                    double startITOWAvg = (interpAStart.iTOW + interpBStart.iTOW) / 2;
                    double endITOWAvg = (interpAEnd.iTOW + interpBEnd.iTOW) / 2;

                    double fraction = static_cast<double>((iTOW - startITOWAvg)) /
                            (endITOWAvg - startITOWAvg);

                    uptime = (reverseIter_RoverA_Start.value() + reverseIter_RoverB_Start.value()) / 2 +
                            fraction *
                            (((reverseIter_RoverA_End.value() - reverseIter_RoverA_Start.value()) +
                             (reverseIter_RoverB_End.value() - reverseIter_RoverB_Start.value())) / 2);
                }
            }

            bool distanceValid = false;

            QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(uptime);

            if ((distIter != distances.cend()) && (uptime != -1))
            {
                QMap<qint64, DistanceItem>::const_iterator nextDistIter = distIter;

                if (distIter != distances.cbegin())
                {
                    distIter--;

                    QMap<qint64, DistanceItem>::const_iterator prevDistIter = distIter;

                    if (prevDistIter.value().type == DistanceItem::Type::CONSTANT)
                    {
                        distanceValid = true;
                        stylusTipDistanceFromRoverA = prevDistIter.value().distance;
                    }
                    else if ((prevDistIter.value().type == DistanceItem::Type::MEASURED) &&
                             (nextDistIter.value().type == DistanceItem::Type::MEASURED))
                    {
                        // Interpolate between measured distances

                        double fraction = static_cast<double>((uptime - prevDistIter.key())) /
                                (nextDistIter.key() - prevDistIter.key());

                        stylusTipDistanceFromRoverA = prevDistIter.value().distance +
                                fraction * (nextDistIter.value().distance - prevDistIter.value().distance);

                        if ((uptime - prevDistIter.key()) < 500)
                        {
                            distanceValid = true;
                        }
                    }
                    else if (prevDistIter.value().type == DistanceItem::Type::MEASURED)
                    {
                        // No more measured distances -> Use the last one

                        stylusTipDistanceFromRoverA = prevDistIter.value().distance;

                        if ((uptime - prevDistIter.key()) < 500)
                        {
                            distanceValid = true;
                        }
                    }
                    else
                    {
                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                   QString::number(distIter.value().sourceFileLine)+
                                   ", uptime " + QString::number(distIter.key()) +
                                   ": Unknown distance type between measured ones. Skipped.");
                    }
                }
            }

            Eigen::Vector3d roverAPosNED(
                    relposned_RoverA.relPosN,
                    relposned_RoverA.relPosE,
                    relposned_RoverA.relPosD);

            Eigen::Vector3d roverBPosNED(
                    relposned_RoverB.relPosN,
                    relposned_RoverB.relPosE,
                    relposned_RoverB.relPosD);

            Eigen::Vector3d roverBToAVecNormalizedNED = (roverAPosNED - roverBPosNED).normalized();

            Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalizedNED * stylusTipDistanceFromRoverA;

            Eigen::Vector3d roverAAccNED(
                        relposned_RoverA.accN,
                        relposned_RoverA.accE,
                        relposned_RoverA.accD);

            Eigen::Vector3d roverBAccNED(
                        relposned_RoverB.accN,
                        relposned_RoverB.accE,
                        relposned_RoverB.accD);

            Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
            Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
            Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

            Eigen::Vector3d roverAAccXYZ = transform_NoTranslation * roverAAccNED;
            Eigen::Vector3d roverBAccXYZ = transform_NoTranslation * roverBAccNED;

            // Use accuracies of rover A (used for stylus tip accuracy)
            // Could calculate some kind of "worst case" scenario using both rovers,
            // but probably errors are mostly common to both of them.
            Eigen::Vector3d stylusTipAccXYZ = roverAAccXYZ;

            Eigen::Vector3d downVecNED(0,0,1);
            Eigen::Vector3d stylusForwardAxis = roverBToAVecNormalizedNED;
            Eigen::Vector3d stylusRightAxis = -(roverBToAVecNormalizedNED.cross(downVecNED).normalized());
            Eigen::Vector3d stylusDownAxis = roverBToAVecNormalizedNED.cross(stylusRightAxis).normalized();

            Eigen::Vector3d cameraPosNED = roverAPosNED +
                    stylusForwardAxis * cameraNShift +
                    stylusRightAxis * cameraEShift +
                    stylusDownAxis * cameraDShift;

            Eigen::Vector3d cameraPosXYZ = transform * cameraPosNED;

            Eigen::Vector3d lookAtPosNED = roverAPosNED +
                    stylusForwardAxis * lookAtNShift +
                    stylusRightAxis * lookAtEShift +
                    stylusDownAxis * lookAtDShift;

            Eigen::Vector3d lookAtPosXYZ = transform * lookAtPosNED;

            QString *stylusTipLocationValidityString = new QString;

            if (distanceValid)
            {
                *stylusTipLocationValidityString = "Valid";
            }
            else
            {
                *stylusTipLocationValidityString = "Invalid";
            }

            QString lineOut =
                    frameType +
                    "\t" + QString::number(iTOW) +
                    "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(2), 'f', 4) +
                    "\t" + *stylusTipLocationValidityString;

            textStream << (lineOut + "\n");

            frameCounter++;
            iTOW = (frameCounter * 1000) / fps + startingITOW;
        }
    }
    addLogLine("Movie script generated.");
}

void PostProcessingForm::on_pushButton_ClearDistanceData_clicked()
{
    distances.clear();
    addLogLine("Distance data cleared.");
}

void PostProcessingForm::addDistanceData(const QStringList& fileNames)
{
    double distanceCorrection = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value();

    addLogLine("Reading distances...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile distanceFile;
        distanceFile.setFileName(fileName);
        if (distanceFile.open(QIODevice::ReadOnly))
        {
            int numberOfDistances = 0;

            QTextStream textStream(&distanceFile);

            int64_t fileLength = distanceFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                distanceFile.close();
                continue;
            }

            QString headerLine = textStream.readLine();

            if (headerLine.compare("Time\tDistance\tType\tUptime(Start)\tFrame time", Qt::CaseInsensitive))
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Skipped.");
                distanceFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;

            int firstDuplicateUptimeLine = 0;
            int lastDuplicateUptimeLine = 0;

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if (subItems.count() < 5)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                bool distanceConvOk;

                DistanceItem newDistanceItem;
                newDistanceItem.distance = subItems[1].toDouble(&distanceConvOk);

                if (!distanceConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 2 (distance) to double. Line skipped.");
                    continue;
                }

                if (!subItems[2].compare("constant", Qt::CaseInsensitive))
                {
                    newDistanceItem.type = DistanceItem::Type::CONSTANT;
                }
                else if (!subItems[2].compare("measured", Qt::CaseInsensitive))
                {
                    newDistanceItem.type = DistanceItem::Type::MEASURED;
                    newDistanceItem.distance += distanceCorrection;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Distance type not either \"constant\" nor \"measured\". Line skipped");
                    continue;
                }

                bool uptimeConvOk;

                qint64 uptime = subItems[3].toLongLong(&uptimeConvOk);

                if (!uptimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 4 (Uptime(Start)) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool frametimeConvOk;

                newDistanceItem.frameDuration = subItems[4].toLongLong(&frametimeConvOk);

                if (!frametimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Frame time) to 64-bit integer. Frame time set to 0.");
                    newDistanceItem.frameDuration = 0;
                }

                if (distances.find(uptime) != distances.end())
                {
                    if (!firstDuplicateUptimeLine)
                    {
                        firstDuplicateUptimeLine = lineNumber;
                    }

                    lastDuplicateUptimeLine = lineNumber;

                    discardedLines++;
                    continue;
                }

                if (firstDuplicateUptimeLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateUptimeLine) + "-"  +
                               QString::number(lastDuplicateUptimeLine) +
                               ": Distance(s) with duplicate uptime(s). Line(s) skipped.");

                    firstDuplicateUptimeLine = 0;
                }

                newDistanceItem.sourceFile = fileName;
                newDistanceItem.sourceFileLine = lineNumber;

                distances[uptime] = newDistanceItem;

                numberOfDistances ++;
            }

            if (firstDuplicateUptimeLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateUptimeLine) + "-"  +
                           QString::number(lastDuplicateUptimeLine) +
                           ": Distance(s) with duplicate uptime(s). Line(s) skipped.");
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid distances: " +
                       QString::number(numberOfDistances) +
                       ", total lines: " + QString::number(lineNumber) +
                       ", discarded lines: " + QString::number(discardedLines) + ".");
        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}


void PostProcessingForm::on_pushButton_AddDistanceData_clicked()
{
    if (fileDialog_Distances.exec())
    {
        QStringList fileNames = fileDialog_Distances.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Distances.setDirectory(QFileInfo(fileNames[0]).path());
            syncLogFileDialogDirectories(fileDialog_Distances.directory().path(), true);
        }

        addDistanceData(fileNames);
    }
}

void PostProcessingForm::on_pushButton_ClearSyncData_clicked()
{
    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        rovers[i].roverSyncData.clear();
        rovers[i].reverseSync.clear();
    }

    addLogLine("Sync data cleared.");
}

void PostProcessingForm::addSyncData(const QStringList& fileNames)
{
    addLogLine("Reading sync data...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile syncFile;
        syncFile.setFileName(fileName);
        if (syncFile.open(QIODevice::ReadOnly))
        {
            int numberOfSyncItems = 0;

            QTextStream textStream(&syncFile);

            int64_t fileLength = syncFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                syncFile.close();
                continue;
            }

            QString headerLine = textStream.readLine();

            if (headerLine.compare("Time\tSource\tType\tiTOW\tUptime(Start)\tFrame time", Qt::CaseInsensitive))
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Skipped.");
                syncFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;

            int firstDuplicateSyncItemLine = 0;
            int lastDuplicateSyncItemLine = 0;

            unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if (subItems.count() < 6)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                RoverSyncItem newSyncItem;

                QMap<qint64, RoverSyncItem>* roverContainer = nullptr;
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>* reverseContainer = nullptr;

                if (!subItems[1].compare("rover a", Qt::CaseInsensitive))
                {
                    roverContainer = &rovers[0].roverSyncData;
                    reverseContainer = &rovers[0].reverseSync;
                }
                else if (!subItems[1].compare("rover b", Qt::CaseInsensitive))
                {
                    roverContainer = &rovers[1].roverSyncData;
                    reverseContainer = &rovers[1].reverseSync;
                }
                else if (!subItems[1].compare("rover c", Qt::CaseInsensitive))
                {
                    roverContainer = &rovers[2].roverSyncData;
                    reverseContainer = &rovers[2].reverseSync;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Source not either \"rover a\", \"rover b\" nor \"rover c\". Line skipped");
                    continue;
                }

                if (!subItems[2].compare("RELPOSNED", Qt::CaseInsensitive))
                {
                    newSyncItem.messageType = RoverSyncItem::MSGTYPE_UBX_RELPOSNED;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Type not \"RELPOSNED\" (currently only supported type). Line skipped");
                    continue;
                }

                bool iTOWConvOk;

                newSyncItem.iTOW = subItems[3].toInt(&iTOWConvOk);

                if (!iTOWConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 4 (iTOW) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool uptimeConvOk;

                qint64 uptime = subItems[4].toLongLong(&uptimeConvOk);

                if (!uptimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Uptime(Start)) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool frametimeConvOk;

                newSyncItem.frameTime = subItems[5].toLongLong(&frametimeConvOk);

                if (!frametimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Frame time) to 64-bit integer. Frame time set to 0.");
                    newSyncItem.frameTime = 0;
                }

                newSyncItem.sourceFile = fileName;
                newSyncItem.sourceFileLine = lineNumber;

                if (roverContainer->find(uptime) != roverContainer->end())
                {
                    discardedLines++;

                    if (!firstDuplicateSyncItemLine)
                    {
                        firstDuplicateSyncItemLine = lineNumber;
                    }

                    lastDuplicateSyncItemLine = lineNumber;
                    continue;
                }

                if (firstDuplicateSyncItemLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateSyncItemLine) + "-" +
                               QString::number(lastDuplicateSyncItemLine) +
                               ": Duplicate rover sync item(s). Line(s) skipped.");

                    firstDuplicateSyncItemLine = 0;
                }

                if ((newSyncItem.iTOW % expectedITOWAlignment) != 0)
                {
                    unsigned int iTOWautoAlignThreshold = ui->spinBox_ITOWAutoAlignThreshold->value();
                    int autoAlignedITOW = 0;
                    bool iTOWAutoAligned = false;

                    if ((newSyncItem.iTOW % expectedITOWAlignment) <= iTOWautoAlignThreshold)
                    {
                        // Negative correction (f. exp 1001 > 1000)
                        autoAlignedITOW = newSyncItem.iTOW - (newSyncItem.iTOW % expectedITOWAlignment);
                        iTOWAutoAligned = true;
                    }
                    else if ((expectedITOWAlignment - (newSyncItem.iTOW % expectedITOWAlignment)) <= iTOWautoAlignThreshold)
                    {
                        // Positive correction (f. exp 999 > 1000)
                        autoAlignedITOW = newSyncItem.iTOW - (newSyncItem.iTOW % expectedITOWAlignment) + expectedITOWAlignment;
                        iTOWAutoAligned = true;
                    }

                    if (iTOWAutoAligned)
                    {
                        if (ui->checkBox_ReportITOWAutoAlign->isChecked())
                        {
                            addLogLine("Warning: Line " + QString::number(lineNumber) + ": Rover iTOW auto-aligned to expected interval (" +
                                       QString::number(expectedITOWAlignment) +" ms). original iTOW: " + QString::number(newSyncItem.iTOW) +
                                       ", auto-aligned: " + QString::number(autoAlignedITOW) +
                                       " (adjustment: " + QString::number(int(autoAlignedITOW) - int(newSyncItem.iTOW)) + ")");
                        }

                        newSyncItem.iTOW = autoAlignedITOW;
                    }
                    else
                    {
                        if (ui->checkBox_ReportUnalignedITOWS->isChecked())
                        {
                            addLogLine("Warning: Line " + QString::number(lineNumber) + ": Rover iTOW not aligned or auto-alignable to expected interval (" +
                                       QString::number(expectedITOWAlignment) +" ms). iTOW: " + QString::number(newSyncItem.iTOW));
                        }
                    }
                }

                roverContainer->insert(uptime, newSyncItem);
                reverseContainer->insert(newSyncItem.iTOW, uptime);

                numberOfSyncItems ++;
            }

            if (firstDuplicateSyncItemLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateSyncItemLine) + "-" +
                           QString::number(lastDuplicateSyncItemLine) +
                           ": Duplicate rover sync item(s). Line(s) skipped.");
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid sync items: " +
                       QString::number(numberOfSyncItems) +
                       ", total lines: " + QString::number(lineNumber) +
                       ", discarded lines: " + QString::number(discardedLines) + ".");
        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}


void PostProcessingForm::on_pushButton_AddSyncData_clicked()
{
    if (fileDialog_Sync.exec())
    {
        QStringList fileNames = fileDialog_Sync.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Sync.setDirectory(QFileInfo(fileNames[0]).path());
            syncLogFileDialogDirectories(fileDialog_Sync.directory().path(), true);
        }

        addSyncData(fileNames);
    }
}

void PostProcessingForm::on_pushButton_GenerateSyncDataBasedOnITOWS_clicked()
{
    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        rovers[i].roverSyncData.clear();
        rovers[i].reverseSync.clear();
    }

    addLogLine("Previous sync data cleared.");

    int itemCount = 0;

    unsigned int numofRovers = sizeof(rovers) / sizeof(rovers[0]);

    for (unsigned int roverId = 0; roverId < numofRovers; roverId++)
    {
        int lineNumber = roverId + roverId < numofRovers; // Header at first line

        for (const auto& relposnedData : rovers[roverId].relposnedMessages)
        {
            RoverSyncItem fakeSyncItem;

            fakeSyncItem.sourceFile = "None";
            fakeSyncItem.sourceFileLine = lineNumber;
            fakeSyncItem.messageType = RoverSyncItem::MSGTYPE_UBX_RELPOSNED;
            fakeSyncItem.iTOW = relposnedData.iTOW;
            fakeSyncItem.frameTime = 0;
            rovers[roverId].roverSyncData.insert(fakeSyncItem.iTOW, fakeSyncItem);
            rovers[roverId].reverseSync.insert(fakeSyncItem.iTOW, fakeSyncItem.iTOW);

            lineNumber += roverId < numofRovers;    // "Interleaved" data
            itemCount++;
        }
    }

    addLogLine(QString::number(itemCount) + " sync items created.");
}

bool PostProcessingForm::generateTransformationMatrix(Eigen::Transform<double, 3, Eigen::Affine>& outputMatrix)
{
    Eigen::Transform<double, 3, Eigen::Affine> translation_NED;

    translation_NED = translation_NED.Identity();

    translation_NED(0, 3) = ui->doubleSpinBox_Translation_N->value();
    translation_NED(1, 3) = ui->doubleSpinBox_Translation_E->value();
    translation_NED(2, 3) = ui->doubleSpinBox_Translation_D->value();

    Eigen::Transform<double, 3, Eigen::Affine> prelimTransform;

    for (int i = 0; i < 4; i++)
    {
        for (int k= 0; k < 4; k++)
        {
            bool ok;
            prelimTransform(i, k) = ui->tableWidget_TransformationMatrix->item(i, k)->text().toDouble(&ok);

            if (!ok)
            {
                addLogLine("Error: Row " + QString::number(i + 1) +
                           ", column " + QString::number(k + 1) +
                           " of transformation matrix not convertible to a (double precision) floating point value. "
                           "Unable to perform NED->XYZ-coordinate conversion.");

                return false;
            }
        }
    }

    outputMatrix = prelimTransform * translation_NED;

    return true;
}

void PostProcessingForm::on_pushButton_ClearAllFileData_clicked()
{
    this->on_pushButton_ClearRELPOSNEDData_RoverA_clicked();
    this->on_pushButton_ClearRELPOSNEDData_RoverB_clicked();
    this->on_pushButton_ClearRELPOSNEDData_RoverC_clicked();
    this->on_pushButton_ClearTagData_clicked();
    this->on_pushButton_ClearDistanceData_clicked();
    this->on_pushButton_ClearSyncData_clicked();
    this->on_pushButton_ClearLidarData_clicked();
}

void PostProcessingForm::addAllData(const bool includeTransformation)
{
    if (fileDialog_All.exec())
    {
        QStringList selectedFileNames = fileDialog_All.selectedFiles();

        if (selectedFileNames.size() != 0)
        {
            fileDialog_All.setDirectory(QFileInfo(selectedFileNames[0]).path());
            syncLogFileDialogDirectories(fileDialog_All.directory().path(), true);
        }

        QStringList baseFileNames;

        for (const auto& fileName : selectedFileNames)
        {
            struct
            {
                QString endsWith;
                QString formatName;
            } const fileNameEndings [] =
            {
                { "_base.NMEA", "NMEA (base)" },
                { "_base.raw", "raw (base)" },
                { "_base.RTCM", "RTCM (base)" },
                { "_base.ubx", "ubx (base)" },

                { "_RoverA.NMEA", "NMEA (Rover A)" },
                { "_RoverA.raw", "raw (Rover A)" },
                { "_RoverA.ubx", "ubx (Rover A)" },
                { "_RoverA_RELPOSNED.ubx", "ubx (relposned, Rover A)" },

                { "_RoverB.NMEA", "NMEA (Rover B)" },
                { "_RoverB.raw", "raw (Rover B)" },
                { "_RoverB.ubx", "ubx (Rover B)" },
                { "_RoverB_RELPOSNED.ubx", "ubx (relposned, Rover B)" },

                { "_RoverC.NMEA", "NMEA (Rover C)" },
                { "_RoverC.raw", "raw (Rover C)" },
                { "_RoverC.ubx", "ubx (Rover C)" },
                { "_RoverC_RELPOSNED.ubx", "ubx (relposned, Rover C)" },

                { "_tags.tags", "tags (""double extension""" },
                { ".tags", "tags (""single extension""" },

                { ".distances", "distances" },
                { ".lidar", "lidar" },

                { ".sync", "sync" },
                { ".transformation", "transformation" },
            };

            bool formatKnown = false;

            for (unsigned int endingIndex = 0; endingIndex < (sizeof(fileNameEndings) / sizeof(fileNameEndings[0])); endingIndex++)
            {
                if (fileName.endsWith(fileNameEndings[endingIndex].endsWith, Qt::CaseInsensitive))
                {
                    QString baseFileName = fileName;
                    baseFileName.resize(baseFileName.length() - fileNameEndings[endingIndex].endsWith.length());

                    addLogLine(QString("File selection """) + fileName + """ recognized as a format """ +
                               fileNameEndings[endingIndex].formatName + """. Base filename: """ +
                               baseFileName + """");

                    baseFileNames.append(baseFileName);

                    formatKnown = true;
                    break;
                }
            }

            if (!formatKnown)
            {
                addLogLine(QString("Warning: file selection """) + fileName +
                           """ not recognized as any supported format. Skipping... ");
            }
        }

        QStringList fileNames = getAppendedFileNames(baseFileNames, "_RoverA_RELPOSNED.ubx");
        addRELPOSNEDData_Rover(fileNames, 0);

        fileNames = getAppendedFileNames(baseFileNames, "_RoverB_RELPOSNED.ubx");
        addRELPOSNEDData_Rover(fileNames, 1);

        fileNames = getAppendedFileNames(baseFileNames, "_RoverC_RELPOSNED.ubx");
        addRELPOSNEDData_Rover(fileNames, 2);

        fileNames = getAppendedFileNames(baseFileNames, "_tags.tags");
        addTagData(fileNames);

        fileNames = getAppendedFileNames(baseFileNames, ".distances");
        addDistanceData(fileNames);

        fileNames = getAppendedFileNames(baseFileNames, ".lidar");
        addLidarData(fileNames);

        fileNames = getAppendedFileNames(baseFileNames, ".sync");
        addSyncData(fileNames);

        if (includeTransformation)
        {
            fileNames = getAppendedFileNames(baseFileNames, ".Transformation");

            if (fileNames.count() == 0)
            {
                addLogLine("Warning: No file(s) selected. Transformation not read.");
            }
            else
            {
                if (fileNames.count() != 1)
                {
                    addLogLine("Warning: Multiple files selected. Transformation read only using the first one ("""+
                               fileNames[0] + """)");
                }

                loadTransformation(fileNames[0]);
            }
        }
    }
}

void PostProcessingForm::on_pushButton_AddAll_clicked()
{
    addAllData(false);
}

void PostProcessingForm::syncLogFileDialogDirectories(const QString dir, const bool savesetting)
{
    fileDialog_UBX.setDirectory(QDir(dir));
    fileDialog_Tags.setDirectory(QDir(dir));
    fileDialog_Distances.setDirectory(QDir(dir));
    fileDialog_Sync.setDirectory(QDir(dir));
    fileDialog_Lidar.setDirectory(QDir(dir));
    fileDialog_All.setDirectory(QDir(dir));

    if (savesetting)
    {
        QSettings settings;
        settings.setValue("PostProcessing_Directory_Dialog_Logs", dir);
    }
}

QStringList PostProcessingForm::getAppendedFileNames(const QStringList& fileNames, const QString appendix)
{
    QStringList appendedList;

    for (auto fileName : fileNames)
    {
        appendedList.append(fileName + appendix);
    }

    return appendedList;
}

static QString getTransformationFileHeaderLine(void)
{
    QString line = "Translation_N\tTranslation_E\tTranslation_D";

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            line += "\tTransf R" + QString::number(row + 1) + ", C" + QString::number(column + 1);
        }
    }

    return line;
}

void PostProcessingForm::loadTransformation(const QString fileName)
{
    QFileInfo fileInfo(fileName);
    addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

    QFile transformationFile;
    transformationFile.setFileName(fileName);
    if (transformationFile.open(QIODevice::ReadOnly))
    {
        QTextStream textStream(&transformationFile);

        QString headerLine = textStream.readLine();

        if (headerLine.compare(getTransformationFileHeaderLine(), Qt::CaseInsensitive))
        {
            addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Data not read.");
            transformationFile.close();
            return;
        }

        QString dataLine = textStream.readLine();

        QStringList dataLineItems = dataLine.split("\t");

        if (dataLineItems.count() != (16 + 3))
        {
            addLogLine("Error: File's \"" + fileInfo.fileName() + "\" data line doesn't have correct number of items (19). Data not read.");
            transformationFile.close();
            return;
        }
        else
        {
            double convertedItems[3];
            bool convOk = true;

            for (int itemIndex = 0; itemIndex < (3); itemIndex++)
            {
                convertedItems[itemIndex] = dataLineItems[itemIndex].toDouble(&convOk);

                if (!convOk)
                {
                    addLogLine("Error: File's \"" + fileInfo.fileName() +
                               "\" data line item #" + QString::number(itemIndex + 1) +
                               "can't be converted to double. Data not read.");

                    transformationFile.close();

                    break;
                }
            }

            if (convOk)
            {
                ui->doubleSpinBox_Translation_N->setValue(convertedItems[0]);
                ui->doubleSpinBox_Translation_E->setValue(convertedItems[1]);
                ui->doubleSpinBox_Translation_D->setValue(convertedItems[2]);

                for (int row = 0; row < 4; row++)
                {
                    for (int column = 0; column < 4; column++)
                    {
                        ui->tableWidget_TransformationMatrix->item(row, column)->setText(dataLineItems[3 + (row * 4) + column]);
                    }
                }
                addLogLine("File read.");
            }
        }
    }
    else
    {
        addLogLine("Error: can't open file \"" + fileInfo.fileName() + "\". Data not read.");
    }
}

void PostProcessingForm::on_pushButton_LoadTransformation_clicked()
{
    if (fileDialog_Transformation_Load.exec())
    {
        QStringList fileNames = fileDialog_Transformation_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Transformation_Load.setDirectory(QFileInfo(fileNames[0]).path());
            fileDialog_Transformation_Save.setDirectory(QFileInfo(fileNames[0]).path());

            loadTransformation(fileNames[0]);
        }
        else
        {
            addLogLine("Warning: No transformation file selected. Data not read.");
        }
    }
}

void PostProcessingForm::on_pushButton_SaveTransformation_clicked()
{
    if (fileDialog_Transformation_Save.exec())
    {
        QStringList fileNameList = fileDialog_Transformation_Save.selectedFiles();

        if (fileNameList.size() != 0)
        {
            fileDialog_Transformation_Save.setDirectory(QFileInfo(fileNameList[0]).path());
            fileDialog_Transformation_Load.setDirectory(QFileInfo(fileNameList[0]).path());
        }

        if (fileNameList.length() != 1)
        {
            addLogLine("Error: Multiple file selection not supported. Transformation not saved.");
            return;
        }

        QFile transformationFile;

        transformationFile.setFileName(fileNameList[0]);

        if (transformationFile.exists())
        {
            QMessageBox msgBox;
            msgBox.setText("File already exists.");
            msgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

            msgBox.setDefaultButton(cancelButton);

            msgBox.exec();

            if (msgBox.clickedButton() != overwriteButton)
            {
                addLogLine("Transformation not saved.");
                return;
            }
        }

        if (!transformationFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Error: Can't open transformation file.");
            return;
        }

        QTextStream textStream(&transformationFile);

        textStream << getTransformationFileHeaderLine() << "\n";

        textStream << QString::number(ui->doubleSpinBox_Translation_N->value()) << "\t"
                      << QString::number(ui->doubleSpinBox_Translation_E->value()) << "\t"
                      << QString::number(ui->doubleSpinBox_Translation_D->value());

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                textStream << "\t" << ui->tableWidget_TransformationMatrix->item(row, column)->text();
            }
        }

        textStream << "\n";
    }
}

void PostProcessingForm::on_pushButton_AddAllIncludingTransform_clicked()
{
    addAllData(true);
}

void PostProcessingForm::on_pushButton_Preset_clicked()
{
    int presetIndex = ui->comboBox_Presets->currentIndex();

    if ((presetIndex < 0) || (presetIndex > int(sizeof(transformationPresets) / sizeof(transformationPresets[0]))))
    {
        addLogLine("Error: index out of bounds (this should never happen...)");
    }
    else
    {
        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                ui->tableWidget_TransformationMatrix->item(row, column)->setText(QString::number(transformationPresets[presetIndex].values[row][column]));
            }
        }

        ui->tableWidget_TransformationMatrix->item(3, 3)->setText(QString::number(transformationPresets[presetIndex].values[3][3]));
    }
}

QString PostProcessingForm::getRoverIdentString(const unsigned int roverId)
{
    if (roverId < ('X' - 'A'))
    {
        return QString(char('A' + (char)roverId));
    }
    else
    {
        // Should not happen
        return("X");
    }
}

static QString getAntennaLocationsFileHeaderLine(void)
{
    QString line = "Rover\tCoord_N\tCoord_E\tCoord_D";
    return line;
}

void PostProcessingForm::loadAntennaLocations(const QString fileName)
{
    QFileInfo fileInfo(fileName);
    addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

    QFile antennaLocationsFile;
    antennaLocationsFile.setFileName(fileName);
    if (antennaLocationsFile.open(QIODevice::ReadOnly))
    {
        QTextStream textStream(&antennaLocationsFile);

        QString headerLine = textStream.readLine();

        if (headerLine.compare(getAntennaLocationsFileHeaderLine(), Qt::CaseInsensitive))
        {
            addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Data not read.");
            antennaLocationsFile.close();
            return;
        }

        QString coordItems[3][3];

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            QString dataLine = textStream.readLine();

            QStringList dataLineItems = dataLine.split("\t");

            if (dataLineItems.count() != (1 + 3))
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" line " + QString::number(roverIndex + 1) +
                           " doesn't have correct number of items (4). Data not read.");

                antennaLocationsFile.close();
                return;
            }
            else
            {
                QString expectedRoverIdent = "Rover " + getRoverIdentString(roverIndex);

                if (expectedRoverIdent.compare(dataLineItems[0], Qt::CaseInsensitive))
                {
                    addLogLine("Error: File's \"" + fileInfo.fileName() + "\" line " + QString::number(roverIndex + 1) +
                               " Rover ident string error. Data not read.");

                    return;
                }

                for (int i = 0; i < 3; i++)
                {
                    coordItems[roverIndex][i] = dataLineItems[i + 1];
                }
            }
        }

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            for (int coordIndex = 0; coordIndex < 3; coordIndex++)
            {
                ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, coordIndex)->setText(coordItems[roverIndex][coordIndex]);
            }
        }
    }
    else
    {
        addLogLine("Error: can't open file \"" + fileInfo.fileName() + "\". Data not read.");
    }
}

void PostProcessingForm::on_pushButton_LoadAntennaLocations_clicked()
{
    if (fileDialog_AntennaLocations_Load.exec())
    {
        QStringList fileNames = fileDialog_AntennaLocations_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_AntennaLocations_Load.setDirectory(QFileInfo(fileNames[0]).path());
            fileDialog_AntennaLocations_Save.setDirectory(QFileInfo(fileNames[0]).path());

            loadAntennaLocations(fileNames[0]);
        }
        else
        {
            addLogLine("Warning: No antenna locations file selected. Data not read.");
        }
    }
}

void PostProcessingForm::on_pushButton_SaveAntennaLocations_clicked()
{
    if (fileDialog_AntennaLocations_Save.exec())
    {
        QStringList fileNames = fileDialog_AntennaLocations_Save.selectedFiles();

        if (fileNames.length() != 1)
        {
            addLogLine("Error: Multiple file selection not supported. Antenna locations not saved.");
            return;
        }

        fileDialog_AntennaLocations_Save.setDirectory(QFileInfo(fileNames[0]).path());
        fileDialog_AntennaLocations_Load.setDirectory(QFileInfo(fileNames[0]).path());

        QFile antennaLocationsFile;
        antennaLocationsFile.setFileName(fileNames[0]);

        if (antennaLocationsFile.exists())
        {
            QMessageBox msgBox;
            msgBox.setText("File already exists.");
            msgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

            msgBox.setDefaultButton(cancelButton);

            msgBox.exec();

            if (msgBox.clickedButton() != overwriteButton)
            {
                addLogLine("Antenna locations not saved.");
                return;
            }
        }

        if (!antennaLocationsFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Error: Can't open antenna locations file.");
            return;
        }

        QTextStream textStream(&antennaLocationsFile);

        textStream << getAntennaLocationsFileHeaderLine() << "\n";

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            QString expectedRoverIdent = "Rover " + getRoverIdentString(roverIndex);

            textStream << expectedRoverIdent;

            for (int i = 0; i < 3; i++)
            {
                textStream << "\t" << ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, i)->text();
            }

            textStream << "\n";
        }
    }
}

bool PostProcessingForm::updateLOSolverReferencePointLocations(LOSolver& loSolver)
{
    Eigen::Vector3d antennaLocations[3];

    for (int roverIndex = 0; roverIndex < 3; roverIndex++)
    {
        bool ok;
        for (int valueIndex = 0; valueIndex < 3; valueIndex++)
        {
            antennaLocations[roverIndex](valueIndex) = ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, valueIndex)->text().toDouble(&ok);

            if (!ok)
            {
                addLogLine("Error: Row " + QString::number(roverIndex + 1) +
                           ", column " + QString::number(valueIndex + 1) +
                           " of reference point (=antenna) locations not convertible to a (double precision) floating point value. "
                           "Unable to update reference point locations.");

                return false;
            }
        }
    }

    if (!loSolver.setReferencePoints(antennaLocations))
    {
        addLogLine("Error: Can not set reference point (=antenna) locations, error code: " + QString::number(loSolver.getLastError()));

        return false;
    }

    return true;
}

void PostProcessingForm::on_pushButton_ValidateAntennaLocations_clicked()
{
    QMessageBox msgBox;

    LOSolver loSolver;

    if (updateLOSolverReferencePointLocations(loSolver))
    {
        msgBox.setText("Reference point (=antenna) locations valid.");
    }
    else
    {
        msgBox.setText("Reference point (=antenna) locations not valid.");
        msgBox.setInformativeText("See log for more info.");
    }

    msgBox.exec();
}

void PostProcessingForm::on_pushButton_LOSolver_GenerateScript_clicked()
{
    Eigen::Transform<double, 3, Eigen::Affine> transform_NEDToXYZ;

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    LOSolver loSolver;

    if (!updateLOSolverReferencePointLocations(loSolver))
    {
        return;
    }

    TransformMatrixGenerator matrixGenerator;

    Eigen::Transform<double, 3, Eigen::Affine> transform_Generated;

    try
    {
        QStringList lines = ui->plainTextEdit_LOSolver_TransformMatrixScript->document()->toPlainText().split("\n");

        transform_Generated = matrixGenerator.generate(lines).matrix();
    }
    catch (TransformMatrixGenerator::Issue& issue)
    {
        addLogLine("Generating transform matrix failed. Error: " + issue.text +
                   " Row: " + QString::number(issue.item.lineNumber + 1) + ", column: " + QString::number(issue.item.firstCol + 1));

        QTextCursor cursor = ui->plainTextEdit_LOSolver_TransformMatrixScript->textCursor();
        cursor.setPosition(0, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::NextBlock, QTextCursor::MoveAnchor, issue.item.lineNumber);
        if (issue.item.firstCol != -1)
        {
            cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::MoveAnchor, issue.item.firstCol);

            if (issue.item.lastCol != -1)
            {
                cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor, issue.item.lastCol - issue.item.firstCol + 1);
            }
        }

        ui->plainTextEdit_LOSolver_TransformMatrixScript->setTextCursor(cursor);
        ui->plainTextEdit_LOSolver_TransformMatrixScript->setFocus();

        return;
    }

    Eigen::Transform<double, 3, Eigen::Affine> transform_XYZToNED_NoTranslation;
    transform_XYZToNED_NoTranslation = transform_NEDToXYZ.linear().transpose();

    if (fileDialog_LOSolver_Script.exec())
    {
        QStringList fileNameList = fileDialog_LOSolver_Script.selectedFiles();

        if (fileNameList.size() != 0)
        {
            fileDialog_LOSolver_Script.setDirectory(QFileInfo(fileNameList[0]).path());
        }

        if (fileNameList.length() != 1)
        {
            addLogLine("Location/orientation script: Multiple file selection not supported. Script not created.");
            return;
        }

        QFile loScriptFile;

        loScriptFile.setFileName(fileNameList[0]);

        if (loScriptFile.exists())
        {
            QMessageBox msgBox;
            msgBox.setText("File already exists.");
            msgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

            msgBox.setDefaultButton(cancelButton);

            msgBox.exec();

            if (msgBox.clickedButton() != overwriteButton)
            {
                addLogLine("Location/orientation script not created.");
                return;
            }
        }

        if (!loScriptFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Can't open location/orientation script file.");
            return;
        }

        QTextStream textStream(&loScriptFile);

        addLogLine("Processing location/orientation script...");

        // Add some metadata to make possible changes in the future easier
        textStream << "META\tHEADER\tGNSS location/orientation script\n";
        textStream << "META\tVERSION\t1.0.1\n";
        textStream << "META\tFORMAT\tASCII\n";
        textStream << "META\tCONTENT\tDEFAULT\n";

        QString timeStampColumnText = "iTOW";

        if (ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex() == 1)
        {
            textStream << "META\tTIMESTAMPS\tUPTIME\n";
            timeStampColumnText = "Uptime";
        }
        textStream << "META\tEND\n";

        textStream << timeStampColumnText + "\t"
                      "Origin_X\tOrigin_Y\tOrigin_Z\t"
                      "Basis_XX\tBasis_XY\tBasis_XZ\t"
                      "Basis_YX\tBasis_YY\tBasis_YZ\t"
                      "Basis_ZX\tBasis_ZY\tBasis_ZZ\n";

        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = ui->spinBox_LOSolver_Movie_ITOW_Script_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = ui->spinBox_LOSolver_Movie_ITOW_Script_Max->value();

        //iTOWRange_Script_Min -= iTOWRange_Script_Min % expectedITOWAlignment; // Round to previous aligned ITOW

        UBXMessage_RELPOSNED::ITOW currentITOW = iTOWRange_Script_Min;

        unsigned int itemCount = 0;

        UBXMessage_RELPOSNED::ITOW iTOWMismatchStart = -1;
        unsigned int iTOWMismatchCount = 0;

        unsigned int warningCount = 0;

        while (currentITOW <= iTOWRange_Script_Max)
        {
            if (warningCount >= 1000)
            {
                addLogLine("Error: Maximum number of warnings (1000) reached. "
                           "Please check your data.");

                iTOWMismatchCount = 0;
                break;
            }

            QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterators[3];

            relposIterators[0] = rovers[0].relposnedMessages.lowerBound(currentITOW);
            relposIterators[1] = rovers[1].relposnedMessages.lowerBound(currentITOW);
            relposIterators[2] = rovers[2].relposnedMessages.lowerBound(currentITOW);

            bool endOfData = false;

            for (unsigned int i = 0; i < 3; i++)
            {
                if (relposIterators[i] == rovers[i].relposnedMessages.end())
                {
                    // No more data
                    endOfData = true;
                }
            }

            if (endOfData)
            {
                break;
            }

            UBXMessage_RELPOSNED::ITOW lowestNextRoverITOW = 1e9;

            for (unsigned int i = 0; i < 3; i++)
            {
                if (relposIterators[i].value().iTOW < lowestNextRoverITOW)
                {
                    lowestNextRoverITOW = relposIterators[i].value().iTOW;
                }
            }

            bool roverITOWSInSync = true;

            for (unsigned int i = 0; i < 3; i++)
            {
                if (relposIterators[i].value().iTOW != lowestNextRoverITOW)
                {
                    roverITOWSInSync = false;
                }
            }

            if (!roverITOWSInSync)
            {
                if (iTOWMismatchCount == 0)
                {
                    // First mismatch in this set

                    iTOWMismatchStart = lowestNextRoverITOW;
                    iTOWMismatchCount = 1;
                }
                else
                {
                    iTOWMismatchCount++;
                }

                currentITOW = lowestNextRoverITOW + 1;
                continue;
            }
            else if (iTOWMismatchCount != 0)
            {
                addLogLine("Warning: Mismatch in rover iTOWs, range: \"" +
                           QString::number(iTOWMismatchStart) + " - " + QString::number(lowestNextRoverITOW - 1) +
                           ", number of discarded iTOWS: " + QString::number(iTOWMismatchCount));

                iTOWMismatchCount = 0;
                warningCount++;
            }

            Eigen::Vector3d points[3];

            for (unsigned int i = 0; i < 3; i++)
            {
                points[i] = Eigen::Vector3d(relposIterators[i].value().relPosN, relposIterators[i].value().relPosE, relposIterators[i].value().relPosD);
            }

            if (!loSolver.setPoints(points))
            {
                addLogLine("Warning: Error setting points. ITOW: \"" +
                           QString::number(lowestNextRoverITOW) +
                           ", error code: " + QString::number(loSolver.getLastError()));

                currentITOW = lowestNextRoverITOW + 1;
                warningCount++;
                continue;
            }

            Eigen::Transform<double, 3, Eigen::Affine> loTransformNED;

            if (!loSolver.getTransformMatrix(loTransformNED))
            {
                addLogLine("Warning: Error calculating transform matrix. ITOW: \"" +
                           QString::number(lowestNextRoverITOW) +
                           ", error code: " + QString::number(loSolver.getLastError()));

                currentITOW = lowestNextRoverITOW + 1;
                warningCount++;
                continue;
            }

            Eigen::Transform<double, 3, Eigen::Affine> finalMatrix = transform_NEDToXYZ * loTransformNED * transform_Generated * transform_XYZToNED_NoTranslation;

            QString timeString;

            if (ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex() == 1)
            {
                qint64 timeSum = 0;
                bool fail = false;

                for (int i = 0; i < 3; i++)
                {
                    if (rovers[i].reverseSync.find(lowestNextRoverITOW) != rovers[i].reverseSync.end())
                    {
                        timeSum += rovers[i].reverseSync.find(lowestNextRoverITOW).value();
                    }
                    else
                    {
                        fail = true;
                        break;
                    }
                }

                if (fail)
                {
                    addLogLine("Warning: Can not find reverse sync (ITOW -> uptime) for all rovers. ITOW: \"" +
                               QString::number(lowestNextRoverITOW));

                    currentITOW = lowestNextRoverITOW + 1;
                    warningCount++;
                }

                timeSum /= 3;

                timeString = QString::number(timeSum);
            }
            else
            {
                timeString = QString::number(lowestNextRoverITOW);
            }

            const int originDecimals = 4;
            const int unitVectorDecimals = 6;

            QString lineOut =
                    timeString +

                    "\t" + QString::number(finalMatrix(0, 3), 'f', originDecimals) +
                    "\t" + QString::number(finalMatrix(1, 3), 'f', originDecimals) +
                    "\t" + QString::number(finalMatrix(2, 3), 'f', originDecimals) +

                    "\t" + QString::number(finalMatrix(0, 0), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(1, 0), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(2, 0), 'f', unitVectorDecimals) +

                    "\t" + QString::number(finalMatrix(0, 1), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(1, 1), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(2, 1), 'f', unitVectorDecimals) +

                    "\t" + QString::number(finalMatrix(0, 2), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(1, 2), 'f', unitVectorDecimals) +
                    "\t" + QString::number(finalMatrix(2, 2), 'f', unitVectorDecimals);
                    textStream << (lineOut + "\n");

            currentITOW = lowestNextRoverITOW + 1;

            itemCount++;
        }

        if (iTOWMismatchCount != 0)
        {
            addLogLine("Warning: Mismatch in rover iTOWs in the end of rover data, first ITOW: \"" +
                       QString::number(iTOWMismatchStart) +
                       ", number of discarded iTOWS: " + QString::number(iTOWMismatchCount));

            warningCount++;
        }

        addLogLine("Location/orientation script generated. Number of rows: " + QString::number(itemCount));
    }
}

void PostProcessingForm::addLidarData(const QStringList& fileNames)
{
    addLogLine("Reading lidar data...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile lidarFile;
        lidarFile.setFileName(fileName);
        if (lidarFile.open(QIODevice::ReadOnly))
        {
            QDataStream dataStream(&lidarFile);
            dataStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

            int64_t fileLength = lidarFile.size();

/*            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                lidarFile.close();
                continue;
            }
*/

            qint64 numberOfSamples = 0;
            unsigned int numberOfRounds = 0;
            unsigned int parseErrors = 0;
            unsigned int chunkIndex = 0;
            unsigned int firstDuplicateChunk = 0;
            qint64 firstDuplicateUptime = -1;
            unsigned int lastDuplicateChunk = 0;
            qint64 lastDuplicateUptime = -1;
            int discardedChunks = 0;

            while (!dataStream.atEnd())
            {
                if (parseErrors >= 100)
                {
                    addLogLine("Warning:  Maximum number of parse errors (100) reached. Your file is probably completely broken. Skipping the end of fle");
                    break;
                }

                if (fileLength - lidarFile.pos() < (unsigned int)(2 * sizeof(unsigned int)))
                {
                    addLogLine("Warning: Unexpected end of file (can not read header).");
                    break;
                }

                unsigned int dataType;
                unsigned int dataChunkLength;

                dataStream >> dataType >> dataChunkLength;

                if (lidarFile.pos() + dataChunkLength > fileLength)
                {
                    addLogLine("Warning: Unexpected end of file (chunk extends over the end of file).");
                    break;
                }

                chunkIndex++;

                switch (dataType)
                {
                case 1:
                {
                    if (dataChunkLength < sizeof(unsigned int))
                    {
                        addLogLine("Warning: Data chunk length less than the minimum. Skipping chunk.");
                        dataStream.skipRawData(dataChunkLength);
                        parseErrors++;
                        break;
                    }

                    unsigned int numOfItems;

                    dataStream >> numOfItems;

                    qint64 startTime;
                    qint64 endTime;

                    if (dataChunkLength != sizeof(numOfItems) + sizeof(startTime) + sizeof(endTime) + numOfItems * 3 * sizeof(float))
                    {
                        addLogLine("Warning: Data chunk length doesn't match with the number of items. Skipping chunk.");
                        dataStream.skipRawData(dataChunkLength - sizeof(numOfItems));
                        parseErrors++;
                        break;
                    }

                    LidarRound newRound;

                    dataStream >> startTime >> endTime;

                    newRound.startTime = startTime;
                    newRound.endTime = endTime;
                    newRound.fileName = fileName;
                    newRound.chunkIndex = chunkIndex;

                    if (lidarRounds.find(endTime) != lidarRounds.end())
                    {
                        if (firstDuplicateUptime == -1)
                        {
                            firstDuplicateChunk = chunkIndex;
                            firstDuplicateUptime = endTime;
                        }

                        lastDuplicateChunk = chunkIndex;
                        lastDuplicateUptime = endTime;

                        discardedChunks++;
                        dataStream.skipRawData(dataChunkLength - sizeof(numOfItems) - sizeof(startTime) - sizeof(endTime));
                        break;
                    }

                    if (firstDuplicateUptime != -1)
                    {
                        addLogLine("Warning: Chunk(s) " + QString::number(firstDuplicateChunk) +
                                   "-" + QString::number(lastDuplicateChunk) +
                                   " (uptime range: " + QString::number(firstDuplicateUptime) +
                                   "-" + QString::number(lastDuplicateUptime) +
                                   "): Distance(s) with duplicate uptime(s). Line(s) skipped.");

                        firstDuplicateUptime = -1;
                    }

                    for (unsigned int i = 0; i < numOfItems; i++)
                    {
                        RPLidarThread::DistanceItem newItem;
                        dataStream >> newItem.distance >> newItem.angle >> newItem.quality;
                        newRound.distanceItems.push_back(newItem);
                        numberOfSamples++;
                    }

                    lidarRounds[endTime] = newRound;

                    numberOfRounds++;
                    break;
                }
                default:
                    addLogLine("Warning: Unsupported data type (" + QString::number(dataType) + "). Skipping chunk.");
                    dataStream.skipRawData(dataChunkLength);
                    parseErrors++;
                    break;

                }
            }

            if (firstDuplicateUptime != -1)
            {
                addLogLine("Warning: Chunk(s) " + QString::number(firstDuplicateChunk) +
                           "-" + QString::number(lastDuplicateChunk) +
                           " (uptime range: " + QString::number(firstDuplicateUptime) +
                           "-" + QString::number(lastDuplicateUptime) +
                           "): Distance(s) with duplicate uptime(s). Line(s) skipped.");
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid lidar rounds: " +
                       QString::number(numberOfRounds) +
                       ", samples: " + QString::number(numberOfSamples) +
                       ", discarded chunks: " + QString::number(discardedChunks));

        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}

void PostProcessingForm::on_pushButton_AddLidarData_clicked()
{
    if (fileDialog_Lidar.exec())
    {
        QStringList fileNames = fileDialog_Lidar.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Lidar.setDirectory(QFileInfo(fileNames[0]).path());
            syncLogFileDialogDirectories(fileDialog_Lidar.directory().path(), true);
        }

        addLidarData(fileNames);
    }
}

void PostProcessingForm::on_pushButton_ClearLidarData_clicked()
{
    lidarRounds.clear();
    addLogLine("Lidar data cleared.");
}


void PostProcessingForm::on_pushButton_Lidar_GeneratePointClouds_clicked()
{
    generatePointClouds(SOURCE_LIDAR);
}

bool PostProcessingForm::generatePointCloudPointSet_Stylus(const Tag& beginningTag, const Tag& endingTag,
                                                           const qint64 beginningUptime, const qint64 endingUptime,
                                                           QTextStream* outStream,
                                                           const Eigen::Transform<double, 3, Eigen::Affine>& transform,
                                                           int& pointsWritten)
{
    double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
    bool includeNormals = ui->checkBox_Stylus_PointCloud_IncludeNormals->checkState();

    bool constDistancesOnly = true;

    QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(beginningUptime);

    while ((distIter != distances.end()) && (distIter.key() < endingUptime))
    {
        if (distIter.value().type == DistanceItem::Type::MEASURED)
        {
            constDistancesOnly = false;
            break;
        }

        distIter++;
    }

    int pointsBetweenTags = 0;

    if (constDistancesOnly)
    {
        distIter = distances.upperBound(beginningUptime);

        if (distIter != distances.end())
        {
            distIter--;

            if ((distIter == distances.end()) ||
                    (distIter.value().type != DistanceItem::Type::CONSTANT))
            {
                addLogLine("Warning: File \"" + endingTag.sourceFile + "\", line " +
                           QString::number(endingTag.sourceFileLine)+
                           ", uptime " + QString::number(endingUptime) +
                           ", iTOW " + QString::number(endingTag.iTOW) +
                           ": Points between tags having only constant distances without preceeding constant distance. Skipped.");

                return false;
            }

            stylusTipDistanceFromRoverA = distIter.value().distance;
        }

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.upperBound(beginningTag.iTOW);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.upperBound(beginningTag.iTOW);

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = rovers[0].relposnedMessages.upperBound(endingTag.iTOW);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = rovers[1].relposnedMessages.upperBound(endingTag.iTOW);

        while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
            (relposIterator_RoverB != relposIterator_RoverB_EndTag))
        {
            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                   (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
            {
                // Skip all rover A RELPOSNEDs that have lower iTOW than the next rover B RELPOSNED (sync)
                relposIterator_RoverA++;
            }

            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                   (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
            {
                // Skip all rover B RELPOSNEDs that have lower iTOW than the next rover A RELPOSNED (sync)
                relposIterator_RoverB++;
            }

            if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                (relposIterator_RoverB != relposIterator_RoverB_EndTag))
            {
                Eigen::Vector3d roverAPosNED(
                        relposIterator_RoverA.value().relPosN,
                        relposIterator_RoverA.value().relPosE,
                        relposIterator_RoverA.value().relPosD);

                Eigen::Vector3d roverBPosNED(
                        relposIterator_RoverB.value().relPosN,
                        relposIterator_RoverB.value().relPosE,
                        relposIterator_RoverB.value().relPosD);

                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                // Convert to XYZ-coordinates
                Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
                Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;
                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                QString lineOut;

                if (includeNormals)
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                }
                else
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                }

                outStream->operator<<(lineOut + "\n");

                pointsWritten++;

                pointsBetweenTags++;
                relposIterator_RoverA++;
                relposIterator_RoverB++;
            }
        }
    }
    else
    {
        // Distances found, sync point creation to them.
        // Rover coordinates are interpolated according to distance timestamps.

        distIter = distances.upperBound(beginningUptime);

        while ((distIter != distances.end()) && (distIter.key() < endingUptime))
        {
            if (distIter.value().type == DistanceItem::Type::CONSTANT)
            {
                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                           QString::number(distIter.value().sourceFileLine)+
                           ", uptime " + QString::number(distIter.key()) +
                           ": Constant distance between measured ones. Skipped.");
                distIter++;
                continue;
            }
            else if (distIter.value().type == DistanceItem::Type::MEASURED)
            {
                // Try to find next and previous rover coordinates
                // for this uptime

                qint64 distanceUptime = distIter.key();
                // TODO: Add/subtract fine tune sync value here if needed

                UBXMessage_RELPOSNED interpolated_Rovers[2];
                bool fail = false;

                for (int i = 0; i < 2; i++)
                {
                    QMap<qint64, RoverSyncItem>::const_iterator roverUptimeIter = rovers[i].roverSyncData.lowerBound(distanceUptime);

                    if (roverUptimeIter != rovers[i].roverSyncData.end())
                    {
                        const RoverSyncItem upperSyncItem = roverUptimeIter.value();
                        RoverSyncItem lowerSyncItem;
                        roverUptimeIter--;
                        if (roverUptimeIter != rovers[i].roverSyncData.end())
                        {
                            lowerSyncItem = roverUptimeIter.value();
                        }
                        else
                        {
                            addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + getRoverIdentString(i) + " sync data (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        if (rovers[i].relposnedMessages.find(upperSyncItem.iTOW) == rovers[i].relposnedMessages.end())
                        {
                            addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + getRoverIdentString(i) + " iTOW (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        if (rovers[i].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[i].relposnedMessages.end())
                        {
                            addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + getRoverIdentString(i) + " iTOW (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        qint64 timeDiff = distanceUptime - roverUptimeIter.key();

                        interpolated_Rovers[i] = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[i].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                rovers[i].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                    }
                    else
                    {
                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                   QString::number(distIter.value().sourceFileLine)+
                                   ", uptime " + QString::number(distIter.key()) +
                                   ": Can not find corresponding rover" + getRoverIdentString(i) + " sync data (upper limit). Skipped.");
                        distIter++;
                        fail = true;
                        break;
                    }
                }

                if (fail)
                {
                    continue;
                }

                stylusTipDistanceFromRoverA = distIter.value().distance;

                // TODO: Refine this quick hack or at least make the distance configurable!
                // Skip distances that are too far away
                // (measurement module seems to emit "outliers" now and then)
                if (stylusTipDistanceFromRoverA > 2)
                {
                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                               QString::number(distIter.value().sourceFileLine)+
                               ", uptime " + QString::number(distIter.key()) +
                               ": Distance between RoverA and tip too high (" +
                               QString::number(stylusTipDistanceFromRoverA) +
                               " m). Skipped.");
                    distIter++;
                    continue;
                }

                Eigen::Vector3d roverAPosNED(
                        interpolated_Rovers[0].relPosN,
                        interpolated_Rovers[0].relPosE,
                        interpolated_Rovers[0].relPosD);

                Eigen::Vector3d roverBPosNED(
                        interpolated_Rovers[1].relPosN,
                        interpolated_Rovers[1].relPosE,
                        interpolated_Rovers[1].relPosD);

                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                // Convert to XYZ-coordinates
                Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
                Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;
                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                QString lineOut;

                if (includeNormals)
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                }
                else
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                }

                outStream->operator<<(lineOut + "\n");
                pointsWritten++;
                pointsBetweenTags++;
            }
            else
            {
                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                           QString::number(distIter.value().sourceFileLine)+
                           ", uptime " + QString::number(distIter.key()) +
                           ": Unknown distance type between measured ones. Skipped.");
                distIter++;
                continue;
            }

            distIter++;
        }
    }

    return true;
}

bool PostProcessingForm::generatePointCloudPointSet_Lidar(const Tag& beginningTag, const Tag& endingTag,
                                       const qint64 beginningUptime, const qint64 endingUptime,
                                       QTextStream* outStream,
                                       const Eigen::Transform<double, 3, Eigen::Affine>& transform_NEDToXYZ,
                                       const Eigen::Transform<double, 3, Eigen::Affine>& transform_BeforeRotation,
                                       const Eigen::Transform<double, 3, Eigen::Affine>& transform_AfterRotation,
                                       LOInterpolator& loInterpolator, const RPLidarPlausibilityFilter::Settings& filteringSettings,
                                       int& pointsWritten)
{
    bool includeNormals = ui->checkBox_Lidar_PointCloud_IncludeNormals->checkState();
    bool normalLengthsAsQuality = ui->checkBox_Lidar_PointCloud_NormalLengthsAsQuality->checkState();
    int timeShift = ui->spinBox_Lidar_TimeShift->value();

    Eigen::Vector3d boundingSphere_Center = Eigen::Vector3d(ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value(),
                    ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value(),
                    ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value());

    double boundingSphere_Radius = ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value();

    QMap<qint64, LidarRound>::const_iterator lidarIter = lidarRounds.upperBound(beginningUptime);

    // As lidar rounds are "mapped" according to their arriving (=end) timestamps,
    // roll here to the first one with a bigger starting timestamp
    // to prevent taking "past" measurements into account

    while ((lidarIter != lidarRounds.end()) && (lidarIter.value().startTime < beginningUptime))
    {
        lidarIter++;
    }

    int pointsBetweenTags = 0;

    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;
    filteredItems.reserve(10000);

    RPLidarPlausibilityFilter plausibilityFilter;

    plausibilityFilter.setSettings(filteringSettings);

    while ((lidarIter != lidarRounds.end()) && (lidarIter.value().startTime < endingUptime))
    {
        plausibilityFilter.filter(lidarIter.value().distanceItems, filteredItems);

        // Q_ASSERT(lidarIter.value().distanceItems.count() == filteredItems.count());

        const LidarRound& round = lidarIter.value();        

        for (int i = 0; i < filteredItems.count(); i++)
        {
            const RPLidarPlausibilityFilter::FilteredItem& currentItem = filteredItems[i];

            if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
            {
                // Rover coordinates interpolated according to distance timestamps.

                qint64 itemUptime = round.startTime + (round.endTime - round.startTime) * i / lidarIter.value().distanceItems.count();
                UBXMessage_RELPOSNED interpolated_Rovers[3];

                qint64 roverUptime = itemUptime + timeShift;

                Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

                try
                {
                    loInterpolator.getInterpolatedLocationOrientationTransformMatrix(roverUptime, transform_LoSolver);
                }
                catch (QString& stringThrown)
                {
                    addLogLine("Warning: File \"" + lidarIter.value().fileName + "\", chunk index " +
                               QString::number(lidarIter.value().chunkIndex)+
                               ", uptime " + QString::number(lidarIter.key()) +
                               ": " + stringThrown + " Skipped the rest of this set of points " +
                               "between tags in lines " + QString::number(beginningTag.sourceFileLine) + " and " +
                               QString::number(endingTag.sourceFileLine) +
                               " in file \"" + beginningTag.sourceFile + "\".");
                    return(false);
                }


                Eigen::Transform<double, 3, Eigen::Affine> transform_LaserRotation;
                transform_LaserRotation = Eigen::AngleAxisd(currentItem.item.angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * (transform_LoSolver * (transform_AfterRotation * (transform_LaserRotation * (transform_BeforeRotation * Eigen::Vector3d::Zero()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserOriginBeforeRotation = transform_BeforeRotation * Eigen::Vector3d::Zero();
                Eigen::Vector3d laserOriginAfterRotation = transform_LaserRotation * laserOriginBeforeRotation;
                Eigen::Vector3d laserOriginAfterPostRotationTransform = transform_AfterRotation * laserOriginAfterRotation;
                Eigen::Vector3d laserOriginAfterLOSolverTransform = transform_LoSolver * laserOriginAfterPostRotationTransform;
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserOriginAfterLOSolverTransform;
                */

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (transform_AfterRotation * (transform_LaserRotation * (transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserVectorBeforeRotation = transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX());
                Eigen::Vector3d laserVectorAfterRotation = transform_LaserRotation * laserVectorBeforeRotation;
                Eigen::Vector3d laserVectorAfterPostRotationTransform = transform_AfterRotation * laserVectorAfterRotation;
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * laserVectorAfterPostRotationTransform;
                */

                if ((laserHitPosAfterLOSolverTransform - boundingSphere_Center).norm() <= boundingSphere_Radius)
                {
                    Eigen::Vector3d laserHitPosAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;

                    Eigen::Vector3d normal = (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).normalized();

                    if (normalLengthsAsQuality)
                    {
                        normal = (1. / (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).norm()) * normal;
                    }

                    QString lineOut;
                    if (includeNormals)
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4) +
                                "\t" + QString::number(normal(0), 'f', 4) +
                                "\t" + QString::number(normal(1), 'f', 4) +
                                "\t" + QString::number(normal(2), 'f', 4);
                    }
                    else
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4);
                    }

                    outStream->operator<<(lineOut + "\n");
                    pointsWritten++;
                    pointsBetweenTags++;
                }
            }
        }

        lidarIter++;
    }

    return true;
}

void PostProcessingForm::getLidarFilteringSettings(RPLidarPlausibilityFilter::Settings& lidarFilteringSettings)
{
    lidarFilteringSettings.startAngle = qDegreesToRadians(ui->doubleSpinBox_Lidar_Filtering_StartAngle->value());
    lidarFilteringSettings.endAngle = qDegreesToRadians(ui->doubleSpinBox_Lidar_Filtering_EndAngle->value());
    lidarFilteringSettings.qualityLimit_PreFiltering = ui->doubleSpinBox_Lidar_Filtering_Quality_Pre->value();
    lidarFilteringSettings.qualityLimit_PostFiltering = ui->doubleSpinBox_Lidar_Filtering_Quality_Post->value();
    lidarFilteringSettings.distanceLimit_Near = ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Near->value();
    lidarFilteringSettings.distanceLimit_Far = ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Far->value();
    lidarFilteringSettings.distanceDeltaLimit = ui->doubleSpinBox_Lidar_Filtering_DistanceDeltaLimit->value() * 360. / (2. * M_PI);
    lidarFilteringSettings.relativeSlopeLimit = ui->doubleSpinBox_Lidar_Filtering_RelativeDistanceSlopeLimit->value() * 360. / (2. * M_PI);
}

PostProcessingForm::LOInterpolator::LOInterpolator(PostProcessingForm* owner)
{
    this->owner = owner;
    for (int i = 0; i < 3; i++)
    {
        for (int ii = 0; ii < 2; ii++)
        {
            roverUptimeLimits[i][ii] = -1;
        }
    }
}

void PostProcessingForm::LOInterpolator::getInterpolatedLocationOrientationTransformMatrix(const qint64 uptime, Eigen::Transform<double, 3, Eigen::Affine>& transform)
{
    // TODO: This should use quaternions to work better.
    // As measurements are received 8 times/s typically,
    // interpolating coordinates may not be a big issue either.
    // If using quaternions, rover data should be ITOW-synced.

    UBXMessage_RELPOSNED interpolated_Rovers[3];

    for (int i = 0; i < 3; i++)
    {
        if ((uptime <= roverUptimeLimits[i][0]) || (uptime > roverUptimeLimits[i][1]))
        {
            // "Cache miss" -> Find new limiting values

            QMap<qint64, RoverSyncItem>::const_iterator roverUptimeIter = owner->rovers[i].roverSyncData.lowerBound(uptime);

            if (roverUptimeIter != owner->rovers[i].roverSyncData.end())
            {
                roverUptimeLimits[i][1] = roverUptimeIter.key();

                const RoverSyncItem upperSyncItem = roverUptimeIter.value();
                RoverSyncItem lowerSyncItem;
                roverUptimeIter--;
                if (roverUptimeIter != owner->rovers[i].roverSyncData.end())
                {
                    lowerSyncItem = roverUptimeIter.value();
                    roverUptimeLimits[i][0] = roverUptimeIter.key();
                }
                else
                {
                    roverUptimeLimits[i][0] = -1;
                    roverUptimeLimits[i][1] = -1;
                    throw QString("Can not find corresponding rover" + owner->getRoverIdentString(i) + " sync data (higher limit).");
                }

                if (owner->rovers[i].relposnedMessages.find(upperSyncItem.iTOW) == owner->rovers[i].relposnedMessages.end())
                {
                    roverUptimeLimits[i][0] = -1;
                    roverUptimeLimits[i][1] = -1;
                    throw QString("Can not find corresponding rover" + owner->getRoverIdentString(i) + " iTOW (higher limit).");
                }

                if (owner->rovers[i].relposnedMessages.find(lowerSyncItem.iTOW) == owner->rovers[i].relposnedMessages.end())
                {
                    roverUptimeLimits[i][0] = -1;
                    roverUptimeLimits[i][1] = -1;
                    throw QString("Can not find corresponding rover" + owner->getRoverIdentString(i) + " iTOW (lower limit).");
                }

                roverRELPOSNEDS_Lower[i] = owner->rovers[i].relposnedMessages.find(lowerSyncItem.iTOW).value();
                roverRELPOSNEDS_Upper[i] = owner->rovers[i].relposnedMessages.find(upperSyncItem.iTOW).value();
            }
            else
            {
                roverUptimeLimits[i][0] = -1;
                roverUptimeLimits[i][1] = -1;
                throw QString("Can not find corresponding rover" + owner->getRoverIdentString(i) + " sync data (upper limit).");
            }
        }

        qint64 timeDiff = uptime - roverUptimeLimits[i][0];

        interpolated_Rovers[i] = UBXMessage_RELPOSNED::interpolateCoordinates(roverRELPOSNEDS_Lower[i],
                                roverRELPOSNEDS_Upper[i], roverRELPOSNEDS_Lower[i].iTOW + timeDiff);
    }

    Eigen::Vector3d points[3] =
    {
        { interpolated_Rovers[0].relPosN, interpolated_Rovers[0].relPosE, interpolated_Rovers[0].relPosD },
        { interpolated_Rovers[1].relPosN, interpolated_Rovers[1].relPosE, interpolated_Rovers[1].relPosD },
        { interpolated_Rovers[2].relPosN, interpolated_Rovers[2].relPosE, interpolated_Rovers[2].relPosD },
    };

    if (!loSolver.setPoints(points))
    {
        throw QString("LOSolver.setPoints failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
    }

    if (!loSolver.getTransformMatrix(transform))
    {
        throw QString("LOSolver.setPoints failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
    }
}

bool PostProcessingForm::generateLidarTransformMatrices(Eigen::Transform<double, 3, Eigen::Affine>& transform_Lidar_Generated_BeforeRotation,
                                    Eigen::Transform<double, 3, Eigen::Affine>& transform_LidarGenerated_AfterRotation)
{
    TransformMatrixGenerator matrixGenerator;
    QPlainTextEdit *currentEditor;
    QString currentTabName;

    try
    {
        currentEditor = ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation;
        currentTabName = "Operations before rotation";
        QStringList lines = currentEditor->document()->toPlainText().split("\n");
        transform_Lidar_Generated_BeforeRotation = matrixGenerator.generate(lines).matrix();

        currentEditor = ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation;
        currentTabName = "Operations after rotation";
        lines = currentEditor->document()->toPlainText().split("\n");
        transform_LidarGenerated_AfterRotation = matrixGenerator.generate(lines).matrix();
    }
    catch (TransformMatrixGenerator::Issue& issue)
    {
        addLogLine("Generating transform matrix failed. Error: " + issue.text +
                   " Row: " + QString::number(issue.item.lineNumber + 1) + ", column: " + QString::number(issue.item.firstCol + 1) +
                   ". See tab\"" + currentTabName + "\".");

        QTextCursor cursor = currentEditor->textCursor();
        cursor.setPosition(0, QTextCursor::MoveAnchor);
        cursor.movePosition(QTextCursor::NextBlock, QTextCursor::MoveAnchor, issue.item.lineNumber);
        if (issue.item.firstCol != -1)
        {
            cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::MoveAnchor, issue.item.firstCol);

            if (issue.item.lastCol != -1)
            {
                cursor.movePosition(QTextCursor::NextCharacter, QTextCursor::KeepAnchor, issue.item.lastCol - issue.item.firstCol + 1);
            }
        }

        currentEditor->setTextCursor(cursor);
        currentEditor->setFocus();

        return false;
    }

    return true;
}


void PostProcessingForm::on_pushButton_Lidar_GenerateScript_clicked()
{
    Eigen::Transform<double, 3, Eigen::Affine> transform_NEDToXYZ;

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    int timeShift = ui->spinBox_Lidar_TimeShift->value();

    Eigen::Vector3d boundingSphere_Center = Eigen::Vector3d(ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value(),
                    ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value(),
                    ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value());

    double boundingSphere_Radius = ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value();

    bool convOk;
    qint64 uptime_Min = ui->lineEdit_Lidar_Script_UptimeRange_Min->text().toLongLong(&convOk);
    if (!convOk)
    {
        addLogLine("Invalid uptime range, min.");
        ui->lineEdit_Uptime_Min->setFocus();
        return;
    }

    qint64 uptime_Max = ui->lineEdit_Lidar_Script_UptimeRange_Max->text().toLongLong(&convOk);
    if (!convOk)
    {
        addLogLine("Invalid uptime range, max.");
        ui->lineEdit_Uptime_Max->setFocus();
        return;
    }

    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;
    filteredItems.reserve(10000);

    RPLidarPlausibilityFilter plausibilityFilter;
    RPLidarPlausibilityFilter::Settings lidarFilteringSettings;
    getLidarFilteringSettings(lidarFilteringSettings);
    plausibilityFilter.setSettings(lidarFilteringSettings);

    Eigen::Transform<double, 3, Eigen::Affine> transform_Lidar_Generated_BeforeRotation;
    Eigen::Transform<double, 3, Eigen::Affine> transform_Lidar_Generated_AfterRotation;

    if (!generateLidarTransformMatrices(transform_Lidar_Generated_BeforeRotation, transform_Lidar_Generated_AfterRotation))
    {
        return;
    }

    LOInterpolator loInterpolator_Lidar(this);

    if (!updateLOSolverReferencePointLocations(loInterpolator_Lidar.loSolver))
    {
        return;
    }

    if (!fileDialog_Lidar_Script.exec())
    {
        return;
    }

    QStringList fileNameList = fileDialog_Lidar_Script.selectedFiles();

    if (fileNameList.size() != 0)
    {
        fileDialog_Lidar_Script.setDirectory(QFileInfo(fileNameList[0]).path());
    }

    if (fileNameList.length() != 1)
    {
        addLogLine("Lidar script: Multiple file selection not supported. Script not created.");
        return;
    }

    QFile lidarScriptFile;

    lidarScriptFile.setFileName(fileNameList[0]);

    if (lidarScriptFile.exists())
    {
        QMessageBox msgBox;
        msgBox.setText("File already exists.");
        msgBox.setInformativeText("How to proceed?");

        QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
        QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

        msgBox.setDefaultButton(cancelButton);

        msgBox.exec();

        if (msgBox.clickedButton() != overwriteButton)
        {
            addLogLine("Lidar script not created.");
            return;
        }
    }

    if (!lidarScriptFile.open(QIODevice::WriteOnly))
    {
        addLogLine("Can't open lidar script file.");
        return;
    }

    QTextStream textStream(&lidarScriptFile);

    addLogLine("Processing lidar script...");

    // Add some metadata to make possible changes in the future easier
    textStream << "META\tHEADER\tGNSS-Stylus lidar script\n";
    textStream << "META\tVERSION\t1.0.0\n";
    textStream << "META\tFORMAT\tASCII\n";
    textStream << "META\tCONTENT\tDEFAULT\n";
    textStream << "META\tEND\n";

    textStream << "Uptime\tType\tDescr/subtype\t"
                  "RotAngle\t"
                  "Origin_X\tOrigin_Y\tOrigin_Z\t"
                  "Hit_X\tHit_Y\tHit_Z\n";

    QMap<qint64, LidarRound>::const_iterator lidarIter = lidarRounds.upperBound(uptime_Min);
    QMultiMap<qint64, Tag>::const_iterator tagIter = tags.begin();

    QString objectName;
    bool objectActive = false;
    bool scanningActive = false;
    bool ignoreBeginningAndEndingTags = false;
    qint64 beginningUptime = -1;
    Tag beginningTag;

    // Some locals to prevent excessive typing:
    QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
    QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
    QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();

    unsigned int pointsWritten = 0;

    while ((lidarIter.key() <= uptime_Max) && (lidarIter != lidarRounds.end()))
    {
        QString previousObjectName = objectName;
        bool previousObjectActive = objectActive;
        bool previousScanningActive = scanningActive;

        while ((tagIter.key() < lidarIter.value().startTime) && tagIter != tags.end())
        {
            // Roll tags to the current uptime to keep track of scanning state and object name

            QList<Tag> tagItems = tags.values(tagIter.key());

            Tag currentTag = tagIter.value();

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const Tag& currentTag = tagItems[i];

                qint64 tagUptime = tagIter.key();

                if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
                {
                    // Tag type: new object

                    if (currentTag.text.length() == 0)
                    {
                        // Empty name for the new object not allowed

                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": New object without a name. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        continue;
                    }

                    objectName = currentTag.text;
                    objectActive = true;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                }
                else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: Begin points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Duplicate beginning tag. Skipped.");
                        continue;
                    }

                    scanningActive = true;
                    beginningUptime = tagUptime;
                    beginningTag = currentTag;
                }
                else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: end points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag without beginning tag. Skipped.");
                        continue;
                    }

                    const Tag& endingTag = currentTag;

                    if (endingTag.sourceFile != beginningTag.sourceFile)
                    {
                        addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                                   beginningTag.sourceFile + "\", line " +
                                   QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                                   endingTag.sourceFile + "\", line " +
                                   QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                        continue;
                    }

                    beginningUptime = -1;
                    scanningActive = false;
                }
            }

            if (previousObjectName != objectName)
            {
                // Note: timeShift used here so that LOScript and this use the same timing

                textStream << QString::number(tagIter.key() + timeShift) +  "\tOBJECTNAME\t" + objectName + "\n";
            }

            if (!previousObjectActive && objectActive)
            {
                textStream << QString::number(tagIter.key() + timeShift) + "\tSTARTOBJECT\n";
            }

            if (previousObjectActive && !objectActive)
            {
                textStream << QString::number(tagIter.key() + timeShift) + "\tENDOBJECT\n";
            }

            if (!previousScanningActive && scanningActive)
            {
                textStream << QString::number(tagIter.key() + timeShift) + "\tSTARTSCAN\n";
            }

            if (previousScanningActive && !scanningActive)
            {
                textStream << QString::number(tagIter.key() + timeShift) + "\tENDSCAN\n";
            }

            tagIter++;
        }

        const LidarRound& round = lidarIter.value();

        plausibilityFilter.filter(round.distanceItems, filteredItems);

        for (int i = 0; i < filteredItems.count(); i++)
        {
            const RPLidarPlausibilityFilter::FilteredItem& currentItem = filteredItems[i];

            // Rover coordinates interpolated according to distance timestamps.

            qint64 itemUptime = round.startTime + (round.endTime - round.startTime) * i / lidarIter.value().distanceItems.count();
            UBXMessage_RELPOSNED interpolated_Rovers[3];

            qint64 roverUptime = itemUptime + timeShift;

            Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

            try
            {
                loInterpolator_Lidar.getInterpolatedLocationOrientationTransformMatrix(roverUptime, transform_LoSolver);
            }
            catch (QString& stringThrown)
            {
                addLogLine("Warning: File \"" + lidarIter.value().fileName + "\", chunk index " +
                           QString::number(lidarIter.value().chunkIndex)+
                           ", uptime " + QString::number(lidarIter.key()) +
                           ": " + stringThrown + " Lidar script generating terminated.");
                return;
            }

            Eigen::Transform<double, 3, Eigen::Affine> transform_LaserRotation;
            transform_LaserRotation = Eigen::AngleAxisd(currentItem.item.angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            // Lot of parentheses here to keep all calculations as matrix * vector
            // This is _much_ faster, in quick tests time was dropped from 510 s to 295 s when using parentheses in the whole lidarscript-creation)
            Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * (transform_LoSolver * (transform_Lidar_Generated_AfterRotation * (transform_LaserRotation * (transform_Lidar_Generated_BeforeRotation * Eigen::Vector3d::Zero()))));

            // Lot of parentheses here to keep all calculations as matrix * vector
            // This is _much_ faster, in quick tests time was dropped from 510 s to 295 s when using parentheses in the whole lidarscript-creation)
            Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (transform_Lidar_Generated_AfterRotation * (transform_LaserRotation * (transform_Lidar_Generated_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX()))));

            Eigen::Vector3d laserHitPosAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;

            QString descr;

            if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
            {
                if (objectActive)
                {
                    if (scanningActive)
                    {
                        if ((laserHitPosAfterLOSolverTransform - boundingSphere_Center).norm() <= boundingSphere_Radius)
                        {
                            descr = "H";
                        }
                        else
                        {
                            descr = "M";
                        }
                    }
                    else
                    {
                        descr = "NS";
                    }
                }
                else
                {
                    descr = "NO";
                }
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_ANGLE)
            {
                descr = "FA";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_QUALITY_PRE)
            {
                descr = "FQ1";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_QUALITY_POST)
            {
                descr = "FQ2";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_NEAR)
            {
                descr = "FDN";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_FAR)
            {
                descr = "FDF";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_DELTA)
            {
                descr = "FDD";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_SLOPE)
            {
                descr = "FS";
            }
            else
            {
                descr = "F?";
            }

            // Note: roverUptime used here so that LOScript and this use the same timing

            textStream << QString::number(roverUptime) + "\tL\t" + descr +
                          "\t" + QString::number(currentItem.item.angle, 'f', 2) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(0), 'f', 4) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(1), 'f', 4) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(2), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4) + "\n";

            pointsWritten++;
        }

        lidarIter++;
    }

    addLogLine("Lidar script generated. Number of points: " + QString::number(pointsWritten));
}

bool PostProcessingForm::loadOperations(QPlainTextEdit* plainTextEdit)
{
    if (fileDialog_Operations_Load.exec())
    {
        QStringList fileNames = fileDialog_Operations_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Operations_Load.setDirectory(QFileInfo(fileNames[0]).path());
            fileDialog_Operations_Save.setDirectory(QFileInfo(fileNames[0]).path());

            QString fileName = fileNames[0];

            QFileInfo fileInfo(fileName);
            addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

            QFile operationsFile;
            operationsFile.setFileName(fileName);
            if (operationsFile.open(QIODevice::ReadOnly))
            {
                plainTextEdit->setPlainText(QString::fromUtf8(operationsFile.readAll()));
                operationsFile.close();
                addLogLine("Operations read.");
                return true;
            }
            else
            {
                addLogLine("Error: Can't open file \"" + fileInfo.fileName() + "\".");
                return false;
            }
        }
        else
        {
            addLogLine("Warning: No operations file selected. Data not read.");
            return false;
        }
    }
    return false;
}

bool PostProcessingForm::saveOperations(QPlainTextEdit* plainTextEdit)
{
    if (fileDialog_Operations_Save.exec())
    {
        QStringList fileNameList = fileDialog_Operations_Save.selectedFiles();

        if (fileNameList.size() != 0)
        {
            fileDialog_Operations_Load.setDirectory(QFileInfo(fileNameList[0]).path());
            fileDialog_Operations_Save.setDirectory(QFileInfo(fileNameList[0]).path());
        }

        if (fileNameList.length() != 1)
        {
            addLogLine("Error: Multiple file selection not supported. Operations not saved.");
            return false;
        }

        QFile operationsFile;

        operationsFile.setFileName(fileNameList[0]);

        if (operationsFile.exists())
        {
            QMessageBox msgBox;
            msgBox.setText("File already exists.");
            msgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

            msgBox.setDefaultButton(cancelButton);

            msgBox.exec();

            if (msgBox.clickedButton() != overwriteButton)
            {
                addLogLine("Operations not saved.");
                return false;
            }
        }

        if (!operationsFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Error: Can't open operations file " + fileNameList[0] + ".");
            return false;
        }

        operationsFile.write(plainTextEdit->toPlainText().toUtf8());

        operationsFile.close();

        addLogLine("Operations saved.");

        return true;
    }
    else
    {
        return false;
    }
}

void PostProcessingForm::on_pushButton_Lidar_OperationsBeforeRotation_Load_clicked()
{
    loadOperations(ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation);
}

void PostProcessingForm::on_pushButton_Lidar_OperationsBeforeRotation_Save_clicked()
{
    saveOperations(ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation);
}

void PostProcessingForm::on_pushButton_Lidar_OperationsAfterRotation_Load_clicked()
{
    loadOperations(ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation);
}

void PostProcessingForm::on_pushButton_Lidar_OperationsAfterRotation_Save_clicked()
{
    saveOperations(ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation);
}

void PostProcessingForm::on_pushButton_LoSolver_Operations_Load_clicked()
{
    loadOperations(ui->plainTextEdit_LOSolver_TransformMatrixScript);
}

void PostProcessingForm::on_pushButton_LOSolver_Operations_Save_clicked()
{
    saveOperations(ui->plainTextEdit_LOSolver_TransformMatrixScript);
}
