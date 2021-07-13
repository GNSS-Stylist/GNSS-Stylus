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
#include <QMessageBox>
#include <QtMath>

#include "postprocessingform.h"
#include "ui_postprocessform.h"
#include "transformmatrixgenerator.h"
#include "Stylus/moviescriptgenerator.h"
#include "Stylus/pointcloudgeneratorstylus.h"
#include "Lidar/pointcloudgeneratorlidar.h"
#include "loscriptgenerator.h"
#include "Lidar/lidarscriptgenerator.h"

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


void PostProcessingForm::loadParametersFromQSettings(QSettings& settings)
{
    ui->spinBox_ExpectedITOWAlignment->setValue(settings.value("PostProcessing_ExpectedITOWAlignment", ui->spinBox_ExpectedITOWAlignment->value()).toInt());
    ui->spinBox_ITOWAutoAlignThreshold->setValue(settings.value("PostProcessing_ITOWAutoAlignThreshold", ui->spinBox_ITOWAutoAlignThreshold->value()).toInt());

    ui->checkBox_ReportITOWAutoAlign->setChecked(settings.value("PostProcessing_ReportITOWAutoAlign", ui->checkBox_ReportITOWAutoAlign->isChecked()).toBool());
    ui->checkBox_ReportMissingITOWs->setChecked(settings.value("PostProcessing_ReportMissingITOWs", ui->checkBox_ReportMissingITOWs->isChecked()).toBool());
    ui->checkBox_ReportUnalignedITOWS->setChecked(settings.value("PostProcessing_ReportUnalignedITOWS", ui->checkBox_ReportUnalignedITOWS->isChecked()).toBool());

    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Fallback", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value()).toDouble());
    ui->lineEdit_TagIndicatingBeginningOfNewObject->setText(settings.value("PostProcessing_TagIndicatingBeginningOfNewObject", ui->lineEdit_TagIndicatingBeginningOfNewObject->text()).toString());
    ui->lineEdit_TagIndicatingBeginningOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingBeginningOfObjectPoints", ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text()).toString());
    ui->lineEdit_TagIndicatingEndOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingEndOfObjectPoints", ui->lineEdit_TagIndicatingEndOfObjectPoints->text()).toString());

    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Correction", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value()).toDouble());

    ui->spinBox_Lidar_TimeShift->setValue(settings.value("PostProcessing_Lidar_TimeShift", ui->spinBox_Lidar_TimeShift->value()).toInt());

    ui->doubleSpinBox_Lidar_Filtering_StartAngle->setValue(settings.value("PostProcessing_Lidar_Filtering_StartAngle", ui->doubleSpinBox_Lidar_Filtering_StartAngle->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_EndAngle->setValue(settings.value("PostProcessing_Lidar_Filtering_EndAngle", ui->doubleSpinBox_Lidar_Filtering_EndAngle->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_Quality_Pre->setValue(settings.value("PostProcessing_Lidar_Filtering_Quality_Pre", ui->doubleSpinBox_Lidar_Filtering_Quality_Pre->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_Quality_Post->setValue(settings.value("PostProcessing_Lidar_Filtering_Quality_Post", ui->doubleSpinBox_Lidar_Filtering_Quality_Post->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Near->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceLimit_Near", ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Near->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Far->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceLimit_Far", ui->doubleSpinBox_Lidar_Filtering_DistanceLimit_Far->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_DistanceDeltaLimit->setValue(settings.value("PostProcessing_Lidar_Filtering_DistanceDeltaLimit", ui->doubleSpinBox_Lidar_Filtering_DistanceDeltaLimit->value()).toDouble());
    ui->doubleSpinBox_Lidar_Filtering_RelativeDistanceSlopeLimit->setValue(settings.value("PostProcessing_Lidar_Filtering_RelativeDistanceSlopeLimit", ui->doubleSpinBox_Lidar_Filtering_RelativeDistanceSlopeLimit->value()).toDouble());

    ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_N", ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value()).toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_E", ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value()).toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Center_D", ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value()).toDouble());
    ui->doubleSpinBox_Lidar_BoundingSphere_Radius->setValue(settings.value("PostProcessing_Lidar_BoundingSphere_Radius", ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value()).toDouble());

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            QString settingKey = "PostProcessing_AntennaLocations_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->setText(settings.value(settingKey, ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->text()).toString());
        }
    }

    ui->doubleSpinBox_Translation_N->setValue(settings.value("PostProcessing_Translation_N", ui->doubleSpinBox_Translation_N->value()).toDouble());
    ui->doubleSpinBox_Translation_E->setValue(settings.value("PostProcessing_Translation_E", ui->doubleSpinBox_Translation_E->value()).toDouble());
    ui->doubleSpinBox_Translation_D->setValue(settings.value("PostProcessing_Translation_D", ui->doubleSpinBox_Translation_D->value()).toDouble());

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            QString settingKey = "PostProcessing_Transform_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            ui->tableWidget_TransformationMatrix->item(row, column)->setText(settings.value(settingKey, ui->tableWidget_TransformationMatrix->item(row, column)->text()).toString());
        }
    }


    ui->doubleSpinBox_ReplaySpeed->setValue(settings.value("PostProcessing_Replay_ReplaySpeed", ui->doubleSpinBox_ReplaySpeed->value()).toDouble());
    ui->doubleSpinBox_LimitInterval->setValue(settings.value("PostProcessing_Replay_IntervalLimit", ui->doubleSpinBox_LimitInterval->value()).toDouble());

    ui->lineEdit_Uptime_Min->setText(settings.value("PostProcessing_Replay_Uptime_Min", ui->lineEdit_Uptime_Min->text()).toString());
    ui->lineEdit_Uptime_Max->setText(settings.value("PostProcessing_Replay_Uptime_Max", ui->lineEdit_Uptime_Max->text()).toString());
    ui->checkBox_Looping->setChecked(settings.value("PostProcessing_Replay_Looping", ui->checkBox_Looping->isChecked()).toBool());


    ui->doubleSpinBox_Stylus_Movie_Camera_N->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_N", ui->doubleSpinBox_Stylus_Movie_Camera_N->value()).toDouble());
    ui->doubleSpinBox_Stylus_Movie_Camera_E->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_E", ui->doubleSpinBox_Stylus_Movie_Camera_E->value()).toDouble());
    ui->doubleSpinBox_Stylus_Movie_Camera_D->setValue(settings.value("PostProcessing_Stylus_Movie_Camera_D", ui->doubleSpinBox_Stylus_Movie_Camera_D->value()).toDouble());

    ui->doubleSpinBox_Stylus_Movie_LookAt_N->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_N", ui->doubleSpinBox_Stylus_Movie_LookAt_N->value()).toDouble());
    ui->doubleSpinBox_Stylus_Movie_LookAt_E->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_E", ui->doubleSpinBox_Stylus_Movie_LookAt_E->value()).toDouble());
    ui->doubleSpinBox_Stylus_Movie_LookAt_D->setValue(settings.value("PostProcessing_Stylus_Movie_LookAt_D", ui->doubleSpinBox_Stylus_Movie_LookAt_D->value()).toDouble());

    ui->spinBox_Stylus_Movie_ITOW_Points_Min->setValue(settings.value("PostProcessing_Stylus_Movie_ITOW_Points_Min", ui->spinBox_Stylus_Movie_ITOW_Points_Min->value()).toInt());
    ui->spinBox_Stylus_Movie_ITOW_Points_Max->setValue(settings.value("PostProcessing_Stylus_Movie_ITOW_Points_Max", ui->spinBox_Stylus_Movie_ITOW_Points_Max->value()).toInt());

    ui->spinBox_Stylus_Movie_ITOW_Script_Min->setValue(settings.value("PostProcessing_Stylus_Movie_ITOW_Script_Min", ui->spinBox_Stylus_Movie_ITOW_Script_Min->value()).toInt());
    ui->spinBox_Stylus_Movie_ITOW_Script_Max->setValue(settings.value("PostProcessing_Stylus_Movie_ITOW_Script_Max", ui->spinBox_Stylus_Movie_ITOW_Script_Max->value()).toInt());

    ui->doubleSpinBox_Stylus_Movie_FPS->setValue(settings.value("PostProcessing_Stylus_Movie_FPS", ui->doubleSpinBox_Stylus_Movie_FPS->value()).toDouble());

    ui->checkBox_Stylus_PointCloud_IncludeNormals->setChecked(settings.value("PostProcessing_Stylus_PointCloud_IncludeNormals", ui->checkBox_Stylus_PointCloud_IncludeNormals->isChecked()).toBool());

    ui->spinBox_LOSolver_Movie_ITOW_Script_Min->setValue(settings.value("PostProcessing_LOSolver_Movie_ITOW_Script_Min", ui->spinBox_LOSolver_Movie_ITOW_Script_Min->value()).toInt());
    ui->spinBox_LOSolver_Movie_ITOW_Script_Max->setValue(settings.value("PostProcessing_LOSolver_Movie_ITOW_Script_Max", ui->spinBox_LOSolver_Movie_ITOW_Script_Max->value()).toInt());

    ui->comboBox_LOSolver_Movie_TimeStamps->setCurrentIndex(settings.value("PostProcessing_LOSolver_Movie_Timestamps", ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex()).toInt());
    ui->plainTextEdit_LOSolver_TransformMatrixScript->setPlainText(settings.value("PostProcessing_LOSolver_TransformMatrixScript", ui->plainTextEdit_LOSolver_TransformMatrixScript->toPlainText()).toString());


    ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->setPlainText(settings.value("PostProcessing_Lidar_TransformMatrixScript_BeforeRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->toPlainText()).toString());
    ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->setPlainText(settings.value("PostProcessing_Lidar_TransformMatrixScript_AfterRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->toPlainText()).toString());

    ui->checkBox_Lidar_PointCloud_IncludeNormals->setChecked(settings.value("PostProcessing_Lidar_PointCloud_IncludeNormals", ui->checkBox_Lidar_PointCloud_IncludeNormals->isChecked()).toBool());
    ui->checkBox_Lidar_PointCloud_NormalLengthsAsQuality->setChecked(settings.value("PostProcessing_Lidar_PointCloud_NormalLengthAsQuality", ui->checkBox_Lidar_PointCloud_NormalLengthsAsQuality->isChecked()).toBool());


    ui->lineEdit_Lidar_Script_UptimeRange_Min->setText(settings.value("PostProcessing_Lidar_Script_Uptime_Min", ui->lineEdit_Lidar_Script_UptimeRange_Min->text()).toString());
    ui->lineEdit_Lidar_Script_UptimeRange_Max->setText(settings.value("PostProcessing_Lidar_Script_Uptime_Max", ui->lineEdit_Lidar_Script_UptimeRange_Max->text()).toString());
}


PostProcessingForm::PostProcessingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PostProcessingForm)
{
    ui->setupUi(this);

    QSettings settings;

    loadParametersFromQSettings(settings);

    ui->spinBox_MaxLogLines->setValue(settings.value("PostProcessing_MaxLogLines", ui->spinBox_MaxLogLines->value()).toInt());
}


void PostProcessingForm::saveParametersToQSettings(QSettings& settings)
{
    settings.setValue("PostProcessing_ExpectedITOWAlignment", ui->spinBox_ExpectedITOWAlignment->value());
    settings.setValue("PostProcessing_ITOWAutoAlignThreshold", ui->spinBox_ITOWAutoAlignThreshold->value());

    settings.setValue("PostProcessing_ReportITOWAutoAlign", ui->checkBox_ReportITOWAutoAlign->isChecked());
    settings.setValue("PostProcessing_ReportMissingITOWs", ui->checkBox_ReportMissingITOWs->isChecked());
    settings.setValue("PostProcessing_ReportUnalignedITOWS", ui->checkBox_ReportUnalignedITOWS->isChecked());

    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Fallback", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfNewObject", ui->lineEdit_TagIndicatingBeginningOfNewObject->text());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfObjectPoints", ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text());
    settings.setValue("PostProcessing_TagIndicatingEndOfObjectPoints", ui->lineEdit_TagIndicatingEndOfObjectPoints->text());
    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Correction", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value());

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

    settings.setValue("PostProcessing_Translation_N", ui->doubleSpinBox_Translation_N->value());
    settings.setValue("PostProcessing_Translation_E", ui->doubleSpinBox_Translation_E->value());
    settings.setValue("PostProcessing_Translation_D", ui->doubleSpinBox_Translation_D->value());

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

    settings.setValue("PostProcessing_Replay_ReplaySpeed", ui->doubleSpinBox_ReplaySpeed->value());
    settings.setValue("PostProcessing_Replay_IntervalLimit", ui->doubleSpinBox_LimitInterval->value());
    settings.setValue("PostProcessing_Replay_Uptime_Min", ui->lineEdit_Uptime_Min->text());
    settings.setValue("PostProcessing_Replay_Uptime_Max", ui->lineEdit_Uptime_Max->text());
    settings.setValue("PostProcessing_Replay_Looping", ui->checkBox_Looping->isChecked());


    settings.setValue("PostProcessing_Stylus_Movie_Camera_N", ui->doubleSpinBox_Stylus_Movie_Camera_N->value());
    settings.setValue("PostProcessing_Stylus_Movie_Camera_E", ui->doubleSpinBox_Stylus_Movie_Camera_E->value());
    settings.setValue("PostProcessing_Stylus_Movie_Camera_D", ui->doubleSpinBox_Stylus_Movie_Camera_D->value());

    settings.setValue("PostProcessing_Stylus_Movie_LookAt_N", ui->doubleSpinBox_Stylus_Movie_LookAt_N->value());
    settings.setValue("PostProcessing_Stylus_Movie_LookAt_E", ui->doubleSpinBox_Stylus_Movie_LookAt_E->value());
    settings.setValue("PostProcessing_Stylus_Movie_LookAt_D", ui->doubleSpinBox_Stylus_Movie_LookAt_D->value());

    settings.setValue("PostProcessing_Stylus_Movie_ITOW_Points_Min", ui->spinBox_Stylus_Movie_ITOW_Points_Min->value());
    settings.setValue("PostProcessing_Stylus_Movie_ITOW_Points_Max", ui->spinBox_Stylus_Movie_ITOW_Points_Max->value());
    settings.setValue("PostProcessing_Stylus_Movie_ITOW_Script_Min", ui->spinBox_Stylus_Movie_ITOW_Script_Min->value());
    settings.setValue("PostProcessing_Stylus_Movie_ITOW_Script_Max", ui->spinBox_Stylus_Movie_ITOW_Script_Max->value());

    settings.setValue("PostProcessing_Stylus_Movie_FPS", ui->doubleSpinBox_Stylus_Movie_FPS->value());


    settings.setValue("PostProcessing_Stylus_PointCloud_IncludeNormals", ui->checkBox_Stylus_PointCloud_IncludeNormals->isChecked());


    settings.setValue("PostProcessing_LOSolver_Movie_ITOW_Script_Min", ui->spinBox_LOSolver_Movie_ITOW_Script_Min->value());
    settings.setValue("PostProcessing_LOSolver_Movie_ITOW_Script_Max", ui->spinBox_LOSolver_Movie_ITOW_Script_Max->value());
    settings.setValue("PostProcessing_LOSolver_Movie_Timestamps", ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex());
    settings.setValue("PostProcessing_LOSolver_TransformMatrixScript", ui->plainTextEdit_LOSolver_TransformMatrixScript->toPlainText());

    settings.setValue("PostProcessing_Lidar_TransformMatrixScript_BeforeRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_BeforeRotation->toPlainText());
    settings.setValue("PostProcessing_Lidar_TransformMatrixScript_AfterRotation", ui->plainTextEdit_Lidar_TransformMatrixScript_AfterRotation->toPlainText());


    settings.setValue("PostProcessing_Lidar_PointCloud_IncludeNormals", ui->checkBox_Lidar_PointCloud_IncludeNormals->checkState() == Qt::Checked);
    settings.setValue("PostProcessing_Lidar_PointCloud_NormalLengthAsQuality", ui->checkBox_Lidar_PointCloud_NormalLengthsAsQuality->checkState() == Qt::Checked);


    settings.setValue("PostProcessing_Lidar_Script_Uptime_Min", ui->lineEdit_Lidar_Script_UptimeRange_Min->text());
    settings.setValue("PostProcessing_Lidar_Script_Uptime_Max", ui->lineEdit_Lidar_Script_UptimeRange_Max->text());
}


PostProcessingForm::~PostProcessingForm()
{
    QSettings settings;

    saveParametersToQSettings(settings);

    settings.setValue("PostProcessing_MaxLogLines", ui->spinBox_MaxLogLines->value());

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

    settings.setValue("PostProcessing_Directory_Dialog_Parameters_Load", fileDialog_Parameters_Load.directory().path());
    settings.setValue("PostProcessing_Directory_Dialog_Parameters_Save", fileDialog_Parameters_Save.directory().path());

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

        QStringList operationsFilters;

        operationsFilters << "Operations-files (*.Operations)"
                << "Any files (*)";

        fileDialog_Operations_Load.setNameFilters(operationsFilters);

        fileDialog_Operations_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_Operations_Save.setDefaultSuffix("Operations");

        fileDialog_Operations_Save.setNameFilters(operationsFilters);


        fileDialog_Parameters_Load.setFileMode(QFileDialog::ExistingFile);

        QStringList parametersFilters;

        parametersFilters << "Parameters-files (*.PPParameters)"
                << "Any files (*)";

        fileDialog_Parameters_Load.setNameFilters(parametersFilters);

        fileDialog_Parameters_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_Parameters_Save.setDefaultSuffix("PPParameters");

        fileDialog_Parameters_Save.setNameFilters(parametersFilters);


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

    fileDialog_Parameters_Load.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Parameters_Load").toString()));
    fileDialog_Parameters_Save.setDirectory(QDir(settings.value("PostProcessing_Directory_Dialog_Parameters_Save").toString()));
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

                QString lineRead = textStream.readLine();

                QString line;

                bool initialWhitespace = true;
                for (int i = 0; i < lineRead.length(); i++)
                {
                    if (initialWhitespace)
                    {
                        if ((lineRead[i] != ' ') && (lineRead[i] != '\t'))
                        {
                            initialWhitespace = false;
                        }
                        else
                        {
                            continue;
                        }
                    }

                    if ((i < lineRead.length() - 1) && (lineRead[i] == '/') && (lineRead[i + 1] == '/'))
                    {
                        // Rest of the line is comment -> skip it
                        break;
                    }
                    else
                    {
                        line += lineRead[i];
                    }
                }

                if (line.length() == 0)
                {
                    continue;
                }

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
    Eigen::Transform<double, 3, Eigen::Affine> transform_NEDToXYZ;

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    if (fileDialog_PointCloud.exec())
    {
        Stylus::PointCloudGenerator::Params params;

        params.transform_NEDToXYZ = &transform_NEDToXYZ;
        params.directory = fileDialog_PointCloud.directory();
        params.tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        params.tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        params.tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        params.initialStylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        params.includeNormals = ui->checkBox_Stylus_PointCloud_IncludeNormals->isChecked();

        params.tags = &tags;
        params.distances = &distances;
        params.rovers = rovers;

        Stylus::PointCloudGenerator pointCloudGenerator;

        QObject::connect(&pointCloudGenerator, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(on_infoMessage(const QString&)));

        QObject::connect(&pointCloudGenerator, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(on_warningMessage(const QString&)));

        QObject::connect(&pointCloudGenerator, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(on_errorMessage(const QString&)));

        pointCloudGenerator.generatePointClouds(params);
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

        Stylus::MovieScriptGenerator::Params params;

        params.fileName = fileNameList[0];
        params.transform = &transform;
        params.tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        params.tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        params.tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        params.initialStylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        params.iTOWRange_Lines_Min = ui->spinBox_Stylus_Movie_ITOW_Points_Min->value();
        params.iTOWRange_Lines_Max = ui->spinBox_Stylus_Movie_ITOW_Points_Max->value();
        params.expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();
        params.iTOWRange_Script_Min = ui->spinBox_Stylus_Movie_ITOW_Script_Min->value();
        params.iTOWRange_Script_Max = ui->spinBox_Stylus_Movie_ITOW_Script_Max->value();
        params.fps = ui->doubleSpinBox_Stylus_Movie_FPS->value();
        params.cameraNShift = ui->doubleSpinBox_Stylus_Movie_Camera_N->value();
        params.cameraEShift = ui->doubleSpinBox_Stylus_Movie_Camera_E->value();
        params.cameraDShift = ui->doubleSpinBox_Stylus_Movie_Camera_D->value();
        params.lookAtNShift = ui->doubleSpinBox_Stylus_Movie_LookAt_N->value();
        params.lookAtEShift = ui->doubleSpinBox_Stylus_Movie_LookAt_E->value();
        params.lookAtDShift = ui->doubleSpinBox_Stylus_Movie_LookAt_D->value();

        params.tags = &tags;
        params.distances = &distances;
        params.rovers = rovers;

        Stylus::MovieScriptGenerator movieScriptGenerator;

        QObject::connect(&movieScriptGenerator, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(on_infoMessage(const QString&)));

        QObject::connect(&movieScriptGenerator, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(on_warningMessage(const QString&)));

        QObject::connect(&movieScriptGenerator, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(on_errorMessage(const QString&)));

        movieScriptGenerator.GenerateMovieScript(params);
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

void PostProcessingForm::addAllData(const bool includeParameters)
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

        if (includeParameters)
        {
            fileNames = getAppendedFileNames(baseFileNames, ".PPParameters");

            if (fileNames.count() == 0)
            {
                addLogLine("Warning: No file(s) selected. Parameters not read.");
            }
            else
            {
                QString fileName = fileNames[0];

                if (fileNames.count() != 1)
                {
                    addLogLine("Warning: Multiple files selected. Parameters read only using the first one ("""+
                               fileName + """)");
                }

                QFileInfo fileInfo(fileName);
                addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

                if (QFile::exists(fileName))
                {
                    QSettings settings(fileName, QSettings::IniFormat);

                    loadParametersFromQSettings(settings);
                    addLogLine("Parameters (found from the file) read.");
                }
                else
                {
                    addLogLine("Error: Can't open file \"" + fileInfo.fileName() + "\".");
                }
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

void PostProcessingForm::on_pushButton_AddAllIncludingParams_clicked()
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

        LOScriptGenerator::Params params;

        params.transform_NEDToXYZ = &transform_NEDToXYZ;
        params.transform_Generated = &transform_Generated;
        params.iTOWRange_Script_Min = ui->spinBox_LOSolver_Movie_ITOW_Script_Min->value();
        params.iTOWRange_Script_Max = ui->spinBox_LOSolver_Movie_ITOW_Script_Max->value();

        params.fileName = fileNameList[0];
        params.timeStampFormat = ui->comboBox_LOSolver_Movie_TimeStamps->currentIndex() == 1 ? LOScriptGenerator::Params::TimeStampFormat::TSF_UPTIME : LOScriptGenerator::Params::TimeStampFormat::TSF_ITOW;
        params.loSolver = &loSolver;

        params.rovers = rovers;

        LOScriptGenerator loScriptGenerator;

        QObject::connect(&loScriptGenerator, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(on_infoMessage(const QString&)));

        QObject::connect(&loScriptGenerator, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(on_warningMessage(const QString&)));

        QObject::connect(&loScriptGenerator, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(on_errorMessage(const QString&)));

        loScriptGenerator.generateScript(params);
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
    Eigen::Transform<double, 3, Eigen::Affine> transform_NEDToXYZ;
    Eigen::Transform<double, 3, Eigen::Affine> transform_Lidar_Generated_BeforeRotation;
    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarGenerated_AfterRotation;
    LOInterpolator loInterpolator_Lidar(this);

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    RPLidarPlausibilityFilter::Settings lidarFilteringSettings;

    if (!generateLidarTransformMatrices(transform_Lidar_Generated_BeforeRotation, transform_LidarGenerated_AfterRotation))
    {
        return;
    }

    if (!updateLOSolverReferencePointLocations(loInterpolator_Lidar.loSolver))
    {
        return;
    }

    getLidarFilteringSettings(lidarFilteringSettings);

    if (fileDialog_PointCloud.exec())
    {
        Lidar::PointCloudGenerator::Params params;

        params.transform_NEDToXYZ = &transform_NEDToXYZ;
        params.transform_AfterRotation = &transform_LidarGenerated_AfterRotation;
        params.transform_BeforeRotation = &transform_Lidar_Generated_BeforeRotation;
        params.directory = fileDialog_PointCloud.directory();
        params.tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        params.tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        params.tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        params.includeNormals = ui->checkBox_Lidar_PointCloud_IncludeNormals->isChecked();
        params.normalLengthsAsQuality = ui->checkBox_Lidar_PointCloud_NormalLengthsAsQuality->isChecked();
        params.timeShift = ui->spinBox_Lidar_TimeShift->value();

        Eigen::Vector3d boundingSphere_Center = Eigen::Vector3d(ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value(),
                        ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value(),
                        ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value());

        params.boundingSphere_Center = &boundingSphere_Center;
        params.boundingSphere_Radius = ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value();

        params.tags = &tags;
        params.rovers = rovers;
        params.lidarRounds = &lidarRounds;
        params.lidarFilteringSettings = &lidarFilteringSettings;
        params.loInterpolator = &loInterpolator_Lidar;

        Lidar::PointCloudGenerator pointCloudGenerator;

        QObject::connect(&pointCloudGenerator, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(on_infoMessage(const QString&)));

        QObject::connect(&pointCloudGenerator, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(on_warningMessage(const QString&)));

        QObject::connect(&pointCloudGenerator, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(on_errorMessage(const QString&)));

        pointCloudGenerator.generatePointClouds(params);
    }
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
    Eigen::Transform<double, 3, Eigen::Affine> transform_Lidar_Generated_BeforeRotation;
    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarGenerated_AfterRotation;
    LOInterpolator loInterpolator_Lidar(this);

    if (!generateTransformationMatrix(transform_NEDToXYZ))
    {
        return;
    }

    RPLidarPlausibilityFilter::Settings lidarFilteringSettings;

    if (!generateLidarTransformMatrices(transform_Lidar_Generated_BeforeRotation, transform_LidarGenerated_AfterRotation))
    {
        return;
    }

    if (!updateLOSolverReferencePointLocations(loInterpolator_Lidar.loSolver))
    {
        return;
    }

    getLidarFilteringSettings(lidarFilteringSettings);

    if (fileDialog_Lidar_Script.exec())
    {
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

        Lidar::LidarScriptGenerator::Params params;

        params.transform_NEDToXYZ = &transform_NEDToXYZ;
        params.transform_AfterRotation = &transform_LidarGenerated_AfterRotation;
        params.transform_BeforeRotation = &transform_Lidar_Generated_BeforeRotation;
        params.fileName = fileNameList[0];
        params.tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        params.tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        params.tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        params.timeShift = ui->spinBox_Lidar_TimeShift->value();

        Eigen::Vector3d boundingSphere_Center = Eigen::Vector3d(ui->doubleSpinBox_Lidar_BoundingSphere_Center_N->value(),
                        ui->doubleSpinBox_Lidar_BoundingSphere_Center_E->value(),
                        ui->doubleSpinBox_Lidar_BoundingSphere_Center_D->value());

        params.boundingSphere_Center = &boundingSphere_Center;
        params.boundingSphere_Radius = ui->doubleSpinBox_Lidar_BoundingSphere_Radius->value();

        bool convOk;
        params.uptime_Min = ui->lineEdit_Lidar_Script_UptimeRange_Min->text().toLongLong(&convOk);
        if (!convOk)
        {
            addLogLine("Invalid uptime range, min.");
            ui->lineEdit_Uptime_Min->setFocus();
            return;
        }

        params.uptime_Max = ui->lineEdit_Lidar_Script_UptimeRange_Max->text().toLongLong(&convOk);
        if (!convOk)
        {
            addLogLine("Invalid uptime range, max.");
            ui->lineEdit_Uptime_Max->setFocus();
            return;
        }

        params.tags = &tags;
        params.rovers = rovers;
        params.lidarRounds = &lidarRounds;
        params.lidarFilteringSettings = &lidarFilteringSettings;
        params.loInterpolator = &loInterpolator_Lidar;

        Lidar::LidarScriptGenerator lidarScriptGenerator;

        QObject::connect(&lidarScriptGenerator, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(on_infoMessage(const QString&)));

        QObject::connect(&lidarScriptGenerator, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(on_warningMessage(const QString&)));

        QObject::connect(&lidarScriptGenerator, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(on_errorMessage(const QString&)));

        lidarScriptGenerator.generateLidarScript(params);
    }
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

void PostProcessingForm::on_infoMessage(const QString& infoString)
{
    addLogLine(infoString);
}

void PostProcessingForm::on_warningMessage(const QString& warningString)
{
    addLogLine("Warning: " + warningString);
}

void PostProcessingForm::on_errorMessage(const QString& errorString)
{
    addLogLine("Error: " + errorString);
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

void PostProcessingForm::on_pushButton_Replay_Uptime_Max_Maximize_clicked()
{
    ui->lineEdit_Uptime_Max->setText("9223372036854775807");
}

void PostProcessingForm::on_pushButton_Stylus_Movie_ITOW_Points_Max_Maximize_clicked()
{
    ui->spinBox_Stylus_Movie_ITOW_Points_Max->setValue(604800000);
}

void PostProcessingForm::on_pushButton_Stylus_Movie_ITOW_Script_Max_Maximize_clicked()
{
    ui->spinBox_Stylus_Movie_ITOW_Script_Max->setValue(604800000);
}

void PostProcessingForm::on_pushButton_LOSolver_Movie_ITOW_Script_Max_Maximize_clicked()
{
    ui->spinBox_LOSolver_Movie_ITOW_Script_Max->setValue(604800000);
}

void PostProcessingForm::on_pushButton_Lidar_Script_UptimeRange_Max_Maximize_clicked()
{
    ui->lineEdit_Lidar_Script_UptimeRange_Max->setText("9223372036854775807");
}

void PostProcessingForm::on_pushButton_LoadEditableFieldsFromFile_clicked()
{
    if (fileDialog_Parameters_Load.exec())
    {
        QStringList fileNames = fileDialog_Parameters_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Parameters_Load.setDirectory(QFileInfo(fileNames[0]).path());
            fileDialog_Parameters_Save.setDirectory(QFileInfo(fileNames[0]).path());

            QString fileName = fileNames[0];

            QFileInfo fileInfo(fileName);
            addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

            if (QFile::exists(fileName))
            {
                QSettings settings(fileName, QSettings::IniFormat);

                loadParametersFromQSettings(settings);
                addLogLine("Parameters (found from the file) read.");
            }
            else
            {
                addLogLine("Error: Can't open file \"" + fileInfo.fileName() + "\".");
            }
        }
        else
        {
            addLogLine("Warning: No operations file selected. Data not read.");
        }
    }
}

void PostProcessingForm::on_pushButton_SaveEditableFieldsToFile_clicked()
{
    if (fileDialog_Parameters_Save.exec())
    {
        QStringList fileNameList = fileDialog_Parameters_Save.selectedFiles();

        if (fileNameList.size() != 0)
        {
            fileDialog_Parameters_Load.setDirectory(QFileInfo(fileNameList[0]).path());
            fileDialog_Parameters_Save.setDirectory(QFileInfo(fileNameList[0]).path());
        }

        if (fileNameList.length() != 1)
        {
            addLogLine("Error: Multiple file selection not supported. Operations not saved.");
            return;
        }

        QString fileName = fileNameList[0];

        if (QFile::exists(fileName))
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
                addLogLine("Parameters not saved.");
                return;
            }
        }

        QSettings settings(fileName, QSettings::IniFormat);

        saveParametersToQSettings(settings);
        addLogLine("Parameters saved.");
    }
}


