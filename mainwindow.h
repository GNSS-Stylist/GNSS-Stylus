/*
    mainwindow.h (part of GNSS-Stylus)
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

/**
 * @file mainwindow.h
 * @brief Declaration for the main application form.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QSpinBox>
#include <QLabel>
#include <QCheckBox>

#include "serialthread.h"
#include "ntripthread.h"
#include "ubloxdatastreamprocessor.h"
#include "messagemonitorform.h"
#include "relposnedform.h"
#include "essentialsform.h"
#include "PostProcessing/postprocessform.h"
#include "laserrangefinder20hzv2messagemonitorform.h"
#include "laserrangefinder20hzv2serialthread.h"
#include "Lidar/rplidarthread.h"
#include "Lidar/rplidarmessagemonitorform.h"
#include "Lidar/lidarchartform.h"
#include "licensesform.h"

class MainWinRover : public QObject
{
    // "Correct" place for this would be inside MainWindow, but "Meta object features are not supported for nested classes".
    Q_OBJECT

public:

    class ExtUIThings
    {
    public:
        QLineEdit* lineEdit_SerialPort = nullptr;
        QSpinBox* spinBox_SerialSpeed = nullptr;
        QPushButton* pushButton_StartThread = nullptr;
        QPushButton* pushButton_TerminateThread = nullptr;
        QPushButton* pushButton_ShowMessageWindow = nullptr;
        QPushButton* pushButton_ShowRELPOSNEDWindow = nullptr;
        QCheckBox* checkBox_SuspendThread = nullptr;

        QLabel* label_RELPOSNEDMessageCount = nullptr;
        QPushButton* pushButton_ClearRELPOSNEDCounter = nullptr;
        QLabel* label_LastErrorMessage = nullptr;
        QPushButton* pushButton_ClearErrorMessage = nullptr;
        QLabel* label_LastWarningMessage = nullptr;
        QPushButton* pushButton_ClearWarningMessage = nullptr;
        QLabel* label_LastInfoMessage = nullptr;
        QPushButton* pushButton_ClearInfoMessage = nullptr;

        EssentialsForm* essentialsForm = nullptr;

    };

    MainWinRover(QWidget *parent, const int index, const ExtUIThings& uiThings);
    ~MainWinRover();

    void startSerialThread(void);
    void terminateSerialThread(void);
    QString getRoverIdentString(const unsigned int roverId);

    int index = 0;

    MessageMonitorForm* messageMonitorForm = nullptr;
    SerialThread* serialThread = nullptr;
    UBloxDataStreamProcessor ubloxDataStreamProcessor;
    int messageCounter_RELPOSNED = 0;
    RELPOSNEDForm* relposnedForm = nullptr;

    ExtUIThings extUIThings;


private slots:
    void on_commThread_ErrorMessage(const QString& errorMessage);
    void on_commThread_WarningMessage(const QString& warningMessage);
    void on_commThread_InfoMessage(const QString& infoMessage);
    void on_commThread_DataReceived(const QByteArray& data, const qint64 firstCharTime, const qint64 lastCharTime);
    void on_commThread_SerialTimeout(void);
    void on_ubloxProcessor_ubxMessageReceived(const UBXMessage&);

    void on_pushButton_StartThread_clicked();
    void on_pushButton_TerminateThread_clicked();
    void on_pushButton_ShowMessageWindow_clicked();
    void on_checkBox_SuspendThread_stateChanged(int arg1);
    void on_pushButton_ClearRELPOSNEDCounter_clicked();
    void on_pushButton_ClearErrorMessage_clicked();
    void on_pushButton_ClearWarningMessage_clicked();
    void on_pushButton_ClearInfoMessage_clicked();
    void on_pushButton_ShowRELPOSNEDWindow_clicked();

    friend class MainWindow;
};

namespace Ui {
class MainWindow;
}

/**
 * @brief Main application form.
 *
 * Doesn't really have much functionality on it's own, processing is mostly done under other forms/classes.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr); //!< Constructor
    ~MainWindow();

private slots:

    void commThread_Base_ErrorMessage(const QString& errorMessage);
    void commThread_Base_WarningMessage(const QString& warningMessage);
    void commThread_Base_InfoMessage(const QString& infoMessage);
    void commThread_Base_DataReceived(const QByteArray& data, const qint64 firstCharTime, const qint64 lastCharTime);
    void commThread_Base_SerialTimeout(void);
    void ubloxProcessor_Base_rtcmMessageReceived_Serial(const RTCMMessage&);

    void ntripThread_Base_ErrorMessage(const QString& errorMessage);
    void ntripThread_Base_WarningMessage(const QString& warningMessage);
    void ntripThread_Base_InfoMessage(const QString& infoMessage);
    void ntripThread_Base_DataReceived(const QByteArray& byte);
    void ntripThread_Base_ThreadEnded(void);
    void ubloxProcessor_Base_rtcmMessageReceived_NTRIP(const RTCMMessage&);

    void commThread_LaserRangeFinder20HzV2_ErrorMessage(const QString& errorMessage);
    void commThread_LaserRangeFinder20HzV2_WarningMessage(const QString& warningMessage);
    void commThread_LaserRangeFinder20HzV2_InfoMessage(const QString& infoMessage);
    void commThread_LaserRangeFinder20HzV2_DistanceReceived(const double& distance, qint64, qint64);
    void commThread_LaserRangeFinder20HzV2_ErrorReceived(const QString& errorString);
    void commThread_LaserRangeFinder20HzV2_UnidentifiedDataReceived(const QByteArray& data);

    void thread_RPLidar_ErrorMessage(const QString& errorMessage);
    void thread_RPLidar_WarningMessage(const QString& warningMessage);
    void thread_RPLidar_InfoMessage(const QString& infoMessage);
    void thread_RPLidar_DistanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64);
    void replay_RPLidar_DistanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64);

    void ubloxProcessor_Rover_ubxMessageReceived(const UBXMessage&, const unsigned int roverId);

    void on_pushButton_StartThread_Base_Serial_clicked();

    void on_pushButton_TerminateThread_Base_Serial_clicked();

    void on_checkBox_SuspendThread_Base_Serial_stateChanged(int arg1);

    void on_pushButton_ShowMessageWindow_Base_Serial_clicked();

    void on_pushButton_ClearRTCMCounter_Base_Serial_clicked();

    void on_pushButton_ClearErrorMessage_Base_Serial_clicked();

    void on_pushButton_ClearWarningMessage_Base_Serial_clicked();

    void on_pushButton_ClearInfoMessage_Base_Serial_clicked();

    void on_pushButton_ShowEssentialsWindow_clicked();

    void on_pushButton_ShowPostProcessingWindow_clicked();

    void on_pushButton_StartThread_Base_NTRIP_clicked();

    void on_pushButton_ClearRTCMCounter_Base_NTRIP_clicked();

    void on_pushButton_ClearErrorMessage_Base_NTRIP_clicked();

    void on_pushButton_ClearWarningMessage_Base_NTRIP_clicked();

    void on_pushButton_ClearInfoMessage_Base_NTRIP_clicked();

    void on_pushButton_ShowMessageWindow_NTRIP_clicked();

    void on_pushButton_TerminateThread_NTRIP_clicked();

    void on_pushButton_StartThread_LaserDist_clicked();

    void on_pushButton_ShowMessageWindow_LaserDist_clicked();

    void on_pushButton_TerminateThread_LaserDist_clicked();

    void on_MainWindow_destroyed();

    void on_doubleSpinBox_Distance_Constant_valueChanged(double arg1);

    void on_pushButton_StartThread_RPLidar_clicked();

    void on_pushButton_TerminateThread_RPLidar_clicked();

    void on_pushButton_ShowMessageWindow_RPLidar_clicked();

    void on_pushButton_ShowLidarChartWindow_RPLidar_clicked();

    void on_pushButton_ClearRoundCounter_RPLidar_clicked();

    void on_pushButton_ClearErrorMessage_RPLidar_clicked();

    void on_pushButton_ClearWarningMessage_RPLidar_clicked();

    void on_pushButton_ClearInfoMessage_RPLidar_clicked();

    void on_checkBox_SuspendThread_RPLidar_stateChanged(int arg1);

    void on_actionExit_triggered();

    void on_actionLicenses_triggered();

signals:
    void distanceChanged(const EssentialsForm::DistanceItem&);  //!< Signal emitted when distance changes

private:

    MainWinRover* rovers[3];

    Ui::MainWindow *ui;

    MessageMonitorForm* messageMonitorForm_Base_Serial = nullptr;
    SerialThread* serialThread_Base = nullptr;
    UBloxDataStreamProcessor ubloxDataStreamProcessor_Base_Serial;
    int messageCounter_RTCM_Base_Serial;

    MessageMonitorForm* messageMonitorForm_Base_NTRIP = nullptr;
    NTRIPThread* ntripThread = nullptr;
    UBloxDataStreamProcessor ubloxDataStreamProcessor_Base_NTRIP;
    int messageCounter_RTCM_Base_NTRIP;

    LaserRangeFinder20HzV2MessageMonitorForm* messageMonitorForm_LaserDist = nullptr;
    LaserRangeFinder20HzV2SerialThread* serialThread_LaserDist = nullptr;
    int messageCounter_LaserDist_Distance;

    RPLidarMessageMonitorForm* messageMonitorForm_RPLidar = nullptr;
    RPLidarThread* thread_RPLidar = nullptr;
    int messageCounter_RPLidar_Rounds;
    LidarChartForm* lidarChartForm = nullptr;

    EssentialsForm* essentialsForm;

    PostProcessingForm* postProcessingForm = nullptr;

    LicensesForm* licencesForm = nullptr;

    void closeEvent (QCloseEvent *event);

};

#endif // MAINWINDOW_H
