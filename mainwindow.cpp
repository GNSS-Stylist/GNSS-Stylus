/*
    mainwindow.cpp (part of GNSS-Stylus)
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

#include <QSettings>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Lidar/rplidar_sdk/include/rplidar.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // These initialization are for QSettings to work without exclicitly creating every instance with names (see documentation of QSettings).
    QCoreApplication::setOrganizationName("GNSSStylusOrganization");
//    QCoreApplication::setOrganizationDomain("ImaginaryUltraCapitalisticPlanetDestroyingMoneyAndPowerLeechingParasiteCompany.com");
    QCoreApplication::setApplicationName("GNSSStylus");

    serialThread_Base = nullptr;
    ntripThread = nullptr;
    serialThread_LaserDist = nullptr;
    thread_RPLidar = nullptr;

    ui->setupUi(this);

    messageMonitorForm_Base_Serial = new MessageMonitorForm(parent, "Message monitor (Base, serial)");
    messageMonitorForm_Base_Serial->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor_Base_Serial);

    messageMonitorForm_Base_NTRIP = new MessageMonitorForm(parent, "Message monitor (Base, NTRIP)");
    messageMonitorForm_Base_NTRIP->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor_Base_NTRIP);

    for (unsigned int roverIndex = 0; roverIndex < sizeof(rovers) / sizeof(rovers[0]); roverIndex++)
    {
        // Initialize for safety
        rovers[roverIndex] = 0;
    }

    essentialsForm = new EssentialsForm(parent);

    // This way to handle multiple rover "UI-instances" isn't pretty
    // and should be refactored somehow. Not learning new Qt-tricks needed for this now
    // (give me VCL's TFrame, please)...

    MainWinRover::ExtUIThings roverAUIThings;

    roverAUIThings.lineEdit_SerialPort = ui->lineEdit_SerialPort_RoverA;
    roverAUIThings.spinBox_SerialSpeed = ui->spinBox_SerialSpeed_RoverA;
    roverAUIThings.pushButton_StartThread = ui->pushButton_StartThread_RoverA;
    roverAUIThings.pushButton_TerminateThread = ui->pushButton_TerminateThread_RoverA;

    roverAUIThings.pushButton_ShowMessageWindow = ui->pushButton_ShowMessageWindow_RoverA;
    roverAUIThings.pushButton_ShowRELPOSNEDWindow = ui->pushButton_ShowRELPOSNEDForm_RoverA;
    roverAUIThings.checkBox_SuspendThread = ui->checkBox_SuspendThread_RoverA;

    roverAUIThings.label_RELPOSNEDMessageCount = ui->label_RELPOSNEDMessageCount_RoverA;
    roverAUIThings.pushButton_ClearRELPOSNEDCounter = ui->pushButton_ClearRELPOSNEDCounter_RoverA;
    roverAUIThings.label_LastErrorMessage = ui->label_LastErrorMessage_RoverA;
    roverAUIThings.pushButton_ClearErrorMessage = ui->pushButton_ClearErrorMessage_RoverA;
    roverAUIThings.label_LastWarningMessage = ui->label_LastWarningMessage_RoverA;
    roverAUIThings.pushButton_ClearWarningMessage = ui->pushButton_ClearWarningMessage_RoverA;
    roverAUIThings.label_LastInfoMessage = ui->label_LastInfoMessage_RoverA;
    roverAUIThings.pushButton_ClearInfoMessage = ui->pushButton_ClearInfoMessage_RoverA;

    roverAUIThings.essentialsForm = essentialsForm;

    rovers[0] = new MainWinRover(parent, 0, roverAUIThings);

    MainWinRover::ExtUIThings roverBUIThings;

    roverBUIThings.lineEdit_SerialPort = ui->lineEdit_SerialPort_RoverB;
    roverBUIThings.spinBox_SerialSpeed = ui->spinBox_SerialSpeed_RoverB;
    roverBUIThings.pushButton_StartThread = ui->pushButton_StartThread_RoverB;
    roverBUIThings.pushButton_TerminateThread = ui->pushButton_TerminateThread_RoverB;

    roverBUIThings.pushButton_ShowMessageWindow = ui->pushButton_ShowMessageWindow_RoverB;
    roverBUIThings.pushButton_ShowRELPOSNEDWindow = ui->pushButton_ShowRELPOSNEDForm_RoverB;
    roverBUIThings.checkBox_SuspendThread = ui->checkBox_SuspendThread_RoverB;

    roverBUIThings.label_RELPOSNEDMessageCount = ui->label_RELPOSNEDMessageCount_RoverB;
    roverBUIThings.pushButton_ClearRELPOSNEDCounter = ui->pushButton_ClearRELPOSNEDCounter_RoverB;
    roverBUIThings.label_LastErrorMessage = ui->label_LastErrorMessage_RoverB;
    roverBUIThings.pushButton_ClearErrorMessage = ui->pushButton_ClearErrorMessage_RoverB;
    roverBUIThings.label_LastWarningMessage = ui->label_LastWarningMessage_RoverB;
    roverBUIThings.pushButton_ClearWarningMessage = ui->pushButton_ClearWarningMessage_RoverB;
    roverBUIThings.label_LastInfoMessage = ui->label_LastInfoMessage_RoverB;
    roverBUIThings.pushButton_ClearInfoMessage = ui->pushButton_ClearInfoMessage_RoverB;

    roverBUIThings.essentialsForm = essentialsForm;

    rovers[1] = new MainWinRover(parent, 1, roverBUIThings);

    MainWinRover::ExtUIThings roverCUIThings;

    roverCUIThings.lineEdit_SerialPort = ui->lineEdit_SerialPort_RoverC;
    roverCUIThings.spinBox_SerialSpeed = ui->spinBox_SerialSpeed_RoverC;
    roverCUIThings.pushButton_StartThread = ui->pushButton_StartThread_RoverC;
    roverCUIThings.pushButton_TerminateThread = ui->pushButton_TerminateThread_RoverC;

    roverCUIThings.pushButton_ShowMessageWindow = ui->pushButton_ShowMessageWindow_RoverC;
    roverCUIThings.pushButton_ShowRELPOSNEDWindow = ui->pushButton_ShowRELPOSNEDForm_RoverC;
    roverCUIThings.checkBox_SuspendThread = ui->checkBox_SuspendThread_RoverC;

    roverCUIThings.label_RELPOSNEDMessageCount = ui->label_RELPOSNEDMessageCount_RoverC;
    roverCUIThings.pushButton_ClearRELPOSNEDCounter = ui->pushButton_ClearRELPOSNEDCounter_RoverC;
    roverCUIThings.label_LastErrorMessage = ui->label_LastErrorMessage_RoverC;
    roverCUIThings.pushButton_ClearErrorMessage = ui->pushButton_ClearErrorMessage_RoverC;
    roverCUIThings.label_LastWarningMessage = ui->label_LastWarningMessage_RoverC;
    roverCUIThings.pushButton_ClearWarningMessage = ui->pushButton_ClearWarningMessage_RoverC;
    roverCUIThings.label_LastInfoMessage = ui->label_LastInfoMessage_RoverC;
    roverCUIThings.pushButton_ClearInfoMessage = ui->pushButton_ClearInfoMessage_RoverC;

    roverCUIThings.essentialsForm = essentialsForm;

    rovers[2] = new MainWinRover(parent, 2, roverCUIThings);

    messageMonitorForm_LaserDist = new LaserRangeFinder20HzV2MessageMonitorForm(parent, "Message monitor (\"Laser distance meter 20Hz V2\")");

    messageMonitorForm_RPLidar = new RPLidarMessageMonitorForm(parent, "Message monitor (RPLidar)");
    lidarChartForm = new LidarChartForm(parent);

    essentialsForm->connectUBloxDataStreamProcessorSlots_Base(&ubloxDataStreamProcessor_Base_Serial);
    essentialsForm->connectUBloxDataStreamProcessorSlots_Base(&ubloxDataStreamProcessor_Base_NTRIP);

    essentialsForm->connectUBloxDataStreamProcessorSlots_Rover(&rovers[0]->ubloxDataStreamProcessor, 0);
    essentialsForm->connectUBloxDataStreamProcessorSlots_Rover(&rovers[1]->ubloxDataStreamProcessor, 1);
    essentialsForm->connectUBloxDataStreamProcessorSlots_Rover(&rovers[2]->ubloxDataStreamProcessor, 2);

    connect(this, &MainWindow::distanceChanged,
                     essentialsForm, &EssentialsForm::on_distanceReceived);

    postProcessingForm = new PostProcessingForm(parent);

    connect(postProcessingForm, &PostProcessingForm::replayData_Rover,
                     this, &MainWindow::ubloxProcessor_Rover_ubxMessageReceived);

    connect(postProcessingForm, &PostProcessingForm::replayData_Lidar,
                     this, &MainWindow::replay_RPLidar_DistanceRoundReceived);

    essentialsForm->connectPostProcessingSlots(postProcessingForm);
    lidarChartForm->connectRPLidarPostProcessingSlots(postProcessingForm);

    connect(&ubloxDataStreamProcessor_Base_Serial, &UBloxDataStreamProcessor::rtcmMessageReceived,
                     this, &MainWindow::ubloxProcessor_Base_rtcmMessageReceived_Serial);

    connect(&ubloxDataStreamProcessor_Base_NTRIP, &UBloxDataStreamProcessor::rtcmMessageReceived,
                     this, &MainWindow::ubloxProcessor_Base_rtcmMessageReceived_NTRIP);

    licencesForm = new LicensesForm(parent);

    QSettings settings;

    ui->lineEdit_SerialPort_Base->setText(settings.value("SerialPort_Base", "\\\\.\\COM").toString());
    ui->spinBox_SerialSpeed_Base->setValue(settings.value("SerialSpeed_Base", "115200").toInt());

    ui->lineEdit_Command_Base_NTRIP->setText(settings.value("Command_Base_NTRIP", "-help").toString());

    ui->lineEdit_SerialPort_RPLidar->setText(settings.value("SerialPort_RPLidar", "\\\\.\\COM").toString());
    ui->spinBox_SerialSpeed_RPLidar->setValue(settings.value("SerialSpeed_RPLidar", "256000").toInt());
    ui->spinBox_MotorPWM_RPLidar->setValue(settings.value("MotorPWM_RPLidar", "660").toInt());
    ui->comboBox_ExpressScanMode->setCurrentIndex(settings.value("ExpressScanMode_RPLidar", "0").toInt());

    // Why is this needed? Q_ENUM should do the job? Different threads causing the need for this?
    qRegisterMetaType<SerialThread::DataReceivedEmitReason>();
}

MainWindow::~MainWindow()
{
    QSettings settings;
    settings.setValue("SerialPort_Base", ui->lineEdit_SerialPort_Base->text());
    settings.setValue("SerialSpeed_Base", ui->spinBox_SerialSpeed_Base->value());

    settings.setValue("Command_Base_NTRIP", ui->lineEdit_Command_Base_NTRIP->text());

    settings.setValue("SerialPort_RPLidar", ui->lineEdit_SerialPort_RPLidar->text());
    settings.setValue("SerialSpeed_RPLidar", ui->spinBox_SerialSpeed_RPLidar->value());
    settings.setValue("MotorPWM_RPLidar", ui->spinBox_MotorPWM_RPLidar->value());
    settings.setValue("ExpressScanMode_RPLidar", ui->comboBox_ExpressScanMode->currentIndex());

    delete messageMonitorForm_Base_Serial;
    delete messageMonitorForm_Base_NTRIP;
    delete messageMonitorForm_LaserDist;
    delete messageMonitorForm_RPLidar;
    delete lidarChartForm;

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        delete rovers[i];
        rovers[i] = nullptr;
    }
    delete essentialsForm;
    delete postProcessingForm;
    delete licencesForm;

    delete ui;
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    if (serialThread_Base)
    {
        serialThread_Base->requestTerminate();
    }

    if (ntripThread)
    {
        ntripThread->requestTerminate();
    }

    if (serialThread_LaserDist)
    {
        serialThread_LaserDist->requestTerminate();
    }

    if (thread_RPLidar)
    {
        thread_RPLidar->requestTerminate();
    }

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (rovers[i]->serialThread)
        {
            rovers[i]->serialThread->requestTerminate();
        }
    }

    messageMonitorForm_Base_Serial->close();
    messageMonitorForm_Base_NTRIP->close();

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        rovers[i]->messageMonitorForm->close();
        rovers[i]->relposnedForm->close();
    }

    essentialsForm->close();
    postProcessingForm->close();

    messageMonitorForm_LaserDist->close();
    messageMonitorForm_RPLidar->close();
    lidarChartForm->close();

    if (serialThread_Base)
    {
        serialThread_Base->wait(5000);
        serialThread_Base = nullptr;
    }

    if (ntripThread)
    {
        ntripThread->wait(5000);
        ntripThread = nullptr;
    }

    if (serialThread_LaserDist)
    {
        serialThread_LaserDist->wait(5000);
        serialThread_LaserDist = nullptr;
    }

    if (thread_RPLidar)
    {
        thread_RPLidar->wait(5000);
        thread_RPLidar = nullptr;
    }

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (rovers[i]->serialThread)
        {
            rovers[i]->serialThread->wait(5000);
            rovers[i]->serialThread = nullptr;
        }
    }

    event->accept();
}

void MainWindow::ubloxProcessor_Rover_ubxMessageReceived(const UBXMessage& ubxMessage, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        rovers[roverId]->ubloxProcessor_ubxMessageReceived(ubxMessage);
    }
}

void MainWindow::commThread_Base_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_Base_Serial->setText(infoMessage);
}

void MainWindow::commThread_Base_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_Base_Serial->setText(errorMessage);
}

void MainWindow::commThread_Base_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_Base_Serial->setText(warningMessage);
}

void MainWindow::commThread_Base_DataReceived(const QByteArray& data, const qint64 firstCharTime, const qint64 lastCharTime, const SerialThread::DataReceivedEmitReason&)
{
    ubloxDataStreamProcessor_Base_Serial.process(data, firstCharTime, lastCharTime);
}

void MainWindow::commThread_Base_SerialTimeout(void)
{
    if (ubloxDataStreamProcessor_Base_Serial.getNumOfUnprocessedBytes() != 0)
    {
        ui->label_LastWarningMessage_Base_Serial->setText(QString("Warning: discarded ") + QString::number(ubloxDataStreamProcessor_Base_Serial.getNumOfUnprocessedBytes()) + " unprocessed bytes due to serial timeout.");
    }

    ubloxDataStreamProcessor_Base_Serial.flushInputBuffer();
}

void MainWindow::ubloxProcessor_Base_rtcmMessageReceived_Serial(const RTCMMessage& rtcmMessage)
{
    messageCounter_RTCM_Base_Serial++;
    ui->label_RTCMMessageCount_Base_Serial->setText(QString::number(messageCounter_RTCM_Base_Serial));

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (rovers[i]->serialThread)
        {
            rovers[i]->serialThread->addToSendQueue(rtcmMessage.rawMessage);
        }
    }
}

void MainWindow::on_pushButton_StartThread_Base_Serial_clicked()
{
    if (!serialThread_Base)
    {
        serialThread_Base = new SerialThread(ui->lineEdit_SerialPort_Base->text(), 20, 1, ui->spinBox_SerialSpeed_Base->value());
        if (ui->checkBox_SuspendThread_Base_Serial->isChecked())
        {
            serialThread_Base->suspend();
        }

        connect(serialThread_Base, &SerialThread::infoMessage,
                         this, &MainWindow::commThread_Base_InfoMessage);

        connect(serialThread_Base, &SerialThread::warningMessage,
                         this, &MainWindow::commThread_Base_WarningMessage);

        connect(serialThread_Base, &SerialThread::errorMessage,
                         this, &MainWindow::commThread_Base_ErrorMessage);

        connect(serialThread_Base, &SerialThread::dataReceived,
                         this, &MainWindow::commThread_Base_DataReceived);

        connect(serialThread_Base, &SerialThread::serialTimeout,
                         this, &MainWindow::commThread_Base_SerialTimeout);

        messageMonitorForm_Base_Serial->connectSerialThreadSlots(serialThread_Base);
        essentialsForm->connectSerialThreadSlots_Base(serialThread_Base);

        serialThread_Base->start();

        ui->lineEdit_SerialPort_Base->setEnabled(false);
        ui->spinBox_SerialSpeed_Base->setEnabled(false);
        ui->pushButton_StartThread_Base_Serial->setEnabled(false);
        ui->pushButton_TerminateThread_Base_Serial->setEnabled(true);
        ui->pushButton_StartThread_Base_NTRIP->setEnabled(false);
        ui->pushButton_TerminateThread_NTRIP->setEnabled(false);

        messageCounter_RTCM_Base_Serial = 0;
        ui->label_RTCMMessageCount_Base_Serial->setText(QString::number(messageCounter_RTCM_Base_Serial));
        ui->label_LastInfoMessage_Base_Serial->setText("");
        ui->label_LastWarningMessage_Base_Serial->setText("");
        ui->label_LastErrorMessage_Base_Serial->setText("");
    }
}

void MainWindow::on_pushButton_TerminateThread_Base_Serial_clicked()
{
    if (serialThread_Base)
    {
        serialThread_Base->requestTerminate();
        serialThread_Base->wait(5000);

        disconnect(serialThread_Base, &SerialThread::infoMessage,
                         this, &MainWindow::commThread_Base_InfoMessage);

        disconnect(serialThread_Base, &SerialThread::warningMessage,
                         this, &MainWindow::commThread_Base_WarningMessage);

        disconnect(serialThread_Base, &SerialThread::errorMessage,
                         this, &MainWindow::commThread_Base_ErrorMessage);

        disconnect(serialThread_Base, &SerialThread::dataReceived,
                         this, &MainWindow::commThread_Base_DataReceived);

        disconnect(serialThread_Base, &SerialThread::serialTimeout,
                         this, &MainWindow::commThread_Base_SerialTimeout);

        messageMonitorForm_Base_Serial->disconnectSerialThreadSlots(serialThread_Base);
        essentialsForm->disconnectSerialThreadSlots_Base(serialThread_Base);

        delete serialThread_Base;
        serialThread_Base = nullptr;

        ui->lineEdit_SerialPort_Base->setEnabled(true);
        ui->spinBox_SerialSpeed_Base->setEnabled(true);
        ui->pushButton_StartThread_Base_Serial->setEnabled(true);
        ui->pushButton_TerminateThread_Base_Serial->setEnabled(false);
        ui->pushButton_StartThread_Base_NTRIP->setEnabled(true);
        ui->pushButton_TerminateThread_NTRIP->setEnabled(false);
    }
}

void MainWindow::on_checkBox_SuspendThread_Base_Serial_stateChanged(int arg1)
{
    if (serialThread_Base)
    {
        if (arg1 == Qt::Checked)
        {
            serialThread_Base->suspend();
        }
        else
        {
            serialThread_Base->resume();
        }
    }
}

void MainWindow::on_pushButton_ShowMessageWindow_Base_Serial_clicked()
{
    messageMonitorForm_Base_Serial->show();
    messageMonitorForm_Base_Serial->raise();
    messageMonitorForm_Base_Serial->activateWindow();
}

void MainWindow::on_pushButton_ClearRTCMCounter_Base_Serial_clicked()
{
    messageCounter_RTCM_Base_Serial = 0;
    ui->label_RTCMMessageCount_Base_Serial->setText(QString::number(messageCounter_RTCM_Base_Serial));
}

void MainWindow::on_pushButton_ClearErrorMessage_Base_Serial_clicked()
{
    ui->label_LastErrorMessage_Base_Serial->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_Base_Serial_clicked()
{
    ui->label_LastWarningMessage_Base_Serial->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_Base_Serial_clicked()
{
    ui->label_LastInfoMessage_Base_Serial->setText("");
}

void MainWindow::on_pushButton_ShowEssentialsWindow_clicked()
{
    essentialsForm->show();
    essentialsForm->raise();
    essentialsForm->activateWindow();

}

void MainWindow::on_pushButton_ShowPostProcessingWindow_clicked()
{
    postProcessingForm->show();
    postProcessingForm->raise();
    postProcessingForm->activateWindow();
}

void MainWindow::on_pushButton_StartThread_Base_NTRIP_clicked()
{
    if (!ntripThread)
    {
        ntripThread = new NTRIPThread(ui->lineEdit_Command_Base_NTRIP->text());

        connect(ntripThread, &NTRIPThread::infoMessage,
                         this, &MainWindow::ntripThread_Base_InfoMessage);

        connect(ntripThread, &NTRIPThread::warningMessage,
                         this, &MainWindow::ntripThread_Base_WarningMessage);

        connect(ntripThread, &NTRIPThread::errorMessage,
                         this, &MainWindow::ntripThread_Base_ErrorMessage);

        connect(ntripThread, &NTRIPThread::dataReceived,
                         this, &MainWindow::ntripThread_Base_DataReceived);

        connect(ntripThread, &NTRIPThread::threadEnded,
                         this, &MainWindow::ntripThread_Base_ThreadEnded);

//        connect(ntripThread, SIGNAL(serialTimeout(void)),
//                         this, SLOT(ntripThread_Base_SerialTimeout(void)));

        messageMonitorForm_Base_NTRIP->connectNTRIPThreadSlots(ntripThread);
        essentialsForm->connectNTRIPThreadSlots_Base(ntripThread);

        ntripThread->start();

        ui->lineEdit_Command_Base_NTRIP->setEnabled(false);
        ui->pushButton_StartThread_Base_Serial->setEnabled(false);
        ui->pushButton_TerminateThread_Base_Serial->setEnabled(false);
        ui->pushButton_StartThread_Base_NTRIP->setEnabled(false);
        ui->pushButton_TerminateThread_NTRIP->setEnabled(true);

        messageCounter_RTCM_Base_NTRIP = 0;
        ui->label_RTCMMessageCount_Base_NTRIP->setText(QString::number(messageCounter_RTCM_Base_NTRIP));
        ui->label_LastInfoMessage_Base_NTRIP->setText("");
        ui->label_LastWarningMessage_Base_NTRIP->setText("");
        ui->label_LastErrorMessage_Base_NTRIP->setText("");
    }
}

void MainWindow::ntripThread_Base_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_Base_NTRIP->setText(infoMessage.trimmed());
}

void MainWindow::ntripThread_Base_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_Base_NTRIP->setText(errorMessage.trimmed());
}

void MainWindow::ntripThread_Base_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_Base_NTRIP->setText(warningMessage.trimmed());
}

void MainWindow::ntripThread_Base_DataReceived(const QByteArray& data)
{
    QElapsedTimer timer;

    timer.start();

    qint64 timestamp = timer.msecsSinceReference();

    ubloxDataStreamProcessor_Base_NTRIP.process(data, timestamp, timestamp);
}

void MainWindow::ubloxProcessor_Base_rtcmMessageReceived_NTRIP(const RTCMMessage& rtcmMessage)
{
    messageCounter_RTCM_Base_NTRIP++;
    ui->label_RTCMMessageCount_Base_NTRIP->setText(QString::number(messageCounter_RTCM_Base_NTRIP));

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (rovers[i]->serialThread)
        {
            rovers[i]->serialThread->addToSendQueue(rtcmMessage.rawMessage);
        }
    }
}


void MainWindow::ntripThread_Base_ThreadEnded(void)
{
    if (ntripThread)
    {
        ntripThread->wait(5000);

        disconnect(ntripThread, &NTRIPThread::infoMessage,
                         this, &MainWindow::ntripThread_Base_InfoMessage);

        disconnect(ntripThread, &NTRIPThread::warningMessage,
                         this, &MainWindow::ntripThread_Base_WarningMessage);

        disconnect(ntripThread, &NTRIPThread::errorMessage,
                         this, &MainWindow::ntripThread_Base_ErrorMessage);

        disconnect(ntripThread, &NTRIPThread::dataReceived,
                         this, &MainWindow::ntripThread_Base_DataReceived);

        disconnect(ntripThread, &NTRIPThread::threadEnded,
                         this, &MainWindow::ntripThread_Base_ThreadEnded);

//        disconnect(ntripThread, SIGNAL(serialTimeout(void)),
//                         this, SLOT(ntripThread_SerialTimeout(void)));

        messageMonitorForm_Base_NTRIP->disconnectNTRIPThreadSlots(ntripThread);
        essentialsForm->disconnectNTRIPThreadSlots_Base(ntripThread);

        delete ntripThread;
        ntripThread = nullptr;

        ui->lineEdit_Command_Base_NTRIP->setEnabled(true);
        ui->pushButton_StartThread_Base_Serial->setEnabled(true);
        ui->pushButton_TerminateThread_Base_Serial->setEnabled(false);
        ui->pushButton_StartThread_Base_NTRIP->setEnabled(true);
        ui->pushButton_TerminateThread_NTRIP->setEnabled(false);
    }
}

void MainWindow::on_pushButton_ShowMessageWindow_NTRIP_clicked()
{
    messageMonitorForm_Base_NTRIP->show();
    messageMonitorForm_Base_NTRIP->raise();
    messageMonitorForm_Base_NTRIP->activateWindow();
}

void MainWindow::on_pushButton_ClearRTCMCounter_Base_NTRIP_clicked()
{
    messageCounter_RTCM_Base_NTRIP = 0;
    ui->label_RTCMMessageCount_Base_NTRIP->setText(QString::number(messageCounter_RTCM_Base_NTRIP));
}

void MainWindow::on_pushButton_ClearErrorMessage_Base_NTRIP_clicked()
{
    ui->label_LastErrorMessage_Base_NTRIP->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_Base_NTRIP_clicked()
{
    ui->label_LastWarningMessage_Base_NTRIP->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_Base_NTRIP_clicked()
{
    ui->label_LastInfoMessage_Base_NTRIP->setText("");
}

void MainWindow::on_pushButton_TerminateThread_NTRIP_clicked()
{
    if (ntripThread)
    {
        ntripThread->requestTerminate();
        ntripThread->wait(5000);

        disconnect(ntripThread, &NTRIPThread::infoMessage,
                         this, &MainWindow::ntripThread_Base_InfoMessage);

        disconnect(ntripThread, &NTRIPThread::warningMessage,
                         this, &MainWindow::ntripThread_Base_WarningMessage);

        disconnect(ntripThread, &NTRIPThread::errorMessage,
                         this, &MainWindow::ntripThread_Base_ErrorMessage);

        disconnect(ntripThread, &NTRIPThread::dataReceived,
                         this, &MainWindow::ntripThread_Base_DataReceived);

        disconnect(ntripThread, &NTRIPThread::threadEnded,
                         this, &MainWindow::ntripThread_Base_ThreadEnded);

//        disconnect(ntripThread, SIGNAL(serialTimeout(void)),
//                         this, SLOT(commThread_Base_SerialTimeout(void)));

        messageMonitorForm_Base_NTRIP->disconnectNTRIPThreadSlots(ntripThread);
        essentialsForm->disconnectNTRIPThreadSlots_Base(ntripThread);

        delete ntripThread;
        ntripThread = nullptr;

        ui->lineEdit_Command_Base_NTRIP->setEnabled(true);
        ui->pushButton_StartThread_Base_Serial->setEnabled(true);
        ui->pushButton_TerminateThread_Base_Serial->setEnabled(false);
        ui->pushButton_StartThread_Base_NTRIP->setEnabled(true);
        ui->pushButton_TerminateThread_NTRIP->setEnabled(false);
    }

}

void MainWindow::on_pushButton_StartThread_LaserDist_clicked()
{
    if (!serialThread_LaserDist)
    {
        serialThread_LaserDist = new LaserRangeFinder20HzV2SerialThread(ui->lineEdit_SerialPort_LaserDist->text(), ui->doubleSpinBox_DistanceOffset_LaserDist->value(), LaserRangeFinder20HzV2SerialThread::RESOLUTION_01mm);
        if (ui->checkBox_SuspendThread_LaserDist->isChecked())
        {
            serialThread_LaserDist->suspend();
        }

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::infoMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_InfoMessage);

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::warningMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_WarningMessage);

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::errorMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_ErrorMessage);

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::distanceReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_DistanceReceived);

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::errorReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_ErrorReceived);

        connect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::unidentifiedDataReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_UnidentifiedDataReceived);

        messageMonitorForm_LaserDist->connectSerialThreadSlots(serialThread_LaserDist);
        essentialsForm->connectLaserRangeFinder20HzV2SerialThreadSlots(serialThread_LaserDist);

        serialThread_LaserDist->start();

        ui->lineEdit_SerialPort_LaserDist->setEnabled(false);
        ui->doubleSpinBox_DistanceOffset_LaserDist->setEnabled(false);
        ui->pushButton_StartThread_LaserDist->setEnabled(false);
        ui->pushButton_TerminateThread_LaserDist->setEnabled(true);

/*        messageCounter_RELPOSNED_RoverB = 0;
        ui->label_RELPOSNEDMessageCount_RoverB->setText(QString::number(messageCounter_RELPOSNED_RoverB));
        ui->label_LastInfoMessage_RoverB->setText("");
        ui->label_LastWarningMessage_RoverB->setText("");
        ui->label_LastErrorMessage_RoverB->setText("");
        */
    }

}

void MainWindow::commThread_LaserRangeFinder20HzV2_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_LaserDist->setText(infoMessage);
}

void MainWindow::commThread_LaserRangeFinder20HzV2_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_LaserDist->setText(errorMessage);
}

void MainWindow::commThread_LaserRangeFinder20HzV2_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_LaserDist->setText(warningMessage);
}

void MainWindow::commThread_LaserRangeFinder20HzV2_DistanceReceived(const double& distance, qint64, qint64)
{
    ui->label_Distance_LaserDist->setText(QString::number(distance, 'f', 4));

    // No need for this, signal is connected directly to essentialForm
#if 0
    EssentialsForm::DistanceItem distanceItem;

    distanceItem.type = EssentialsForm::DistanceItem::MEASURED;
    distanceItem.distance = distance;
    distanceItem.frameStartTime = frameStartTime;
    distanceItem.frameEndTime = frameEndTime;

    emit distanceChanged(distanceItem);
#endif

}

void MainWindow::commThread_LaserRangeFinder20HzV2_ErrorReceived(const QString& errorString, qint64, qint64)
{
    ui->label_Distance_LaserDist->setText(errorString);
}

void MainWindow::commThread_LaserRangeFinder20HzV2_UnidentifiedDataReceived(const QByteArray& data, qint64, qint64)
{
    (void) data;
    ui->label_Distance_LaserDist->setText("Unidentified data");
}


void MainWindow::on_pushButton_ShowMessageWindow_LaserDist_clicked()
{
    messageMonitorForm_LaserDist->show();
    messageMonitorForm_LaserDist->raise();
    messageMonitorForm_LaserDist->activateWindow();
}

void MainWindow::on_pushButton_TerminateThread_LaserDist_clicked()
{
    if (serialThread_LaserDist)
    {
        serialThread_LaserDist->requestTerminate();
        serialThread_LaserDist->wait(5000);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::infoMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_InfoMessage);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::warningMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_WarningMessage);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::errorMessage,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_ErrorMessage);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::distanceReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_DistanceReceived);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::errorReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_ErrorReceived);

        disconnect(serialThread_LaserDist, &LaserRangeFinder20HzV2SerialThread::unidentifiedDataReceived,
                         this, &MainWindow::commThread_LaserRangeFinder20HzV2_UnidentifiedDataReceived);

        messageMonitorForm_LaserDist->disconnectSerialThreadSlots(serialThread_LaserDist);
        essentialsForm->disconnectLaserRangeFinder20HzV2SerialThreadSlots(serialThread_LaserDist);

        delete serialThread_LaserDist;
        serialThread_LaserDist = nullptr;

        ui->lineEdit_SerialPort_LaserDist->setEnabled(true);
        ui->doubleSpinBox_DistanceOffset_LaserDist->setEnabled(true);
        ui->pushButton_StartThread_LaserDist->setEnabled(true);
        ui->pushButton_TerminateThread_LaserDist->setEnabled(false);
    }
}

void MainWindow::on_doubleSpinBox_Distance_Constant_valueChanged(double distance)
{
    if (!serialThread_LaserDist)
    {
        EssentialsForm::DistanceItem distanceItem;

        distanceItem.type = EssentialsForm::DistanceItem::CONSTANT;
        distanceItem.distance = distance;

        QElapsedTimer elapsedTimer;
        elapsedTimer.start();

        distanceItem.frameStartTime = elapsedTimer.msecsSinceReference();
        distanceItem.frameEndTime = distanceItem.frameStartTime;

        emit distanceChanged(distanceItem);
    }
}

MainWinRover::MainWinRover(QWidget *parent, const int index, const ExtUIThings& uiThings)
{
    this->index = index;    
    this->extUIThings = uiThings;

    QString roverString = getRoverIdentString(index);

    messageMonitorForm = new MessageMonitorForm(parent, "Message monitor (Rover " + roverString + ")");
    messageMonitorForm->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor);
    relposnedForm = new RELPOSNEDForm(parent, "RELPOSNED (Rover " + roverString + ")");

    connect(&ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxMessageReceived,
                     this, &MainWinRover::ubloxProcessor_ubxMessageReceived);

    connect(extUIThings.pushButton_StartThread, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_StartThread_clicked);

    connect(extUIThings.pushButton_TerminateThread, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_TerminateThread_clicked);

    connect(extUIThings.pushButton_ShowMessageWindow, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ShowMessageWindow_clicked);

    connect(extUIThings.pushButton_ShowRELPOSNEDWindow, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ShowRELPOSNEDWindow_clicked);

    connect(extUIThings.checkBox_SuspendThread, &QCheckBox::stateChanged,
                     this, &MainWinRover::on_checkBox_SuspendThread_stateChanged);


    connect(extUIThings.pushButton_ClearRELPOSNEDCounter, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ClearRELPOSNEDCounter_clicked);

    connect(extUIThings.pushButton_ClearErrorMessage, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ClearErrorMessage_clicked);

    connect(extUIThings.pushButton_ClearWarningMessage, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ClearWarningMessage_clicked);

    connect(extUIThings.pushButton_ClearInfoMessage, &QAbstractButton::clicked,
                     this, &MainWinRover::on_pushButton_ClearInfoMessage_clicked);

    QSettings settings;
    extUIThings.lineEdit_SerialPort->setText(settings.value("SerialPort_Rover" + roverString, "\\\\.\\COM").toString());
    extUIThings.spinBox_SerialSpeed->setValue(settings.value("SerialSpeed_Rover" + roverString, "115200").toInt());
}

MainWinRover::~MainWinRover()
{
    QString roverString = getRoverIdentString(index);

    QSettings settings;
    settings.setValue("SerialPort_Rover" + roverString, extUIThings.lineEdit_SerialPort->text());
    settings.setValue("SerialSpeed_Rover" + roverString, extUIThings.spinBox_SerialSpeed->value());

    delete messageMonitorForm;
    delete relposnedForm;
}

void MainWinRover::startSerialThread(void)
{
    if (!serialThread)
    {
        serialThread = new SerialThread(extUIThings.lineEdit_SerialPort->text(), 20, 1, extUIThings.spinBox_SerialSpeed->value());
        if (extUIThings.checkBox_SuspendThread->isChecked())
        {
            serialThread->suspend();
        }

        connect(serialThread, &SerialThread::infoMessage,
                         this, &MainWinRover::commThread_InfoMessage);

        connect(serialThread, &SerialThread::warningMessage,
                         this, &MainWinRover::commThread_WarningMessage);

        connect(serialThread, &SerialThread::errorMessage,
                         this, &MainWinRover::commThread_ErrorMessage);

        connect(serialThread, &SerialThread::dataReceived,
                         this, &MainWinRover::commThread_DataReceived);

        connect(serialThread, &SerialThread::serialTimeout,
                         this, &MainWinRover::commThread_SerialTimeout);

        messageMonitorForm->connectSerialThreadSlots(serialThread);

        extUIThings.lineEdit_SerialPort->setEnabled(false);
        extUIThings.spinBox_SerialSpeed->setEnabled(false);
        extUIThings.pushButton_StartThread->setEnabled(false);
        extUIThings.pushButton_TerminateThread->setEnabled(true);

        messageCounter_RELPOSNED = 0;
        extUIThings.label_RELPOSNEDMessageCount->setText(QString::number(messageCounter_RELPOSNED));
        extUIThings.label_LastInfoMessage->setText("");
        extUIThings.label_LastWarningMessage->setText("");
        extUIThings.label_LastErrorMessage->setText("");

        serialThread->start();
    }
}

void MainWinRover::terminateSerialThread(void)
{
    if (serialThread)
    {
        serialThread->requestTerminate();
        serialThread->wait(5000);

        disconnect(serialThread, &SerialThread::infoMessage,
                         this, &MainWinRover::commThread_InfoMessage);

        disconnect(serialThread, &SerialThread::warningMessage,
                         this, &MainWinRover::commThread_WarningMessage);

        disconnect(serialThread, &SerialThread::errorMessage,
                         this, &MainWinRover::commThread_ErrorMessage);

        disconnect(serialThread, &SerialThread::dataReceived,
                         this, &MainWinRover::commThread_DataReceived);

        disconnect(serialThread, &SerialThread::serialTimeout,
                         this, &MainWinRover::commThread_SerialTimeout);

        messageMonitorForm->disconnectSerialThreadSlots(serialThread);

        delete serialThread;
        serialThread = nullptr;

        extUIThings.lineEdit_SerialPort->setEnabled(true);
        extUIThings.spinBox_SerialSpeed->setEnabled(true);
        extUIThings.pushButton_StartThread->setEnabled(true);
        extUIThings.pushButton_TerminateThread->setEnabled(false);
    }
}

void MainWinRover::commThread_InfoMessage(const QString& infoMessage)
{
    extUIThings.label_LastInfoMessage->setText(infoMessage);
}

void MainWinRover::commThread_ErrorMessage(const QString& errorMessage)
{
    extUIThings.label_LastErrorMessage->setText(errorMessage);
}

void MainWinRover::commThread_WarningMessage(const QString& warningMessage)
{
    extUIThings.label_LastWarningMessage->setText(warningMessage);
}

void MainWinRover::commThread_DataReceived(const QByteArray& data, const qint64 firstCharTime, const qint64 lastCharTime, const SerialThread::DataReceivedEmitReason&)
{
    ubloxDataStreamProcessor.process(data, firstCharTime, lastCharTime);
}

void MainWinRover::commThread_SerialTimeout(void)
{
    if (ubloxDataStreamProcessor.getNumOfUnprocessedBytes() != 0)
    {
        extUIThings.label_LastWarningMessage->setText(QString("Warning: discarded ") + QString::number(ubloxDataStreamProcessor.getNumOfUnprocessedBytes()) + " unprocessed bytes due to serial timeout.");
    }

    ubloxDataStreamProcessor.flushInputBuffer();
}


void MainWinRover::ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        messageCounter_RELPOSNED++;
        extUIThings.label_RELPOSNEDMessageCount->setText(QString::number(messageCounter_RELPOSNED));

        relposnedForm->updateFields(relposned);
    }
}

QString MainWinRover::getRoverIdentString(const unsigned int roverId)
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

void MainWinRover::on_pushButton_StartThread_clicked(bool)
{
    startSerialThread();
    extUIThings.essentialsForm->connectSerialThreadSlots_Rover(serialThread, index);
}

void MainWinRover::on_pushButton_TerminateThread_clicked(bool)
{
    terminateSerialThread();
    extUIThings.essentialsForm->disconnectSerialThreadSlots_Rover(serialThread, index);
}

void MainWinRover::on_pushButton_ShowMessageWindow_clicked(bool)
{
    messageMonitorForm->show();
    messageMonitorForm->raise();
    messageMonitorForm->activateWindow();
}

void MainWinRover::on_pushButton_ShowRELPOSNEDWindow_clicked(bool)
{
    relposnedForm->show();
    relposnedForm->raise();
    relposnedForm->activateWindow();
}

void MainWinRover::on_checkBox_SuspendThread_stateChanged(int arg1)
{
    if (serialThread)
    {
        if (arg1 == Qt::Checked)
        {
            serialThread->suspend();
        }
        else
        {
            serialThread->resume();
        }
    }
}

void MainWinRover::on_pushButton_ClearRELPOSNEDCounter_clicked(bool)
{
    messageCounter_RELPOSNED = 0;
    extUIThings.label_RELPOSNEDMessageCount->setText(QString::number(messageCounter_RELPOSNED));
}

void MainWinRover::on_pushButton_ClearErrorMessage_clicked(bool)
{
    extUIThings.label_LastErrorMessage->setText("");
}

void MainWinRover::on_pushButton_ClearWarningMessage_clicked(bool)
{
    extUIThings.label_LastWarningMessage->setText("");
}

void MainWinRover::on_pushButton_ClearInfoMessage_clicked(bool)
{
    extUIThings.label_LastInfoMessage->setText("");
}

void MainWindow::on_pushButton_StartThread_RPLidar_clicked()
{
    if (!thread_RPLidar)
    {
        thread_RPLidar = new RPLidarThread(ui->lineEdit_SerialPort_RPLidar->text(), ui->spinBox_SerialSpeed_RPLidar->value(), ui->spinBox_MotorPWM_RPLidar->value(), ui->comboBox_ExpressScanMode->currentIndex() - 1);
        if (ui->checkBox_SuspendThread_RPLidar->isChecked())
        {
            thread_RPLidar->suspend();
        }

        connect(thread_RPLidar, &RPLidarThread::infoMessage,
                         this, &MainWindow::thread_RPLidar_InfoMessage);

        connect(thread_RPLidar, &RPLidarThread::warningMessage,
                         this, &MainWindow::thread_RPLidar_WarningMessage);

        connect(thread_RPLidar, &RPLidarThread::errorMessage,
                         this, &MainWindow::thread_RPLidar_ErrorMessage);

        connect(thread_RPLidar, &RPLidarThread::distanceRoundReceived,
                         this, &MainWindow::thread_RPLidar_DistanceRoundReceived);

        messageMonitorForm_RPLidar->connectRPLidarThreadSlots(thread_RPLidar);
        lidarChartForm->connectRPLidarThreadSlots(thread_RPLidar);
        essentialsForm->connectRPLidarThreadSlots(thread_RPLidar);

        thread_RPLidar->start();

        ui->lineEdit_SerialPort_RPLidar->setEnabled(false);
        ui->spinBox_SerialSpeed_RPLidar->setEnabled(false);
        ui->spinBox_MotorPWM_RPLidar->setEnabled(false);
        ui->comboBox_ExpressScanMode->setEnabled(false);
        ui->pushButton_StartThread_RPLidar->setEnabled(false);
        ui->pushButton_TerminateThread_RPLidar->setEnabled(true);
    }


}

void MainWindow::on_pushButton_TerminateThread_RPLidar_clicked()
{
    if (thread_RPLidar)
    {
        thread_RPLidar->requestTerminate();

        thread_RPLidar->wait(5000);

        disconnect(thread_RPLidar, &RPLidarThread::infoMessage,
                         this, &MainWindow::thread_RPLidar_InfoMessage);

        disconnect(thread_RPLidar, &RPLidarThread::warningMessage,
                         this, &MainWindow::thread_RPLidar_WarningMessage);

        disconnect(thread_RPLidar, &RPLidarThread::errorMessage,
                         this, &MainWindow::thread_RPLidar_ErrorMessage);

        disconnect(thread_RPLidar, &RPLidarThread::distanceRoundReceived,
                         this, &MainWindow::thread_RPLidar_DistanceRoundReceived);

        messageMonitorForm_RPLidar->disconnectRPLidarThreadSlots(thread_RPLidar);
        lidarChartForm->disconnectRPLidarThreadSlots(thread_RPLidar);
        essentialsForm->disconnectRPLidarThreadSlots(thread_RPLidar);

        delete thread_RPLidar;
        thread_RPLidar = nullptr;

        ui->lineEdit_SerialPort_RPLidar->setEnabled(true);
        ui->spinBox_SerialSpeed_RPLidar->setEnabled(true);
        ui->spinBox_MotorPWM_RPLidar->setEnabled(true);
        ui->comboBox_ExpressScanMode->setEnabled(true);
        ui->pushButton_StartThread_RPLidar->setEnabled(true);
        ui->pushButton_TerminateThread_RPLidar->setEnabled(false);
    }
}


void MainWindow::thread_RPLidar_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_RPLidar->setText(infoMessage);
}

void MainWindow::thread_RPLidar_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_RPLidar->setText(errorMessage);
}

void MainWindow::thread_RPLidar_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_RPLidar->setText(warningMessage);
}

void MainWindow::thread_RPLidar_DistanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& distanceItems, qint64 startUptime, qint64 endUptime)
{
    (void) distanceItems;
    (void) startUptime;
    (void) endUptime;

    messageCounter_RPLidar_Rounds++;
    ui->label_RoundCount_RPLidar->setText(QString::number(messageCounter_RPLidar_Rounds));
}

void MainWindow::replay_RPLidar_DistanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& distanceItems, qint64 startUptime, qint64 endUptime)
{
    thread_RPLidar_DistanceRoundReceived(distanceItems, startUptime, endUptime);
}

void MainWindow::on_pushButton_ShowMessageWindow_RPLidar_clicked()
{
    messageMonitorForm_RPLidar->show();
    messageMonitorForm_RPLidar->raise();
    messageMonitorForm_RPLidar->activateWindow();
}

void MainWindow::on_pushButton_ShowLidarChartWindow_RPLidar_clicked()
{
    lidarChartForm->show();
    lidarChartForm->raise();
    lidarChartForm->activateWindow();
}


void MainWindow::on_pushButton_ClearRoundCounter_RPLidar_clicked()
{
    messageCounter_RPLidar_Rounds = 0;
    ui->label_RoundCount_RPLidar->setText(QString::number(messageCounter_RPLidar_Rounds));
}

void MainWindow::on_pushButton_ClearErrorMessage_RPLidar_clicked()
{
    ui->label_LastErrorMessage_RPLidar->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_RPLidar_clicked()
{
    ui->label_LastWarningMessage_RPLidar->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_RPLidar_clicked()
{
    ui->label_LastInfoMessage_RPLidar->setText("");
}

void MainWindow::on_checkBox_SuspendThread_RPLidar_stateChanged(int arg1)
{
    if (thread_RPLidar)
    {
        if (arg1 == Qt::Checked)
        {
            thread_RPLidar->suspend();
        }
        else
        {
            thread_RPLidar->resume();
        }
    }
}

void MainWindow::on_actionExit_triggered()
{
    close();
}



void MainWindow::on_actionLicenses_triggered()
{
    licencesForm->show();
    licencesForm->raise();
    licencesForm->activateWindow();
}
