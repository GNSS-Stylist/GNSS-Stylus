/*
    mainwindow.cpp (part of GNSS-Stylus)
    Copyright (C) 2019 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    // These initialization are for QSettings to work without exclicitly creating every instance with names (see documentation of QSettings).
    QCoreApplication::setOrganizationName("GNSSStylusOrganization");
//    QCoreApplication::setOrganizationDomain("ImaginaryUltraCapitalisticPlanetDestroyingMoneyAndPowerLeechingParasiteCompany.com");
    QCoreApplication::setApplicationName("GNSSStylus");

    serialThread_Base = nullptr;
    serialThread_RoverA = nullptr;
    serialThread_RoverB = nullptr;

    ui->setupUi(this);

    messageMonitorForm_Base = new MessageMonitorForm(parent, "Message monitor (Base)");
    messageMonitorForm_Base->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor_Base);

    messageMonitorForm_RoverA = new MessageMonitorForm(parent, "Message monitor (Rover A)");
    messageMonitorForm_RoverA->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor_RoverA);
    relposnedForm_RoverA = new RELPOSNEDForm(parent, "RELPOSNED (Rover A)");

    messageMonitorForm_RoverB = new MessageMonitorForm(parent, "Message monitor (Rover B)");
    messageMonitorForm_RoverB->connectUBloxDataStreamProcessorSlots(&ubloxDataStreamProcessor_RoverB);
    relposnedForm_RoverB = new RELPOSNEDForm(parent, "RELPOSNED (Rover B)");

    essentialsForm = new EssentialsForm(parent);
    essentialsForm->connectUBloxDataStreamProcessorSlots_Base(&ubloxDataStreamProcessor_Base);
    essentialsForm->connectUBloxDataStreamProcessorSlots_RoverA(&ubloxDataStreamProcessor_RoverA);
    essentialsForm->connectUBloxDataStreamProcessorSlots_RoverB(&ubloxDataStreamProcessor_RoverB);

    postProcessingForm = new PostProcessingForm(parent);

    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverA(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_RoverA_ubxMessageReceived(const UBXMessage&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverB(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_RoverB_ubxMessageReceived(const UBXMessage&)));

    essentialsForm->connectPostProcessingSlots(postProcessingForm);

    QObject::connect(&ubloxDataStreamProcessor_Base, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(ubloxProcessor_Base_rtcmMessageReceived(const RTCMMessage&)));

    QObject::connect(&ubloxDataStreamProcessor_RoverA, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_RoverA_ubxMessageReceived(const UBXMessage&)));

    QObject::connect(&ubloxDataStreamProcessor_RoverB, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_RoverB_ubxMessageReceived(const UBXMessage&)));

    QSettings settings;

    ui->lineEdit_ComPort_Base->setText(settings.value("ComPort_Base", "\\\\.\\COM").toString());
    ui->lineEdit_ComPort_RoverA->setText(settings.value("ComPort_RoverA", "\\\\.\\COM").toString());
    ui->lineEdit_ComPort_RoverB->setText(settings.value("ComPort_RoverB", "\\\\.\\COM").toString());

}

MainWindow::~MainWindow()
{
    QSettings settings;
    settings.setValue("ComPort_Base", ui->lineEdit_ComPort_Base->text());
    settings.setValue("ComPort_RoverA", ui->lineEdit_ComPort_RoverA->text());
    settings.setValue("ComPort_RoverB", ui->lineEdit_ComPort_RoverB->text());

    delete messageMonitorForm_Base;
    delete messageMonitorForm_RoverA;
    delete relposnedForm_RoverA;
    delete messageMonitorForm_RoverB;
    delete relposnedForm_RoverB;
    delete essentialsForm;
    delete postProcessingForm;

    delete ui;
}

void MainWindow::closeEvent (QCloseEvent *event)
{
    messageMonitorForm_Base->close();
    messageMonitorForm_RoverA->close();
    relposnedForm_RoverA->close();

    messageMonitorForm_RoverB->close();
    relposnedForm_RoverB->close();

    essentialsForm->close();
    postProcessingForm->close();

    event->accept();
}

void MainWindow::commThread_Base_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_Base->setText(infoMessage);
}

void MainWindow::commThread_Base_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_Base->setText(errorMessage);
}

void MainWindow::commThread_Base_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_Base->setText(warningMessage);
}

void MainWindow::commThread_Base_DataReceived(const QByteArray& data)
{
    ubloxDataStreamProcessor_Base.process(data);
}

void MainWindow::commThread_Base_SerialTimeout(void)
{
    if (ubloxDataStreamProcessor_Base.getNumOfUnprocessedBytes() != 0)
    {
        ui->label_LastWarningMessage_Base->setText(QString("Warning: discarded ") + QString::number(ubloxDataStreamProcessor_Base.getNumOfUnprocessedBytes()) + " unprocessed bytes due to serial timeout.");
    }

    ubloxDataStreamProcessor_Base.flushInputBuffer();
}

void MainWindow::ubloxProcessor_Base_rtcmMessageReceived(const RTCMMessage& rtcmMessage)
{
    messageCounter_RTCM_Base++;
    ui->label_RTCMMessageCount_Base->setText(QString::number(messageCounter_RTCM_Base));

    if (serialThread_RoverA)
    {
        serialThread_RoverA->addToSendQueue(rtcmMessage.rawMessage);
    }

    if (serialThread_RoverB)
    {
        serialThread_RoverB->addToSendQueue(rtcmMessage.rawMessage);
    }
}


void MainWindow::on_pushButton_StartThread_Base_clicked()
{
    if (!serialThread_Base)
    {
        serialThread_Base = new SerialThread(ui->lineEdit_ComPort_Base->text());
        if (ui->checkBox_SuspendThread_Base->isChecked())
        {
            serialThread_Base->suspend();
        }

        QObject::connect(serialThread_Base, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_Base_InfoMessage(const QString&)));

        QObject::connect(serialThread_Base, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_Base_WarningMessage(const QString&)));

        QObject::connect(serialThread_Base, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_Base_ErrorMessage(const QString&)));

        QObject::connect(serialThread_Base, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_Base_DataReceived(const QByteArray&)));

        QObject::connect(serialThread_Base, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_Base_SerialTimeout(void)));

        messageMonitorForm_Base->connectSerialThreadSlots(serialThread_Base);
        essentialsForm->connectSerialThreadSlots_Base(serialThread_Base);

        serialThread_Base->start();

        ui->lineEdit_ComPort_Base->setEnabled(false);
        ui->pushButton_StartThread_Base->setEnabled(false);
        ui->pushButton_TerminateThread_Base->setEnabled(true);

        messageCounter_RTCM_Base = 0;
        ui->label_RTCMMessageCount_Base->setText(QString::number(messageCounter_RTCM_Base));
        ui->label_LastInfoMessage_Base->setText("");
        ui->label_LastWarningMessage_Base->setText("");
        ui->label_LastErrorMessage_Base->setText("");
    }
}

void MainWindow::on_pushButton_TerminateThread_Base_clicked()
{
    if (serialThread_Base)
    {
        serialThread_Base->requestTerminate();
        serialThread_Base->wait(5000);

        QObject::disconnect(serialThread_Base, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_Base_InfoMessage(const QString&)));

        QObject::disconnect(serialThread_Base, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_Base_WarningMessage(const QString&)));

        QObject::disconnect(serialThread_Base, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_Base_ErrorMessage(const QString&)));

        QObject::disconnect(serialThread_Base, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_Base_DataReceived(const QByteArray&)));

        QObject::disconnect(serialThread_Base, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_Base_SerialTimeout(void)));

        messageMonitorForm_Base->disconnectSerialThreadSlots(serialThread_Base);
        essentialsForm->disconnectSerialThreadSlots_Base(serialThread_Base);

        delete serialThread_Base;
        serialThread_Base = nullptr;

        ui->lineEdit_ComPort_Base->setEnabled(true);
        ui->pushButton_StartThread_Base->setEnabled(true);
        ui->pushButton_TerminateThread_Base->setEnabled(false);
    }
}

void MainWindow::on_checkBox_SuspendThread_Base_stateChanged(int arg1)
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

void MainWindow::on_pushButton_ShowMessageWindow_Base_clicked()
{
    messageMonitorForm_Base->show();
    messageMonitorForm_Base->raise();
    messageMonitorForm_Base->activateWindow();
}

void MainWindow::on_pushButton_ClearRTCMCounter_Base_clicked()
{
    messageCounter_RTCM_Base = 0;
    ui->label_RTCMMessageCount_Base->setText(QString::number(messageCounter_RTCM_Base));
}

void MainWindow::on_pushButton_ClearErrorMessage_Base_clicked()
{
    ui->label_LastErrorMessage_Base->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_Base_clicked()
{
    ui->label_LastWarningMessage_Base->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_Base_clicked()
{
    ui->label_LastInfoMessage_Base->setText("");
}

void MainWindow::on_pushButton_StartThread_RoverA_clicked()
{
    if (!serialThread_RoverA)
    {
        serialThread_RoverA = new SerialThread(ui->lineEdit_ComPort_RoverA->text());
        if (ui->checkBox_SuspendThread_RoverA->isChecked())
        {
            serialThread_RoverA->suspend();
        }

        QObject::connect(serialThread_RoverA, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_RoverA_InfoMessage(const QString&)));

        QObject::connect(serialThread_RoverA, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_RoverA_WarningMessage(const QString&)));

        QObject::connect(serialThread_RoverA, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_RoverA_ErrorMessage(const QString&)));

        QObject::connect(serialThread_RoverA, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_RoverA_DataReceived(const QByteArray&)));

        QObject::connect(serialThread_RoverA, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_RoverA_SerialTimeout(void)));

        messageMonitorForm_RoverA->connectSerialThreadSlots(serialThread_RoverA);
        essentialsForm->connectSerialThreadSlots_RoverA(serialThread_RoverA);

        serialThread_RoverA->start();

        ui->lineEdit_ComPort_RoverA->setEnabled(false);
        ui->pushButton_StartThread_RoverA->setEnabled(false);
        ui->pushButton_TerminateThread_RoverA->setEnabled(true);

        messageCounter_RELPOSNED_RoverA = 0;
        ui->label_RELPOSNEDMessageCount_RoverA->setText(QString::number(messageCounter_RELPOSNED_RoverA));
        ui->label_LastInfoMessage_RoverA->setText("");
        ui->label_LastWarningMessage_RoverA->setText("");
        ui->label_LastErrorMessage_RoverA->setText("");
    }

}

void MainWindow::on_pushButton_TerminateThread_RoverA_clicked()
{
    if (serialThread_RoverA)
    {
        serialThread_RoverA->requestTerminate();
        serialThread_RoverA->wait(5000);

        QObject::disconnect(serialThread_RoverA, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_RoverA_InfoMessage(const QString&)));

        QObject::disconnect(serialThread_RoverA, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_RoverA_WarningMessage(const QString&)));

        QObject::disconnect(serialThread_RoverA, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_RoverA_ErrorMessage(const QString&)));

        QObject::disconnect(serialThread_RoverA, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_RoverA_DataReceived(const QByteArray&)));

        QObject::disconnect(serialThread_RoverA, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_RoverA_SerialTimeout(void)));

        messageMonitorForm_RoverA->disconnectSerialThreadSlots(serialThread_RoverA);
        essentialsForm->disconnectSerialThreadSlots_RoverA(serialThread_RoverA);

        delete serialThread_RoverA;
        serialThread_RoverA = nullptr;

        ui->lineEdit_ComPort_RoverA->setEnabled(true);
        ui->pushButton_StartThread_RoverA->setEnabled(true);
        ui->pushButton_TerminateThread_RoverA->setEnabled(false);
    }

}

void MainWindow::on_pushButton_ShowMessageWindow_RoverA_clicked()
{
    messageMonitorForm_RoverA->show();
    messageMonitorForm_RoverA->raise();
    messageMonitorForm_RoverA->activateWindow();
}

void MainWindow::on_checkBox_SuspendThread_RoverA_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked)
    {
        serialThread_RoverA->suspend();
    }
    else
    {
        serialThread_RoverA->resume();
    }
}



void MainWindow::on_pushButton_ClearRTCMCounter_RoverA_clicked()
{
    messageCounter_RELPOSNED_RoverA = 0;
    ui->label_RELPOSNEDMessageCount_RoverA->setText(QString::number(messageCounter_RELPOSNED_RoverA));
}

void MainWindow::on_pushButton_ClearErrorMessage_RoverA_clicked()
{
    ui->label_LastErrorMessage_RoverA->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_RoverA_clicked()
{
    ui->label_LastWarningMessage_RoverA->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_RoverA_clicked()
{
    ui->label_LastInfoMessage_RoverA->setText("");
}

void MainWindow::on_pushButton_ShowRELPOSNEDForm_RoverA_clicked()
{
    relposnedForm_RoverA->show();
    relposnedForm_RoverA->raise();
    relposnedForm_RoverA->activateWindow();
}


void MainWindow::commThread_RoverA_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_RoverA->setText(infoMessage);
}

void MainWindow::commThread_RoverA_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_RoverA->setText(errorMessage);
}

void MainWindow::commThread_RoverA_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_RoverA->setText(warningMessage);
}

void MainWindow::commThread_RoverA_DataReceived(const QByteArray& data)
{
    ubloxDataStreamProcessor_RoverA.process(data);
}

void MainWindow::commThread_RoverA_SerialTimeout(void)
{
    if (ubloxDataStreamProcessor_RoverA.getNumOfUnprocessedBytes() != 0)
    {
        ui->label_LastWarningMessage_RoverA->setText(QString("Warning: discarded ") + QString::number(ubloxDataStreamProcessor_RoverA.getNumOfUnprocessedBytes()) + " unprocessed bytes due to serial timeout.");
    }

    ubloxDataStreamProcessor_RoverA.flushInputBuffer();
}


void MainWindow::ubloxProcessor_RoverA_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        messageCounter_RELPOSNED_RoverA++;
        ui->label_RELPOSNEDMessageCount_RoverA->setText(QString::number(messageCounter_RELPOSNED_RoverA));

        relposnedForm_RoverA->updateFields(relposned);
    }
}


void MainWindow::on_pushButton_StartThread_RoverB_clicked()
{
    if (!serialThread_RoverB)
    {
        serialThread_RoverB = new SerialThread(ui->lineEdit_ComPort_RoverB->text());
        if (ui->checkBox_SuspendThread_RoverB->isChecked())
        {
            serialThread_RoverB->suspend();
        }

        QObject::connect(serialThread_RoverB, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_RoverB_InfoMessage(const QString&)));

        QObject::connect(serialThread_RoverB, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_RoverB_WarningMessage(const QString&)));

        QObject::connect(serialThread_RoverB, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_RoverB_ErrorMessage(const QString&)));

        QObject::connect(serialThread_RoverB, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_RoverB_DataReceived(const QByteArray&)));

        QObject::connect(serialThread_RoverB, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_RoverB_SerialTimeout(void)));

        messageMonitorForm_RoverB->connectSerialThreadSlots(serialThread_RoverB);
        essentialsForm->connectSerialThreadSlots_RoverB(serialThread_RoverB);

        serialThread_RoverB->start();

        ui->lineEdit_ComPort_RoverB->setEnabled(false);
        ui->pushButton_StartThread_RoverB->setEnabled(false);
        ui->pushButton_TerminateThread_RoverB->setEnabled(true);

        messageCounter_RELPOSNED_RoverB = 0;
        ui->label_RELPOSNEDMessageCount_RoverB->setText(QString::number(messageCounter_RELPOSNED_RoverB));
        ui->label_LastInfoMessage_RoverB->setText("");
        ui->label_LastWarningMessage_RoverB->setText("");
        ui->label_LastErrorMessage_RoverB->setText("");
    }

}

void MainWindow::on_pushButton_TerminateThread_RoverB_clicked()
{
    if (serialThread_RoverB)
    {
        serialThread_RoverB->requestTerminate();
        serialThread_RoverB->wait(5000);

        QObject::disconnect(serialThread_RoverB, SIGNAL(infoMessage(const QString&)),
                         this, SLOT(commThread_RoverB_InfoMessage(const QString&)));

        QObject::disconnect(serialThread_RoverB, SIGNAL(warningMessage(const QString&)),
                         this, SLOT(commThread_RoverB_WarningMessage(const QString&)));

        QObject::disconnect(serialThread_RoverB, SIGNAL(errorMessage(const QString&)),
                         this, SLOT(commThread_RoverB_ErrorMessage(const QString&)));

        QObject::disconnect(serialThread_RoverB, SIGNAL(dataReceived(const QByteArray&)),
                         this, SLOT(commThread_RoverB_DataReceived(const QByteArray&)));

        QObject::disconnect(serialThread_RoverB, SIGNAL(serialTimeout(void)),
                         this, SLOT(commThread_RoverB_SerialTimeout(void)));

        messageMonitorForm_RoverB->disconnectSerialThreadSlots(serialThread_RoverB);
        essentialsForm->disconnectSerialThreadSlots_RoverB(serialThread_RoverB);

        delete serialThread_RoverB;
        serialThread_RoverB = nullptr;

        ui->lineEdit_ComPort_RoverB->setEnabled(true);
        ui->pushButton_StartThread_RoverB->setEnabled(true);
        ui->pushButton_TerminateThread_RoverB->setEnabled(false);
    }
}

void MainWindow::on_pushButton_ShowMessageWindow_RoverB_clicked()
{
    messageMonitorForm_RoverB->show();
    messageMonitorForm_RoverB->raise();
    messageMonitorForm_RoverB->activateWindow();
}

void MainWindow::on_pushButton_ShowRELPOSNEDForm_RoverB_clicked()
{
    relposnedForm_RoverB->show();
    relposnedForm_RoverB->raise();
    relposnedForm_RoverB->activateWindow();
}

void MainWindow::on_checkBox_SuspendThread_RoverB_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked)
    {
        serialThread_RoverB->suspend();
    }
    else
    {
        serialThread_RoverB->resume();
    }
}

void MainWindow::on_pushButton_ClearRTCMCounter_RoverB_clicked()
{
    messageCounter_RELPOSNED_RoverB = 0;
    ui->label_RELPOSNEDMessageCount_RoverB->setText(QString::number(messageCounter_RELPOSNED_RoverB));
}

void MainWindow::on_pushButton_ClearErrorMessage_RoverB_clicked()
{
    ui->label_LastErrorMessage_RoverB->setText("");
}

void MainWindow::on_pushButton_ClearWarningMessage_RoverB_clicked()
{
    ui->label_LastWarningMessage_RoverB->setText("");
}

void MainWindow::on_pushButton_ClearInfoMessage_RoverB_clicked()
{
    ui->label_LastInfoMessage_RoverB->setText("");
}

void MainWindow::commThread_RoverB_InfoMessage(const QString& infoMessage)
{
    ui->label_LastInfoMessage_RoverB->setText(infoMessage);
}

void MainWindow::commThread_RoverB_ErrorMessage(const QString& errorMessage)
{
    ui->label_LastErrorMessage_RoverB->setText(errorMessage);
}

void MainWindow::commThread_RoverB_WarningMessage(const QString& warningMessage)
{
    ui->label_LastWarningMessage_RoverB->setText(warningMessage);
}

void MainWindow::commThread_RoverB_DataReceived(const QByteArray& data)
{
    ubloxDataStreamProcessor_RoverB.process(data);
}

void MainWindow::commThread_RoverB_SerialTimeout(void)
{
    if (ubloxDataStreamProcessor_RoverB.getNumOfUnprocessedBytes() != 0)
    {
        ui->label_LastWarningMessage_RoverB->setText(QString("Warning: discarded ") + QString::number(ubloxDataStreamProcessor_RoverB.getNumOfUnprocessedBytes()) + " unprocessed bytes due to serial timeout.");
    }

    ubloxDataStreamProcessor_RoverB.flushInputBuffer();
}


void MainWindow::ubloxProcessor_RoverB_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        messageCounter_RELPOSNED_RoverB++;
        ui->label_RELPOSNEDMessageCount_RoverB->setText(QString::number(messageCounter_RELPOSNED_RoverB));

        relposnedForm_RoverB->updateFields(relposned);
    }
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
