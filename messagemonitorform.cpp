/*
    messagemonitorform.cpp (part of GNSS-Stylus)
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
 * @file messagemonitorform.cpp
 * @brief Declaration for a form that shows some data about messages sent by u-blox-devices.
 */


#include <QTime>

#include "messagemonitorform.h"
#include "ui_messagemonitorform.h"

MessageMonitorForm::MessageMonitorForm(QWidget *parent, const QString& title) :
    QWidget(parent),
    ui(new Ui::MessageMonitorForm)
{
    ui->setupUi(this);
    this->setWindowTitle(title);
}

MessageMonitorForm::~MessageMonitorForm()
{
    delete ui;
}

void MessageMonitorForm::addLogLine(const QString& line)
{
    //ui->listWidget->selectionBehavior()

    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Output->setMaximumBlockCount(ui->spinBox_MaxLines->value());
    ui->plainTextEdit_Output->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Output->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Output->appendPlainText(timeString + ": " + line);
}

void MessageMonitorForm::ubloxProcessor_nmeaSentenceReceived(const NMEAMessage& nmeaSentence)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_NMEA->checkState()))
    {
        addLogLine(QString("NMEA: ") + nmeaSentence.rawMessage.trimmed());
    }
}

void MessageMonitorForm::ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    QString timeString = "";

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        qint64 startTimeDifference = ubxMessage.messageStartTime - lastRELPOSNEDMessageStartTime;
        qint64 burstDuration = ubxMessage.messageEndTime - ubxMessage.messageStartTime;
        qint64 idleTime = ubxMessage.messageStartTime - lastRELPOSNEDMessageEndTime;

        timeString = " Start time difference: ";
        timeString += QString::number(startTimeDifference);
        timeString += ", burst duration: " + QString::number(burstDuration);
        timeString += ", idle time: " + QString::number(idleTime);

        lastRELPOSNEDMessageStartTime = ubxMessage.messageStartTime;
        lastRELPOSNEDMessageEndTime = ubxMessage.messageEndTime;
    }

    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_UBX->checkState()))
    {
        QString messageTypeString = "Unhandled";

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            messageTypeString = "RELPOSNED";
        }

        addLogLine(QString("UBX message received. Payload length: ") + QString::number(ubxMessage.payloadLength) +
                   ", class: " + QString::number(ubxMessage.messageClass) +
                   ", id: " + QString::number(ubxMessage.messageId) + " (" + messageTypeString + ")." + timeString);
    }
}

void MessageMonitorForm::ubloxProcessor_rtcmMessageReceived(const RTCMMessage& rtcmMessage)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_RTCM->checkState()))
    {
        addLogLine(QString("RTCM: Message type: ") + QString::number(rtcmMessage.messageType) + ", length: " + QString::number(rtcmMessage.rawMessage.size()));
    }
}


void MessageMonitorForm::ubloxProcessor_ubxParseError(const QString& error)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_UBXParseErrors->checkState()))
    {
        addLogLine(QString("UBX parse error: ") + error);
    }
}

void MessageMonitorForm::ubloxProcessor_nmeaParseError(const QString& error)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_NMEAParseErrors->checkState()))
    {
        addLogLine(QString("NMEA parse error: ") + error);
    }
}

void MessageMonitorForm::ubloxProcessor_unidentifiedDataReceived(const QByteArray& data)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_UnidentifiedData->checkState()))
    {
        QString dataString;

        for (int i = 0; i < data.length(); i++)
        {
            if (i != 0)
            {
                dataString += ", ";
            }
            if (!(data[i] & 0xF0))
            {
                dataString += "0";
            }
            dataString += QString::number(static_cast<unsigned char>(data[i]), 16);
        }

        addLogLine(QString("Unidentified data received. Num of bytes: ") + QString::number(data.length()) +
                   ", Data(hex): " + dataString + " (as string: " + data + ").");
    }
}


void MessageMonitorForm::ErrorMessage(const QString& errorMessage)
{
    addLogLine(QString("Serial thread error: ") + errorMessage);
}

void MessageMonitorForm::WarningMessage(const QString& warningMessage)
{
    addLogLine(QString("Serial thread warning: ") + warningMessage);
}

void MessageMonitorForm::InfoMessage(const QString& infoMessage)
{
    addLogLine(QString("Serial thread info: ") + infoMessage);
}

void MessageMonitorForm::serialDataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)
{

}

void MessageMonitorForm::ntripDataReceived(const QByteArray&)
{

}

void MessageMonitorForm::serialTimeout(void)
{

}



void MessageMonitorForm::connectSerialThreadSlots(SerialThread* serThread)
{
    connect(serThread, &SerialThread::infoMessage,
                     this, &MessageMonitorForm::InfoMessage);

    connect(serThread, &SerialThread::warningMessage,
                     this, &MessageMonitorForm::WarningMessage);

    connect(serThread, &SerialThread::errorMessage,
                     this, &MessageMonitorForm::ErrorMessage);

    connect(serThread, &SerialThread::dataReceived,
                     this, &MessageMonitorForm::serialDataReceived);

    connect(serThread, &SerialThread::serialTimeout,
                     this, &MessageMonitorForm::serialTimeout);
}

void MessageMonitorForm::disconnectSerialThreadSlots(SerialThread* serThread)
{
    disconnect(serThread, &SerialThread::infoMessage,
                     this, &MessageMonitorForm::InfoMessage);

    disconnect(serThread, &SerialThread::warningMessage,
                     this, &MessageMonitorForm::WarningMessage);

    disconnect(serThread, &SerialThread::errorMessage,
                     this, &MessageMonitorForm::ErrorMessage);

    disconnect(serThread, &SerialThread::dataReceived,
                     this, &MessageMonitorForm::serialDataReceived);

    disconnect(serThread, &SerialThread::serialTimeout,
                     this, &MessageMonitorForm::serialTimeout);
}

void MessageMonitorForm::connectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::nmeaSentenceReceived,
                     this, &MessageMonitorForm::ubloxProcessor_nmeaSentenceReceived);

    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxMessageReceived,
                     this, &MessageMonitorForm::ubloxProcessor_ubxMessageReceived);

    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxParseError,
                     this, &MessageMonitorForm::ubloxProcessor_ubxParseError);

    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::nmeaParseError,
                     this, &MessageMonitorForm::ubloxProcessor_nmeaParseError);

    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::rtcmMessageReceived,
                     this, &MessageMonitorForm::ubloxProcessor_rtcmMessageReceived);

    connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::unidentifiedDataReceived,
                     this, &MessageMonitorForm::ubloxProcessor_unidentifiedDataReceived);
}

void MessageMonitorForm::disconnectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::nmeaSentenceReceived,
                     this, &MessageMonitorForm::ubloxProcessor_nmeaSentenceReceived);

    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxMessageReceived,
                     this, &MessageMonitorForm::ubloxProcessor_ubxMessageReceived);

    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxParseError,
                     this, &MessageMonitorForm::ubloxProcessor_ubxParseError);

    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::nmeaParseError,
                     this, &MessageMonitorForm::ubloxProcessor_nmeaParseError);

    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::rtcmMessageReceived,
                     this, &MessageMonitorForm::ubloxProcessor_rtcmMessageReceived);

    disconnect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::unidentifiedDataReceived,
                     this, &MessageMonitorForm::ubloxProcessor_unidentifiedDataReceived);
}

void MessageMonitorForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Output->clear();
}

void MessageMonitorForm::connectNTRIPThreadSlots(NTRIPThread* ntripThread)
{
    connect(ntripThread, &NTRIPThread::infoMessage,
                     this, &MessageMonitorForm::InfoMessage);

    connect(ntripThread, &NTRIPThread::warningMessage,
                     this, &MessageMonitorForm::WarningMessage);

    connect(ntripThread, &NTRIPThread::errorMessage,
                     this, &MessageMonitorForm::ErrorMessage);

    connect(ntripThread, &NTRIPThread::dataReceived,
                     this, &MessageMonitorForm::ntripDataReceived);

//    connect(serThread, SIGNAL(serialTimeout(void)),
//                     this, SLOT(commThread_SerialTimeout(void)));
}

void MessageMonitorForm::disconnectNTRIPThreadSlots(NTRIPThread* ntripThread)
{
    disconnect(ntripThread, &NTRIPThread::infoMessage,
                     this, &MessageMonitorForm::InfoMessage);

    disconnect(ntripThread, &NTRIPThread::warningMessage,
                     this, &MessageMonitorForm::WarningMessage);

    disconnect(ntripThread, &NTRIPThread::errorMessage,
                     this, &MessageMonitorForm::ErrorMessage);

    disconnect(ntripThread, &NTRIPThread::dataReceived,
                     this, &MessageMonitorForm::ntripDataReceived);

//    disconnect(serThread, SIGNAL(serialTimeout(void)),
//                     this, SLOT(commThread_SerialTimeout(void)));
}

