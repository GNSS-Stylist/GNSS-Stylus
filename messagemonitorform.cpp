/*
    messagemonitorform.cpp (part of GNSS-Stylus)
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

void MessageMonitorForm::ubloxProcessor_nmeaSentenceReceived(const QByteArray& nmeaSentence)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_NMEA->checkState()))
    {
        addLogLine(QString("NMEA: ") + nmeaSentence.trimmed());
    }
}

void MessageMonitorForm::ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_UBX->checkState()))
    {
        QString messageTypeString = "Unhandled";

        UBXMessage_RELPOSNED relposned(ubxMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            messageTypeString = "RELPOSNED";
        }

        addLogLine(QString("UBX message received. Payload length: ") + QString::number(ubxMessage.payloadLength) +
                   ", class: " + QString::number(ubxMessage.messageClass) +
                   ", id: " + QString::number(ubxMessage.messageId) + " (" + messageTypeString + ").");
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

void MessageMonitorForm::DataReceived(const QByteArray&)
{

}

void MessageMonitorForm::serialTimeout(void)
{

}



void MessageMonitorForm::connectSerialThreadSlots(SerialThread* serThread)
{
    QObject::connect(serThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(InfoMessage(const QString&)));

    QObject::connect(serThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(WarningMessage(const QString&)));

    QObject::connect(serThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(ErrorMessage(const QString&)));

    QObject::connect(serThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(DataReceived(const QByteArray&)));

    QObject::connect(serThread, SIGNAL(serialTimeout(void)),
                     this, SLOT(serialTimeout(void)));
}

void MessageMonitorForm::disconnectSerialThreadSlots(SerialThread* serThread)
{
    QObject::disconnect(serThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(InfoMessage(const QString&)));

    QObject::disconnect(serThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(WarningMessage(const QString&)));

    QObject::disconnect(serThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(ErrorMessage(const QString&)));

    QObject::disconnect(serThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(DataReceived(const QByteArray&)));

    QObject::disconnect(serThread, SIGNAL(serialTimeout(void)),
                     this, SLOT(serialTimeout(void)));
}

void MessageMonitorForm::connectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(ubloxProcessor_nmeaSentenceReceived(const QByteArray&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_ubxMessageReceived(const UBXMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxParseError(const QString&)),
                     this, SLOT(ubloxProcessor_ubxParseError(const QString&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaParseError(const QString&)),
                     this, SLOT(ubloxProcessor_nmeaParseError(const QString&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(ubloxProcessor_rtcmMessageReceived(const RTCMMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(unidentifiedDataReceived(const QByteArray&)),
                     this, SLOT(ubloxProcessor_unidentifiedDataReceived(const QByteArray&)));
}

void MessageMonitorForm::disconnectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(ubloxProcessor_nmeaSentenceReceived(const QByteArray&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubloxProcessor_ubxMessageReceived(const UBXMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxParseError(const QString&)),
                     this, SLOT(ubloxProcessor_ubxParseError(const QString&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaParseError(const QString&)),
                     this, SLOT(ubloxProcessor_nmeaParseError(const QString&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(ubloxProcessor_rtcmMessageReceived(const RTCMMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(unidentifiedDataReceived(const QByteArray&)),
                     this, SLOT(ubloxProcessor_unidentifiedDataReceived(const QByteArray&)));
}

void MessageMonitorForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Output->clear();
}

void MessageMonitorForm::connectNTRIPThreadSlots(NTRIPThread* ntripThread)
{
    QObject::connect(ntripThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(InfoMessage(const QString&)));

    QObject::connect(ntripThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(WarningMessage(const QString&)));

    QObject::connect(ntripThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(ErrorMessage(const QString&)));

    QObject::connect(ntripThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(DataReceived(const QByteArray&)));

//    QObject::connect(serThread, SIGNAL(serialTimeout(void)),
//                     this, SLOT(commThread_SerialTimeout(void)));
}

void MessageMonitorForm::disconnectNTRIPThreadSlots(NTRIPThread* ntripThread)
{
    QObject::disconnect(ntripThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(InfoMessage(const QString&)));

    QObject::disconnect(ntripThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(WarningMessage(const QString&)));

    QObject::disconnect(ntripThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(ErrorMessage(const QString&)));

    QObject::disconnect(ntripThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(DataReceived(const QByteArray&)));

//    QObject::disconnect(serThread, SIGNAL(serialTimeout(void)),
//                     this, SLOT(commThread_SerialTimeout(void)));
}

