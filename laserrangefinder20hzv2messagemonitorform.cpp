/*
    laserrangefinder20hzv2messagemonitorform.cpp (part of GNSS-Stylus)
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
 * @file laserrangefinder20hzv2messagemonitorform.cpp
 * @brief Declaration for a form that shows some data about messages sent by "20hz high Accuracy 80m Laser Sensor Range finder Distance measuring module".
 */


#include <QTime>

#include "laserrangefinder20hzv2messagemonitorform.h"
#include "ui_laserrangefinder20hzv2messagemonitorform.h"

LaserRangeFinder20HzV2MessageMonitorForm::LaserRangeFinder20HzV2MessageMonitorForm(QWidget *parent, const QString& title) :
    QWidget(parent),
    ui(new Ui::LaserRangeFinder20HzV2MessageMonitorForm)
{
    ui->setupUi(this);
    this->setWindowTitle(title);
}

LaserRangeFinder20HzV2MessageMonitorForm::~LaserRangeFinder20HzV2MessageMonitorForm()
{
    delete ui;
}

void LaserRangeFinder20HzV2MessageMonitorForm::addLogLine(const QString& line)
{
    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Output->setMaximumBlockCount(ui->spinBox_MaxLines->value());
    ui->plainTextEdit_Output->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Output->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Output->appendPlainText(timeString + ": " + line);
}


void LaserRangeFinder20HzV2MessageMonitorForm::errorMessage(const QString& errorMessage)
{
    addLogLine(QString("Serial thread error: ") + errorMessage);
}

void LaserRangeFinder20HzV2MessageMonitorForm::warningMessage(const QString& warningMessage)
{
    addLogLine(QString("Serial thread warning: ") + warningMessage);
}

void LaserRangeFinder20HzV2MessageMonitorForm::infoMessage(const QString& infoMessage)
{
    addLogLine(QString("Serial thread info: ") + infoMessage);
}

void LaserRangeFinder20HzV2MessageMonitorForm::distanceReceived(const double& distance, qint64 startTime, qint64 endTime)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_Distance->checkState()))
    {
        addLogLine(QString("New distance received: ") + QString::number(distance, 'f', 4) + "m. " + getTimeDifferenceString(startTime, endTime));
    }
    updateStartAndEndTimes(startTime, endTime);
}

void LaserRangeFinder20HzV2MessageMonitorForm::errorReceived(const QString& errorString, qint64 startTime, qint64 endTime)
{
    if ((!ui->checkBox_SuspendOutput->checkState()) && (ui->checkBox_Errors->checkState()))
    {
        addLogLine(QString("Error received: \"") + errorString + "\". " + getTimeDifferenceString(startTime, endTime));
    }
    updateStartAndEndTimes(startTime, endTime);
}

void LaserRangeFinder20HzV2MessageMonitorForm::unidentifiedDataReceived(const QByteArray& data, qint64 startTime, qint64 endTime)
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
                    ", Data(hex): " + dataString + " (as string: " + data + ")." +
                    getTimeDifferenceString(startTime, endTime));
    }
    updateStartAndEndTimes(startTime, endTime);
}


void LaserRangeFinder20HzV2MessageMonitorForm::connectSerialThreadSlots(LaserRangeFinder20HzV2SerialThread* serThread)
{
    connect(serThread, &LaserRangeFinder20HzV2SerialThread::infoMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::infoMessage);

    connect(serThread, &LaserRangeFinder20HzV2SerialThread::warningMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::warningMessage);

    connect(serThread, &LaserRangeFinder20HzV2SerialThread::errorMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::errorMessage);

    connect(serThread, &LaserRangeFinder20HzV2SerialThread::distanceReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::distanceReceived);

    connect(serThread, &LaserRangeFinder20HzV2SerialThread::errorReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::errorReceived);

    connect(serThread, &LaserRangeFinder20HzV2SerialThread::unidentifiedDataReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::unidentifiedDataReceived);
}

void LaserRangeFinder20HzV2MessageMonitorForm::disconnectSerialThreadSlots(LaserRangeFinder20HzV2SerialThread* serThread)
{
    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::infoMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::infoMessage);

    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::warningMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::warningMessage);

    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::errorMessage,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::errorMessage);

    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::distanceReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::distanceReceived);

    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::errorReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::errorReceived);

    disconnect(serThread, &LaserRangeFinder20HzV2SerialThread::unidentifiedDataReceived,
                     this, &LaserRangeFinder20HzV2MessageMonitorForm::unidentifiedDataReceived);
}

void LaserRangeFinder20HzV2MessageMonitorForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Output->clear();
}

QString LaserRangeFinder20HzV2MessageMonitorForm::getTimeDifferenceString(const qint64 startTime, const qint64 endTime)
{
    qint64 startTimeDifference = startTime - lastStartTime;
    qint64 burstDuration = endTime - startTime;
    qint64 idleTime = startTime - lastEndTime;

    QString retval = "Start time difference: ";
    retval += QString::number(startTimeDifference);
    retval += ", burst duration: " + QString::number(burstDuration);
    retval += ", idle time: " + QString::number(idleTime);

    return retval;
}

void LaserRangeFinder20HzV2MessageMonitorForm::updateStartAndEndTimes(const qint64 startTime, const qint64 endTime)
{
    lastStartTime = startTime;
    lastEndTime = endTime;
}

