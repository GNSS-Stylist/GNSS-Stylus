/*
    rplidarmessagemonitorform.cpp (part of GNSS-Stylus)
    Copyright (C) 2012 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file rplidarmessagemonitorform.cpp
 * @brief Declaration for a form that shows some data about messages sent by RPLidar.
 */


#include <QTime>

#include "rplidarmessagemonitorform.h"
#include "ui_rplidarmessagemonitorform.h"

RPLidarMessageMonitorForm::RPLidarMessageMonitorForm(QWidget *parent, const QString& title) :
    QWidget(parent),
    ui(new Ui::RPLidarMessageMonitorForm)
{
    ui->setupUi(this);
    this->setWindowTitle(title);
}

RPLidarMessageMonitorForm::~RPLidarMessageMonitorForm()
{
    delete ui;
}

void RPLidarMessageMonitorForm::addLogLine(const QString& line)
{
    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Output->setMaximumBlockCount(ui->spinBox_MaxLines->value());
    ui->plainTextEdit_Output->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Output->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Output->appendPlainText(timeString + ": " + line);
}


void RPLidarMessageMonitorForm::errorMessage(const QString& errorMessage)
{
    addLogLine(QString("Serial thread error: ") + errorMessage);
}

void RPLidarMessageMonitorForm::warningMessage(const QString& warningMessage)
{
    addLogLine(QString("Serial thread warning: ") + warningMessage);
}

void RPLidarMessageMonitorForm::infoMessage(const QString& infoMessage)
{
    addLogLine(QString("Serial thread info: ") + infoMessage);
}


void RPLidarMessageMonitorForm::connectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::connect(rpLidarThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(infoMessage(const QString&)));

    QObject::connect(rpLidarThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(warningMessage(const QString&)));

    QObject::connect(rpLidarThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(errorMessage(const QString&)));

    QObject::connect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));

}

void RPLidarMessageMonitorForm::disconnectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::disconnect(rpLidarThread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(infoMessage(const QString&)));

    QObject::disconnect(rpLidarThread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(warningMessage(const QString&)));

    QObject::disconnect(rpLidarThread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(errorMessage(const QString&)));

    QObject::disconnect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));

}


void RPLidarMessageMonitorForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Output->clear();
}

void RPLidarMessageMonitorForm::distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime)
{
    if (ui->checkBox_Distance->checkState())
    {
        int time = endTime - startTime;
        double rpm = 0;
        int sampleRate = 0;

        if (time > 0)
        {
            rpm = 1000. / time;
            sampleRate = data.size() * 1000 / time;
        }

        addLogLine(QString("New round of data received. Items: ") + QString::number(data.size()) +
                   ", elapsed time: " + QString::number(time) + " ms" +
                   ", rpm: " + QString::number(rpm, 'f', 1) +
                   ", sample rate: " + QString::number(sampleRate));
    }
}
