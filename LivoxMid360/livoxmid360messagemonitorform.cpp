/*
    livoxmid360messagemonitorform.cpp (part of GNSS-Stylus)
    Copyright (C) 2020-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file livoxmid360messagemonitorform.cpp
 * @brief Declaration for a form that shows some data about messages sent by Livox Mid-360.
 */


#include <QTime>

#include "livoxmid360messagemonitorform.h"
#include "ui_livoxmid360messagemonitorform.h"

//QUdpSocket* testSocket;

LivoxMid360MessageMonitorForm::LivoxMid360MessageMonitorForm(QWidget *parent, const QString& title) :
    QWidget(parent),
    ui(new Ui::LivoxMid360MessageMonitorForm)
{
    ui->setupUi(this);
    this->setWindowTitle(title);

/*    testSocket = new QUdpSocket;
    QHostAddress hostAddr;
    hostAddr.setAddress("192.168.40.10");
    testSocket->bind(hostAddr, 56201);

    connect(testSocket, &QUdpSocket::readyRead, this, &LivoxMid360MessageMonitorForm::handleDatagram);
*/
}
/*
void LivoxMid360MessageMonitorForm::handleDatagram()
{
    while (testSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = testSocket->receiveDatagram();
        this->addLogLine("Got datagram");
        //processTheDatagram(datagram);
    }

}
*/
LivoxMid360MessageMonitorForm::~LivoxMid360MessageMonitorForm()
{
    delete ui;
}

void LivoxMid360MessageMonitorForm::addLogLine(const QString& line)
{
    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Output->setMaximumBlockCount(ui->spinBox_MaxLines->value());
    ui->plainTextEdit_Output->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Output->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Output->appendPlainText(timeString + ": " + line);
}


void LivoxMid360MessageMonitorForm::errorMessage(const QString& errorMessage)
{
    addLogLine(QString("Serial thread error: ") + errorMessage);
}

void LivoxMid360MessageMonitorForm::warningMessage(const QString& warningMessage)
{
    addLogLine(QString("Serial thread warning: ") + warningMessage);
}

void LivoxMid360MessageMonitorForm::infoMessage(const QString& infoMessage)
{
    addLogLine(QString("Serial thread info: ") + infoMessage);
}


void LivoxMid360MessageMonitorForm::connectLivoxMid360ThreadSlots(LivoxMid360Thread* livoxMid360Thread)
{
    QObject::connect(livoxMid360Thread, &LivoxMid360Thread::infoMessage,
                     this, &LivoxMid360MessageMonitorForm::infoMessage);

    QObject::connect(livoxMid360Thread, &LivoxMid360Thread::warningMessage,
                     this, &LivoxMid360MessageMonitorForm::warningMessage);

    QObject::connect(livoxMid360Thread, &LivoxMid360Thread::errorMessage,
                     this, &LivoxMid360MessageMonitorForm::errorMessage);

//    QObject::connect(rpLidarThread, &LivoxMid360Thread::distanceRoundReceived,
//                     this, &LivoxMid360MessageMonitorForm::distanceRoundReceived);

}

void LivoxMid360MessageMonitorForm::disconnectLivoxMid360ThreadSlots(LivoxMid360Thread* livoxMid360Thread)
{
    QObject::disconnect(livoxMid360Thread, SIGNAL(infoMessage(const QString&)),
                     this, SLOT(infoMessage(const QString&)));

    QObject::disconnect(livoxMid360Thread, SIGNAL(warningMessage(const QString&)),
                     this, SLOT(warningMessage(const QString&)));

    QObject::disconnect(livoxMid360Thread, SIGNAL(errorMessage(const QString&)),
                     this, SLOT(errorMessage(const QString&)));

//    QObject::disconnect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
//                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));

}


void LivoxMid360MessageMonitorForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Output->clear();
}
/*
void LivoxMid360MessageMonitorForm::distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime)
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
*/
