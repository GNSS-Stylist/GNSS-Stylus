/*
    rplidarmessagemonitorform.h (part of GNSS-Stylus)
    Copyright (C) 2020-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file laserrangefinder20hzv2messagemonitorform.h
 * @brief Declaration for a form that shows some data about messages sent by RPLidar.
 */

#ifndef RPLIDARMESSAGEMONITORFORM_H
#define RPLIDARMESSAGEMONITORFORM_H

#include <QWidget>
#include "rplidarthread.h"

namespace Ui {
class RPLidarMessageMonitorForm;
}

/**
 * @brief Form used to show some data about messages sent by u-blox-devices.
 */
class RPLidarMessageMonitorForm : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent widget
     * @param title Form title
     */
    explicit RPLidarMessageMonitorForm(QWidget *parent = nullptr, const QString& title = "Message monitor");
    ~RPLidarMessageMonitorForm();

    /**
     * @brief Connects slots from LaserRangeFinder20HzV2SerialThread
     * @param serThread SerialThread to connect signals from
     */
    void connectRPLidarThreadSlots(RPLidarThread* rpLidarThread);

    /**
     * @brief Disconnects slots from SerialThread
     * @param serThread SerialThread to disconnect signals from
     */
    void disconnectRPLidarThreadSlots(RPLidarThread* rpLidarThread);

private:
    Ui::RPLidarMessageMonitorForm *ui;

    void addLogLine(const QString& line);
    QString getTimeDifferenceString(const qint64 startTime, const qint64 endTime);
    void updateStartAndEndTimes(const qint64 startTime, const qint64 endTime);

    qint64 lastStartTime = 0;
    qint64 lastEndTime = 0;

private slots:
    void errorMessage(const QString& errorMessage);
    void warningMessage(const QString& warningMessage);
    void infoMessage(const QString& infoMessage);

    void distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime);

#if 0
    void distanceReceived(const double& distance, qint64 startTime, qint64 endTime);
    void errorReceived(const QString& errorString, qint64 startTime, qint64 endTime);
    void unidentifiedDataReceived(const QByteArray& data, qint64 startTime, qint64 endTime);
#endif

    void on_pushButton_ClearAll_clicked();
};

#endif // RPLIDARMESSAGEMONITORFORM_H
