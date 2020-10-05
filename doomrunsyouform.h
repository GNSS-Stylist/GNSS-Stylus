/*
    doomrunsyouform.h (part of GNSS-Stylus)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef DOOMRUNSYOUFORM_H
#define DOOMRUNSYOUFORM_H

#include <windows.h>
#include <QWidget>
#include <QTimer>
#include <QChart>
#include <QLineSeries>
#include <QValueAxis>
#include "Eigen/Geometry"

#include "gnssmessage.h"

namespace Ui {
class DoomRunsYouForm;
}

class DoomRunsYouForm : public QWidget
{
    enum CommandType
    {
        CT_LOCATION_ORIENTATION_COMMAND = 0,
        CT_PING_FROM_CLIENT,
        CT_PING_FROM_SERVER,
    };

    struct LocationOrientation
    {
        int uptime;
        double x;
        double y;
        double contYaw;
        double pitch;
    };

    Q_OBJECT

public:
    explicit DoomRunsYouForm(QWidget *parent = nullptr);
    ~DoomRunsYouForm();
    void newPositionData(const UBXMessage_RELPOSNED& roverA, const UBXMessage_RELPOSNED& roverB);

private slots:
    void on_pushButton_ReOpenPipe_clicked();

    void on_pushButton_ClosePipe_clicked();

    void on_pushButton_SendManualCommand_clicked();

    void fastTickTimerTimeout();

    void on_pushButton_Ping_clicked();

    void on_checkBox_Active_stateChanged(int arg1);

    void on_pushButton_ClearAll_clicked();

private:
    Ui::DoomRunsYouForm *ui;
    void addLogLine(const QString& line, bool important = false);
    HANDLE pipeHandle = INVALID_HANDLE_VALUE;
    int doomRunsYouCommandCounter = 1;

    int contYawRounds = 0;
    double lastYawCalculatedFromReceivedData = 0;

    int lastIntYaw = 0;
    int lastIntPitch = 0;

    double lastPosX = 0;
    double lastPosY = 0;

    QVector<LocationOrientation> locationOrientationHistory;

    QTimer fastTickTimer;

    int lastSentCommandUptime = 0;

    QtCharts::QLineSeries* lineSeries_Yaw_Unfiltered;
    QtCharts::QLineSeries* lineSeries_Yaw_Filtered;

    QtCharts::QLineSeries* lineSeries_Pitch_Unfiltered;
    QtCharts::QLineSeries* lineSeries_Pitch_Filtered;

    QtCharts::QChart* chart_Angles;
    QtCharts::QValueAxis* xAxis_Angles;
    QtCharts::QValueAxis* yAxis_Angles_Yaw;
    QtCharts::QValueAxis* yAxis_Angles_Pitch;

    void trimChart(void);
    void removeOldSeriesData(QtCharts::QLineSeries* series, int xAxisThreshold);

    qint64 uptimeBaseline = 0;
    int getRelativeUptime_ms(void);

    double posXFilteringStorage = 0;
    double posYFilteringStorage = 0;
    double yawFilteringStorage = 0;
    double pitchFilteringStorage = 0;

    double lastSentPosX = 0;
    double lastSentPosY = 0;

    double movementRoundingErrorX = 0;
    double movementRoundingErrorY = 0;

};

#endif // DOOMRUNSYOUFORM_H
