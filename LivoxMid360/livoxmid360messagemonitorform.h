/*
    livoxmid360messagemonitorform.h (part of GNSS-Stylus)
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
 * @file livoxmid360messagemonitorform.h
 * @brief Declaration for a form that shows some data about messages sent by Livox Mid-360.
 */

#ifndef LIVOXMID360MESSAGEMONITORFORM_H
#define LIVOXMID360MESSAGEMONITORFORM_H

#include <QWidget>
#include "livoxmid360thread.h"

namespace Ui {
class LivoxMid360MessageMonitorForm;
}

/**
 * @brief Form used to show some data about messages sent by Livox Mid-360.
 */
class LivoxMid360MessageMonitorForm : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent widget
     * @param title Form title
     */
    explicit LivoxMid360MessageMonitorForm(QWidget *parent = nullptr, const QString& title = "Message monitor");
    ~LivoxMid360MessageMonitorForm();

    /**
     * @brief Connects slots from LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to connect signals from
     */
    void connectLivoxMid360ThreadSlots(LivoxMid360Thread* mid360Thread);

    /**
     * @brief Disconnects slots from LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to disconnect signals from
     */
    void disconnectLivoxMid360ThreadSlots(LivoxMid360Thread* mid360Thread);

private:
    Ui::LivoxMid360MessageMonitorForm *ui;

    void addLogLine(const QString& line);

    qint64 lastStartTime = 0;
    qint64 lastEndTime = 0;

private slots:
    void errorMessage(const QString& errorMessage);
    void warningMessage(const QString& warningMessage);
    void infoMessage(const QString& infoMessage);
//    void handleDatagram();

//    void distanceRoundReceived(const QVector<LivoxMid360Thread::DistanceItem>& data, qint64 startTime, qint64 endTime);

    void on_pushButton_ClearAll_clicked();
};

#endif // LIVOXMID360MESSAGEMONITORFORM_H
