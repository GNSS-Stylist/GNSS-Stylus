/*
    livoxmid360devicemonitorform.h (part of GNSS-Stylus)
    Copyright (C) 2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef LIVOXMID360DEVICEMONITORFORM_H
#define LIVOXMID360DEVICEMONITORFORM_H

#include <QWidget>
#include <QMap>
#include <QTreeWidget>
#include <QTimer>
#include "livoxmid360thread.h"

namespace Ui {
class LivoxMid360DeviceMonitorForm;
}

class LivoxMid360DeviceMonitorForm : public QWidget
{
    Q_OBJECT

public:
    explicit LivoxMid360DeviceMonitorForm(QWidget *parent = nullptr);
    ~LivoxMid360DeviceMonitorForm();

    /**
     * @brief Connects slots from LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to connect signals from
     */
    void connectLivoxMid360Thread(LivoxMid360Thread* mid360Thread);

    /**
     * @brief Disconnects slots from LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to disconnect signals from
     */
    void disconnectLivoxMid360Thread(LivoxMid360Thread* mid360Thread);

private slots:
    void on_pushButton_ClearDevices_clicked();
    void on_deviceCounterUpdateTimerTimeout();

    void on_pushButton_ClearSummary_clicked();

    void on_pushButton_ResetCounter_clicked();

private:
    Ui::LivoxMid360DeviceMonitorForm *ui;

    class DeviceItem
    {
    public:
        quint64 lastMessageTime;
        std::unique_ptr<QTreeWidgetItem> treeWidget;
    };

    QMap<quint32, DeviceItem*> deviceItems;

    class CounterItem
    {
    public:
        LivoxMid360Thread::DeviceCounters counters;
        std::unique_ptr<QTreeWidgetItem> treeWidget;
    };

    LivoxMid360Thread* mid360Thread = nullptr;
    QMap<quint32, CounterItem*> deviceCounters;
    quint64 lastDeviceCounterUpdateUptime = 0;
    QTimer deviceCounterUpdateTimer;
//    QMap<quint32, QTreeWidgetItem*> counterTreeItems;

    void pushLidarInformationReceived(quint32 ipAddress, const LivoxMid360::PushLidarInformation&, qint64);
    void clearDeviceList(void);
    void clearSummaryList(void);
};

#endif // LIVOXMID360DEVICEMONITORFORM_H
