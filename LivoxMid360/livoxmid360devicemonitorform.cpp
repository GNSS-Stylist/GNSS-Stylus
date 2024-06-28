/*
    livoxmid360devicemonitorform.cpp (part of GNSS-Stylus)
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

#include <QHostAddress>

#include "livoxmid360devicemonitorform.h"
#include "ui_livoxmid360devicemonitorform.h"

LivoxMid360DeviceMonitorForm::LivoxMid360DeviceMonitorForm(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::LivoxMid360DeviceMonitorForm)
{
    ui->setupUi(this);

    connect(&deviceCounterUpdateTimer, &QTimer::timeout, this, &LivoxMid360DeviceMonitorForm::on_deviceCounterUpdateTimerTimeout);

    deviceCounterUpdateTimer.start(1000);
}

LivoxMid360DeviceMonitorForm::~LivoxMid360DeviceMonitorForm()
{
    clearDeviceList();
    clearSummaryList();
    delete ui;
}

void LivoxMid360DeviceMonitorForm::connectLivoxMid360Thread(LivoxMid360Thread* livoxMid360Thread)
{
    QObject::connect(livoxMid360Thread, &LivoxMid360Thread::pushLidarInformationReceived,
                     this, &LivoxMid360DeviceMonitorForm::pushLidarInformationReceived);

    this->mid360Thread = livoxMid360Thread;
}

void LivoxMid360DeviceMonitorForm::disconnectLivoxMid360Thread(LivoxMid360Thread* livoxMid360Thread)
{
    QObject::disconnect(livoxMid360Thread, &LivoxMid360Thread::pushLidarInformationReceived,
                     this, &LivoxMid360DeviceMonitorForm::pushLidarInformationReceived);

    this->mid360Thread = nullptr;
}

static QString getStringOrNA(const QString& sourceString, bool valid)
{
    if (valid)
    {
        return sourceString;
    }
    else
    {
        return "N/A";
    }
}

static QString getTimeValueOrNA(const quint64& value, bool valid)
{
    if (valid)
    {
        return QString::number(value);
    }
    else
    {
        return "N/A";
    }
}

static QString getTimeValueOrNA(const qint64& value, bool valid)
{
    if (valid)
    {
        return QString::number(value);
    }
    else
    {
        return "N/A";
    }
}

void LivoxMid360DeviceMonitorForm::pushLidarInformationReceived(quint32 ipAddress, const LivoxMid360::PushLidarInformation& info, qint64 upTime)
{
    (void) upTime;

    QElapsedTimer upTimeTimer;
    upTimeTimer.start();
    quint64 localUptime = upTimeTimer.msecsSinceReference();

    DeviceItem* deviceItem;

    if (deviceItems.contains(ipAddress))
    {
        deviceItem = deviceItems.value(ipAddress);
    }
    else
    {
        // This device doesn't exist in the list yet, create new item (line) for it

        deviceItem = new DeviceItem;
        deviceItem->treeWidget = std::make_unique<QTreeWidgetItem>(ui->treeWidget_DetectedDevices);
        deviceItem->lastMessageTime = localUptime;

        deviceItems.insert(ipAddress, deviceItem);
    }

    QTreeWidgetItem* treeItem = deviceItem->treeWidget.get();

    treeItem->setText(0, QHostAddress(ipAddress).toString());
    treeItem->setText(1, "0 s");
    treeItem->setText(2, info.sn);
    treeItem->setText(3, getTimeValueOrNA(info.local_time_now, info.local_time_now_valid));
    treeItem->setText(4, getTimeValueOrNA(info.last_sync_time, info.last_sync_time_valid));
    treeItem->setText(5, getTimeValueOrNA(info.time_offset, info.time_offset_valid));

    QString timeSyncString;
    if (info.time_sync_type_valid)
    {
        const char *timeSyncStrings[] = { "No sync", "PTP", "GPS" };

        if (info.time_sync_type < sizeof(timeSyncStrings) / sizeof(timeSyncStrings[0]))
        {
            timeSyncString = timeSyncStrings[info.time_sync_type];
        }
        else
        {
            timeSyncString = "Value out of range";
        }
    }
    treeItem->setText(6, timeSyncString);

    QString workStateString;

    if (info.cur_work_state_valid)
    {
        const char *workStateStrings[] =
            {
            "Value out of range",   // 0
            "Sampling",             // 1
            "Idle",                 // 2
            "Value out of range",   // 3
            "Error",                // 4
            "Selfcheck",            // 5
            "Motor startup",        // 6
            "Value out of range",   // 7
            "Upgrade",              // 8
            "Ready",                // 9
            };

        if (info.cur_work_state < sizeof(workStateStrings) / sizeof(workStateStrings[0]))
        {
            workStateString = workStateStrings[info.cur_work_state];
        }
        else
        {
            workStateString = "Value out of range";
        }
    }

    treeItem->setText(7, workStateString);

    treeItem->setText(8, getStringOrNA(QString::number(double(info.core_temp) / 100.0), info.core_temp_valid));
}

void LivoxMid360DeviceMonitorForm::clearDeviceList(void)
{
    for (auto iter = deviceItems.constBegin(); iter != deviceItems.constEnd(); iter++)
    {
        delete iter.value();
    }

    deviceItems.clear();
}

void LivoxMid360DeviceMonitorForm::on_pushButton_ClearDevices_clicked()
{
    clearDeviceList();
}

void LivoxMid360DeviceMonitorForm::on_deviceCounterUpdateTimerTimeout()
{
    if (!mid360Thread)
    {
        // Thread not running, just keep previous sum values in the list but clear the rate-fields.

        for (int i = 0; i < ui->treeWidget_DataSummary->topLevelItemCount(); i++)
        {
            ui->treeWidget_DataSummary->topLevelItem(i)->setText(2, "0 (N/A)");
            ui->treeWidget_DataSummary->topLevelItem(i)->setText(4, "0 (N/A)");
            ui->treeWidget_DataSummary->topLevelItem(i)->setText(6, "0 (N/A)");
        }

        return;
    }

    QMap<quint32, LivoxMid360Thread::DeviceCounters> threadCounters = mid360Thread->getDeviceCounters();

    CounterItem* counterItem;

    for (auto threadCounterIter = threadCounters.begin(); threadCounterIter != threadCounters.end(); threadCounterIter++)
    {
        quint32 ipAddress = threadCounterIter.key();

        if (deviceCounters.contains(threadCounterIter.key()))
        {
            counterItem = deviceCounters.value(ipAddress);
        }
        else
        {
            // This device doesn't exist in the list yet, create a new item (line) for it

            counterItem = new CounterItem;
            counterItem->treeWidget = std::make_unique<QTreeWidgetItem>(ui->treeWidget_DataSummary);
            counterItem->counters = threadCounterIter.value();

            deviceCounters.insert(ipAddress, counterItem);
        }

/*        LivoxMid360Thread::DeviceCounters c1;
        LivoxMid360Thread::DeviceCounters c2;

        LivoxMid360Thread::DeviceCounters c3 = c1- counterItem->counters;
*/

        LivoxMid360Thread::DeviceCounters difference = threadCounterIter.value() - counterItem->counters;
        counterItem->counters = threadCounterIter.value();

        QTreeWidgetItem* treeItem = counterItem->treeWidget.get();

        treeItem->setText(0, QHostAddress(ipAddress).toString());
        treeItem->setText(1, QString::number(counterItem->counters.pointCloudPoints, 'g', 4));
        treeItem->setText(2, QString::number(difference.pointCloudPoints, 'g', 3));
        treeItem->setText(3, QString::number(counterItem->counters.imuDataDatagrams, 'g', 4));
        treeItem->setText(4, QString::number(difference.imuDataDatagrams, 'g', 3));
        treeItem->setText(5, QString::number(counterItem->counters.totalDatagramBytes, 'g', 4));
        treeItem->setText(6, QString::number(difference.totalDatagramBytes, 'g', 4));



        /*        treeItem->setText(2, info.sn);
        treeItem->setText(3, getTimeValueOrNA(info.local_time_now, info.local_time_now_valid));
        treeItem->setText(4, getTimeValueOrNA(info.last_sync_time, info.last_sync_time_valid));
        treeItem->setText(5, getTimeValueOrNA(info.time_offset, info.time_offset_valid));
*/
    }

    for (auto counterIter = deviceCounters.begin(); counterIter != deviceCounters.end(); counterIter++)
    {
        if (!threadCounters.contains(counterIter.key()))
        {
            // List contains devices not currently active in the thread.
            // Clear the rate-fields.
            counterIter.value()->treeWidget->setText(2, "0 (N/A)");
            counterIter.value()->treeWidget->setText(4, "0 (N/A)");
        }
    }
}

void LivoxMid360DeviceMonitorForm::clearSummaryList(void)
{
    for (auto iter = deviceCounters.constBegin(); iter != deviceCounters.constEnd(); iter++)
    {
        delete iter.value();
    }

    deviceCounters.clear();
}

void LivoxMid360DeviceMonitorForm::on_pushButton_ClearSummary_clicked()
{
    clearSummaryList();
}


void LivoxMid360DeviceMonitorForm::on_pushButton_ResetCounter_clicked()
{
    if (mid360Thread)
    {
        mid360Thread->clearDeviceCounters();
    }
}

