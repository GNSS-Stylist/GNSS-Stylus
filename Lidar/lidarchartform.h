/*
    lidarchartform.h (part of GNSS-Stylus)
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

#ifndef LIDARCHARTFORM_H
#define LIDARCHARTFORM_H

#include <QWidget>
#include <QChart>
#include <QPolarChart>
#include <QLineSeries>
#include <QScatterSeries>
#include <QValueAxis>
#include <QTreeWidgetItem>
#include <QListWidgetItem>

#include "rplidarthread.h"
#include "../PostProcessing/postprocessform.h"

namespace Ui {
class LidarChartForm;
}

class LidarChartForm : public QWidget
{
    Q_OBJECT

public:
    explicit LidarChartForm(QWidget *parent = nullptr);
    ~LidarChartForm();

    /**
     * @brief Connects slots from RPLidarThread
     * @param rpLidarThread RPLidarThread to connect signals from
     */
    void connectRPLidarThreadSlots(RPLidarThread* rpLidarThread);

    /**
     * @brief Disconnects slots from RPLidarThread
     * @param rpLidarThread RPLidarThread to disconnect signals from
     */
    void disconnectRPLidarThreadSlots(RPLidarThread* rpLidarThread);

    /**
     * @brief Connects slots from PostProcessingForm
     * @param postProcessingForm PostProcessingForm to connect signals from
     */
    void connectRPLidarPostProcessingSlots(PostProcessingForm* postProcessingForm);

    /**
     * @brief Disconnects slots from PostProcessingForm
     * @param postProcessingForm PostProcessingForm to disconnect signals from
     */
    void disconnectRPLidarPostProcessingSlots(PostProcessingForm* postProcessingForm);

private slots:

    void distanceRoundReceived_RealTime(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime);
    void distanceRoundReceived_Replay(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime);

    void on_pushButton_ResetStatistics_clicked();

    void on_listWidget_SeriesVisibility_itemChanged(QListWidgetItem *item);

private:
    Ui::LidarChartForm *ui;

//    bool treeItemsCreated = false;
    QTreeWidgetItem* treeItem_LastRound_Samples = nullptr;
    QTreeWidgetItem* treeItem_LastRound_Duration = nullptr;
    QTreeWidgetItem* treeItem_LastRound_SamplesPerSec = nullptr;
    QTreeWidgetItem* treeItem_LastRound_RoundsPerSec = nullptr;
    QTreeWidgetItem* treeItem_LastRound_DiscardedSamples_Quality = nullptr;
    QTreeWidgetItem* treeItem_LastRound_DiscardedSamples_Filtering = nullptr;
    QTreeWidgetItem* treeItem_LastRound_ChartUpdateTime = nullptr;
    QTreeWidgetItem* treeItem_LastRound_TimeAfterLastData = nullptr;

    QTreeWidgetItem* treeItem_Statistics_Rounds_Total = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Rounds_Handled = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Rounds_Skipped_LagPrevention = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Rounds_Skipped_Deliberate = nullptr;
    QTreeWidgetItem* treeItem_Statistics_ChartUpdateTime_Average = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Samples_Total = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Samples_Discarded_Quality = nullptr;
    QTreeWidgetItem* treeItem_Statistics_Samples_Discarded_Filtering = nullptr;
    QTreeWidgetItem* treeItem_Statistics_SamplesPerSecond_Total_Average = nullptr;
    QTreeWidgetItem* treeItem_Statistics_RoundsPerSecond_Total_Average = nullptr;
    QTreeWidgetItem* treeItem_Statistics_SamplesPerRound_Total_Average = nullptr;

//    QtCharts::QChart* chart = nullptr;
    QtCharts::QPolarChart* polarChart = nullptr;
    QtCharts::QValueAxis* angularAxis = nullptr;
    QtCharts::QValueAxis* radialAxis_Distance = nullptr;
    QtCharts::QValueAxis* radialAxis_Quality = nullptr;

    QtCharts::QLineSeries* lineSeries_Distance_Filtered = nullptr;
    QtCharts::QScatterSeries* scatterSeries_Distance_Filtered = nullptr;

    QtCharts::QLineSeries* lineSeries_Quality = nullptr;
    QtCharts::QScatterSeries* scatterSeries_Quality = nullptr;

    QMap<QListWidgetItem*, QtCharts::QAbstractSeries*> seriesVisibilityMap;

    QVector<RPLidarThread::DistanceItem> lastRoundDistanceItems;
    qint64 lastRoundReceivedUptime = 0;
    qint64 lastRoundStartUptime = 0;
    qint64 lastRoundEndUptime = 0;

    // Statistics:
    unsigned int totalRounds = 0;
    unsigned int totalHandledRounds = 0;
    unsigned int totalSkippedRounds_Deliberate = 0;
    unsigned int totalSkippedRounds_LagPrevention = 0;

    qint64 totalSamples = 0;
    qint64 totalHandledSamples = 0;
    qint64 statisticsStartTime = 0;
    qint64 statisticsEndTime = 0;
    qint64 totalDiscardedSamples_Quality = 0;
    qint64 totalDiscardedSamples_Filtering = 0;
    qint64 totalChartUpdateTime_us = 0;

    unsigned int skipCounter = 1e9;

    void distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime, const bool lagDetection);

    void updateChartData(void);
    void updateStatisticFields(void);

};

#endif // LIDARCHARTFORM_H
