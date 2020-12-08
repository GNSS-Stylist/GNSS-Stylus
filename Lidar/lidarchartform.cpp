/*
    lidarchartform.cpp (part of GNSS-Stylus)
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

#include <math.h>
#include <QDebug>
#include <QElapsedTimer>
#include <QSettings>
#include "lidarchartform.h"
#include "ui_lidarchartform.h"
#include "rplidarplausibilityfilter.h"

LidarChartForm::LidarChartForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LidarChartForm)
{
    ui->setupUi(this);

    polarChart = new QtCharts::QPolarChart();

    lineSeries_Distance_Filtered = new QtCharts::QLineSeries();
    lineSeries_Distance_Filtered->setName("Distance (line)");
    lineSeries_Distance_Filtered->setUseOpenGL(true); // "Polar charts do not support accelerated series"

    scatterSeries_Distance_Filtered = new QtCharts::QScatterSeries();
    scatterSeries_Distance_Filtered->setName("Distance (scatter)");
    scatterSeries_Distance_Filtered->setUseOpenGL(true);  // "Polar charts do not support accelerated series"
    scatterSeries_Distance_Filtered->setPen(QPen(Qt::PenStyle::NoPen));
    scatterSeries_Distance_Filtered->setMarkerSize(5);

    lineSeries_Quality = new QtCharts::QLineSeries();
    lineSeries_Quality->setName("Quality (line)");
    lineSeries_Quality->setUseOpenGL(true); // "Polar charts do not support accelerated series"

    scatterSeries_Quality = new QtCharts::QScatterSeries();
    scatterSeries_Quality->setName("Quality (scatter)");
    scatterSeries_Quality->setUseOpenGL(true);  // "Polar charts do not support accelerated series"
    scatterSeries_Quality->setPen(QPen(Qt::PenStyle::NoPen));
    scatterSeries_Quality->setMarkerSize(5);

    polarChart->addSeries(lineSeries_Quality);
    polarChart->addSeries(scatterSeries_Quality);
    polarChart->addSeries(lineSeries_Distance_Filtered);
    polarChart->addSeries(scatterSeries_Distance_Filtered);

    angularAxis = new QtCharts::QValueAxis();
    angularAxis->setTickCount(81); // First and last ticks are co-located on 0/360 angle.
    angularAxis->setLabelFormat("%d");
    angularAxis->setShadesVisible(true);
    angularAxis->setShadesBrush(QBrush(QColor(249, 249, 255)));
    angularAxis->setRange(0, 360);
    polarChart->addAxis(angularAxis, QtCharts::QPolarChart::PolarOrientationAngular);
//    chart->addAxis(angularAxis, Qt::AlignBottom);

    radialAxis_Distance = new QtCharts::QValueAxis();
    radialAxis_Distance->setTickCount(9);
    radialAxis_Distance->setLabelFormat("%.1f");
    radialAxis_Distance->setRange(0,3);
    polarChart->addAxis(radialAxis_Distance, QtCharts::QPolarChart::PolarOrientationRadial);
//    chart->addAxis(radialAxis, Qt::AlignLeft);

    radialAxis_Quality = new QtCharts::QValueAxis();
    radialAxis_Quality->setTickCount(2);
    radialAxis_Quality->setLabelsVisible(false);
//    radialAxis_Quality->setLabelFormat("%.1f");
    radialAxis_Quality->setRange(0,1);
    polarChart->addAxis(radialAxis_Quality, QtCharts::QPolarChart::PolarOrientationRadial);

    lineSeries_Distance_Filtered->attachAxis(angularAxis);
    lineSeries_Distance_Filtered->attachAxis(radialAxis_Distance);

    scatterSeries_Distance_Filtered->attachAxis(angularAxis);
    scatterSeries_Distance_Filtered->attachAxis(radialAxis_Distance);

    lineSeries_Quality->attachAxis(angularAxis);
    lineSeries_Quality->attachAxis(radialAxis_Quality);

    scatterSeries_Quality->attachAxis(angularAxis);
    scatterSeries_Quality->attachAxis(radialAxis_Quality);

//    polarChart->setTitle("Lidar polar");

    ui->chartView->setChart(polarChart);

    treeItem_LastRound_Samples = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_Samples->setText(0, "Samples");

    treeItem_LastRound_Duration = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_Duration->setText(0, "Duration (round)");

    treeItem_LastRound_SamplesPerSec = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_SamplesPerSec->setText(0, "Samples/s");

    treeItem_LastRound_RoundsPerSec = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_RoundsPerSec->setText(0, "Rounds/s");

    treeItem_LastRound_DiscardedSamples_Quality = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_DiscardedSamples_Quality->setText(0, "Discarded samples (quality)");

    treeItem_LastRound_DiscardedSamples_Filtering = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_DiscardedSamples_Filtering->setText(0, "Discarded samples (filtering)");

    treeItem_LastRound_ChartUpdateTime = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_ChartUpdateTime->setText(0, "Chart update time");

    treeItem_LastRound_TimeAfterLastData = new QTreeWidgetItem(ui->treeWidget_LastRound);
    treeItem_LastRound_TimeAfterLastData->setText(0, "Time after last data");


    treeItem_Statistics_Rounds_Total = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Rounds_Total->setText(0, "Received rounds");

    treeItem_Statistics_Rounds_Handled = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Rounds_Handled->setText(0, "Handled rounds");

    treeItem_Statistics_Rounds_Skipped_LagPrevention = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Rounds_Skipped_LagPrevention->setText(0, "Skipped rounds (lag)");

    treeItem_Statistics_Rounds_Skipped_Deliberate = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Rounds_Skipped_Deliberate->setText(0, "Skipped rounds (deliberate)");

    treeItem_Statistics_ChartUpdateTime_Average = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_ChartUpdateTime_Average->setText(0, "Chart update time avg");

    treeItem_Statistics_Samples_Total = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Samples_Total->setText(0, "Samples");

    treeItem_Statistics_Samples_Discarded_Quality = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Samples_Discarded_Quality->setText(0, "Discarded samples (quality)");

    treeItem_Statistics_Samples_Discarded_Filtering = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_Samples_Discarded_Filtering->setText(0, "Discarded samples (filtering)");

    treeItem_Statistics_SamplesPerSecond_Total_Average = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_SamplesPerSecond_Total_Average->setText(0, "Received samples/s avg");

    treeItem_Statistics_RoundsPerSecond_Total_Average = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_RoundsPerSecond_Total_Average->setText(0, "Received rounds/s avg");

    treeItem_Statistics_SamplesPerRound_Total_Average = new QTreeWidgetItem(ui->treeWidget_Statistics);
    treeItem_Statistics_SamplesPerRound_Total_Average->setText(0, "Samples/round avg");

    QSettings settings;

    ui->spinBox_Settings_RoundsToSkip->setValue(settings.value("LidarChart_RoundsToSkip", "0").toInt());

    for (int i = 0; i < polarChart->series().count(); i++)
    {
        QString seriesName = polarChart->series().at(i)->name();

        QListWidgetItem* newItem = new QListWidgetItem(seriesName);

        Qt::CheckState checkState = settings.value("LidarPolarChart_SeriesVisible_" + seriesName, "0").toBool() ? Qt::Checked : Qt::Unchecked;
        newItem->setCheckState(checkState);
        polarChart->series().at(i)->setVisible(checkState == Qt::Checked);
        newItem->setFlags(newItem->flags() | Qt::ItemIsUserCheckable);
        ui->listWidget_SeriesVisibility->addItem(newItem);
        seriesVisibilityMap[newItem] = polarChart->series().at(i);
    }
}

LidarChartForm::~LidarChartForm()
{
    QSettings settings;

    settings.setValue("LidarChart_RoundsToSkip", ui->spinBox_Settings_RoundsToSkip->value());

    for (int i = 0; i < ui->listWidget_SeriesVisibility->count(); i++)
    {
        QString seriesName = ui->listWidget_SeriesVisibility->item(i)->text();

        settings.setValue("LidarPolarChart_SeriesVisible_" + seriesName, ui->listWidget_SeriesVisibility->item(i)->checkState() == Qt::Checked);
    }

    delete ui;
}

void LidarChartForm::connectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::connect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived_RealTime(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void LidarChartForm::disconnectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::disconnect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived_RealTime(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void LidarChartForm::connectRPLidarPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::connect(postProcessingForm, SIGNAL(replayData_Lidar(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived_Replay(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void LidarChartForm::disconnectRPLidarPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Lidar(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived_Replay(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void LidarChartForm::distanceRoundReceived_RealTime(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime)
{
    distanceRoundReceived(data, startTime, endTime, true);
}

void LidarChartForm::distanceRoundReceived_Replay(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime)
{
    distanceRoundReceived(data, startTime, endTime, false);
}

void LidarChartForm::distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime, const bool lagDetection)
{
    totalRounds++;
    totalSamples += data.size();

    skipCounter++;

    QElapsedTimer timer;

    timer.start();
    qint64 uptime = timer.msecsSinceReference();

    if (statisticsStartTime == 0)
    {
        // First round or statistics reset -> Store starttime and clear counters

        totalRounds = 0;
        totalSamples = 0;

        updateStatisticFields();

        statisticsStartTime = uptime;
        skipCounter = 1e9;  // Update next round
        return;
    }

    if (skipCounter > (unsigned int)ui->spinBox_Settings_RoundsToSkip->value())
    {
        skipCounter = 0;

        lastRoundReceivedUptime = uptime;
        lastRoundStartUptime = startTime;
        lastRoundEndUptime = endTime;

        if (((uptime - endTime) < 100) || (!lagDetection))
        {
            lastRoundDistanceItems = data;
            updateChartData();
        }
        else
        {
            // Most probably this machine is not able to handle data fast enough -> Just skip it
            // qDebug() << "Skip";
            totalSkippedRounds_LagPrevention++;
        }

        // Start incrementing these after first round to make frequency calculations correct
        // (Time counting starts after the first round received).
        totalHandledRounds++;
        totalHandledSamples += data.size();
    }
    else
    {
        totalSkippedRounds_Deliberate++;
    }

    statisticsEndTime = uptime;

    updateStatisticFields();
}

void LidarChartForm::updateChartData(void)
{
    QElapsedTimer elapsedTimer;
    elapsedTimer.start();

//    lineSeries->clear();
//    scatterSeries->clear();

    QVector<QPointF> filteredItems;
    QVector<QPointF> qualities;

    RPLidarPlausibilityFilter filter;

    QVector<RPLidarPlausibilityFilter::FilteredItem> rpFilteredItems;

    filter.filter(lastRoundDistanceItems, rpFilteredItems);

    for (int i = 0; i < rpFilteredItems.size(); i++)
    {
        if (rpFilteredItems[i].type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
        {
            if (lineSeries_Distance_Filtered->isVisible() || scatterSeries_Distance_Filtered->isVisible())
            {
                QPointF newPoint(rpFilteredItems[i].item.angle * 360 / (2 * M_PI), rpFilteredItems[i].item.distance);
                filteredItems.push_back(newPoint);
            }
//            lineSeries->append(lastRoundDistanceItems[i].angle * 360 / (2 * M_PI), lastRoundDistanceItems[i].distance);
//            scatterSeries->append(lastRoundDistanceItems[i].angle * 360 / (2 * M_PI), lastRoundDistanceItems[i].distance);
        }

        if (lineSeries_Quality->isVisible() || scatterSeries_Quality->isVisible())
        {
            QPointF newQualityPoint(rpFilteredItems[i].item.angle * 360 / (2 * M_PI), rpFilteredItems[i].item.quality);
            qualities.push_back(newQualityPoint);
        }
    }

    if (lineSeries_Distance_Filtered->isVisible())
    {
        lineSeries_Distance_Filtered->replace(filteredItems);
    }
    else
    {
        lineSeries_Distance_Filtered->clear();
    }

    if (scatterSeries_Distance_Filtered->isVisible())
    {
        scatterSeries_Distance_Filtered->replace(filteredItems);
    }
    else
    {
        scatterSeries_Distance_Filtered->clear();
    }

    if (lineSeries_Quality->isVisible())
    {
        lineSeries_Quality->replace(qualities);
    }
    else
    {
        lineSeries_Quality->clear();
    }

    if (scatterSeries_Quality->isVisible())
    {
        scatterSeries_Quality->replace(qualities);
    }
    else
    {
        scatterSeries_Quality->clear();
    }

    unsigned int elapsed_us = elapsedTimer.nsecsElapsed() / 1000;

    totalChartUpdateTime_us += elapsed_us;

    const QBrush brush_Valid = QBrush(QColor(128,255,128));
    const QBrush brush_Indeterminate = QBrush(QColor(255,255,0));
    const QBrush brush_Invalid = QBrush(QColor(255,128,128));

    if ((lastRoundReceivedUptime != 0) && (lastRoundStartUptime !=0) && (lastRoundEndUptime != 0) &&
            (lastRoundEndUptime > lastRoundStartUptime))
    {
        treeItem_LastRound_Samples->setText(1, QString::number(lastRoundDistanceItems.size()));


        treeItem_LastRound_Samples->setText(1, QString::number(lastRoundDistanceItems.size()));
        treeItem_LastRound_Duration->setText(1, QString::number(lastRoundEndUptime - lastRoundStartUptime));
        treeItem_LastRound_SamplesPerSec->setText(1, QString::number(lastRoundDistanceItems.size() * 1000 / (lastRoundEndUptime - lastRoundStartUptime)));
        treeItem_LastRound_RoundsPerSec->setText(1, QString::number(1000. / (lastRoundEndUptime - lastRoundStartUptime), 'f', 2));

        treeItem_LastRound_DiscardedSamples_Quality->setText(1, QString::number(0));
        treeItem_LastRound_DiscardedSamples_Filtering->setText(1, QString::number(0));

        treeItem_LastRound_ChartUpdateTime->setText(1, QString::number(elapsed_us) + " us (" +
                                                    QString::number(100. * (elapsed_us / 1000. / (ui->spinBox_Settings_RoundsToSkip->value() + 1)) /
                                                                    (lastRoundEndUptime - lastRoundStartUptime), 'f', 1) + " %)");
        treeItem_LastRound_TimeAfterLastData->setText(1, "0");
    }
    else
    {
        treeItem_LastRound_Samples->setText(1, "N/A");
        treeItem_LastRound_Duration->setText(1, "N/A");
        treeItem_LastRound_SamplesPerSec->setText(1, "N/A");
        treeItem_LastRound_RoundsPerSec->setText(1, "N/A");
        treeItem_LastRound_DiscardedSamples_Quality->setText(1, "N/A");
        treeItem_LastRound_DiscardedSamples_Filtering->setText(1, "N/A");
        treeItem_LastRound_DiscardedSamples_Quality->setText(1, "N/A");
        treeItem_LastRound_DiscardedSamples_Filtering->setText(1, "N/A");
    }


}

void LidarChartForm::updateStatisticFields(void)
{
    if ((totalRounds != 0) && (statisticsStartTime != 0))
    {
        QElapsedTimer timer;
        timer.start();
        qint64 uptime = timer.msecsSinceReference();

        treeItem_Statistics_Rounds_Total->setText(1, QString::number(totalRounds));
        treeItem_Statistics_Rounds_Handled->setText(1, QString::number(totalHandledRounds));
        treeItem_Statistics_Rounds_Skipped_LagPrevention->setText(1, QString::number(totalSkippedRounds_LagPrevention) +
                                                                  " (" + QString::number(100. * totalSkippedRounds_LagPrevention / totalRounds, 'f', 1) + " %)");
        treeItem_Statistics_Rounds_Skipped_Deliberate->setText(1, QString::number(totalSkippedRounds_Deliberate) +
                                                               " (" + QString::number(100. * totalSkippedRounds_Deliberate / totalRounds, 'f', 1) + " %)");
        QString chartUpdateTimeString = "N/A";
        if (totalHandledRounds > 0)
        {
            chartUpdateTimeString = QString::number(totalChartUpdateTime_us / 1000. / totalHandledRounds, 'f', 1) + " ms";
        }

        QString samplesPerSecondAvgString = "N/A";
        QString roundsPerSecondAvgString = "N/A";

        if ((statisticsStartTime != 0) && (uptime - statisticsStartTime > 0))
        {
            qint64 elapsed = uptime - statisticsStartTime;

            if (totalHandledRounds > 0)
            {
                chartUpdateTimeString += " (" + QString::number(100. * (totalChartUpdateTime_us / 1000.) / elapsed, 'f', 1) + "%)";
            }
            samplesPerSecondAvgString = QString::number(totalSamples * 1000 / elapsed);
            roundsPerSecondAvgString = QString::number(totalRounds * 1000. / elapsed, 'f', 2);
        }

        treeItem_Statistics_ChartUpdateTime_Average->setText(1, chartUpdateTimeString);
        treeItem_Statistics_Samples_Total->setText(1, QString::number(totalSamples));
        treeItem_Statistics_Samples_Discarded_Quality->setText(1, QString::number(totalDiscardedSamples_Quality));
        treeItem_Statistics_Samples_Discarded_Filtering->setText(1, QString::number(totalDiscardedSamples_Filtering));
        treeItem_Statistics_SamplesPerSecond_Total_Average->setText(1, samplesPerSecondAvgString);
        treeItem_Statistics_RoundsPerSecond_Total_Average->setText(1, roundsPerSecondAvgString);
        treeItem_Statistics_SamplesPerRound_Total_Average->setText(1, QString::number(totalSamples / totalRounds));
    }
    else
    {
        treeItem_Statistics_Rounds_Total->setText(1, "0");
        treeItem_Statistics_Rounds_Handled->setText(1, "0");
        treeItem_Statistics_Rounds_Skipped_LagPrevention->setText(1, "0");
        treeItem_Statistics_Rounds_Skipped_Deliberate->setText(1, "0");
        treeItem_Statistics_ChartUpdateTime_Average->setText(1, "N/A");
        treeItem_Statistics_Samples_Total->setText(1, "0");
        treeItem_Statistics_Samples_Discarded_Quality->setText(1, "0");
        treeItem_Statistics_Samples_Discarded_Filtering->setText(1, "0");
        treeItem_Statistics_SamplesPerSecond_Total_Average->setText(1, "N/A");
        treeItem_Statistics_RoundsPerSecond_Total_Average->setText(1, "N/A");
        treeItem_Statistics_SamplesPerRound_Total_Average->setText(1, "N/A");
    }
}


void LidarChartForm::on_pushButton_ResetStatistics_clicked()
{
    totalRounds = 0;
    totalHandledRounds = 0;
    totalSkippedRounds_Deliberate = 0;
    totalSkippedRounds_LagPrevention = 0;

    totalSamples = 0;
    totalHandledSamples = 0;
    statisticsStartTime = 0;
    statisticsEndTime = 0;
    totalDiscardedSamples_Quality = 0;
    totalDiscardedSamples_Filtering = 0;
    totalChartUpdateTime_us = 0;

    updateStatisticFields();
}

void LidarChartForm::on_listWidget_SeriesVisibility_itemChanged(QListWidgetItem *item)
{
    QMap<QListWidgetItem*, QtCharts::QAbstractSeries*>::iterator iter = seriesVisibilityMap.find(item);

    if (iter != seriesVisibilityMap.end())
    {
        if (iter.value()->isVisible() != item->checkState())
        {
            iter.value()->setVisible(item->checkState());
            updateChartData();
        }
    }
}
