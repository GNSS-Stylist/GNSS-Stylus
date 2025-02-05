/*
    lidarscriptgenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include <QMessageBox>
#include <QPushButton>
#include <QProgressDialog>
#include "lidarscriptgenerator.h"
#include "../asynclidarscriptfilewriter.h"

namespace Lidar
{

void LidarScriptGenerator::generateLidarScript(const Params& params)
{
    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;
    filteredItems.reserve(10000);

    RPLidarPlausibilityFilter plausibilityFilter;
    plausibilityFilter.setSettings(*params.threadConstData.rpLidar.filteringSettings);

    QMap<LidarDevice, std::shared_ptr<AsyncLidarScriptFileWriter> > fileWriters;

    emit infoMessage("Searching for devices and creating output files...");

//    qint64 iTOWTime_Min = get

    auto rpLidarIter = params.threadConstData.rpLidar.rounds->lowerBound(params.uptime_Min);

//    qint64 minLogUptime = std::numeric_limits<qint64>::max();
//    qint64 maxLogUptime = 0;

    if ((rpLidarIter != params.threadConstData.rpLidar.rounds->end()) && (rpLidarIter.key() <= params.uptime_Max))
    {
        QString fileName = QDir::cleanPath(params.baseFileName + "_RPLidar.lidarscript");

        emit infoMessage("Data for RPLidar device found. Creating file \"" + fileName + "\"...");
        QFile outFile(fileName);
        if (outFile.exists())
        {
            emit errorMessage("File \"" + fileName + "\" already exists. Aborting generating lidar scripts.");
            return;
        }

        if (!params.dontWriteFiles)
        {
            std::shared_ptr<AsyncLidarScriptFileWriter> outFileWriter = std::make_shared<AsyncLidarScriptFileWriter>(fileName, params.fileParams);

            outFileWriter->start();

            if (!outFileWriter->isOpenedSuccessfully())
            {
                // Creating the file failed
                emit errorMessage("File \"" + fileName + "\" can't be created. Aborting generating lidar scripts.");
                return;
            }

            LidarDevice device(LidarDevice::DT_RPLIDAR);

            fileWriters.insert(device, outFileWriter);
        }

//        minLogUptime = std::min(minLogUptime, rpLidarIter.key());
//        maxLogUptime = std::max(maxLogUptime, params.threadConstData.rpLidar.rounds->last().endTime);
    }

    qint64 iTOWTime_Min_ns = getITOW(params.threadConstData.averagedSync, params.uptime_Min) * 1000000ULL;
    qint64 iTOWTime_Max_ns = getITOW(params.threadConstData.averagedSync, params.uptime_Max) * 1000000ULL;

    auto mid360Iter = params.threadConstData.mid360.datagrams->lowerBound(iTOWTime_Min_ns);

    // Slight speedup(?) compare only ip-addresses (as quint32s) instead of LidarDevices
    QVector<quint32> foundMid360Devices;

    while ((mid360Iter != params.threadConstData.mid360.datagrams->end()) && (mid360Iter.key() <= iTOWTime_Max_ns))
    {
        quint32 ipAddress = mid360Iter.value()->datagram.senderAddress().toIPv4Address();

        if (!foundMid360Devices.contains(ipAddress))
        {
            LidarDevice device(LidarDevice::DT_LIVOX_MID360, ipAddress);
            foundMid360Devices.push_back(ipAddress);

            QString ipAddressString = mid360Iter.value()->datagram.senderAddress().toString();
            QString ipAddressString_Snake = ipAddressString;
            ipAddressString_Snake.replace('.', '_');

            QString fileName = QDir::cleanPath(params.baseFileName + "_Mid360_" + ipAddressString_Snake + ".lidarscript");

            emit infoMessage("Data for Mid-360 device, IP-address " + ipAddressString + " found. Creating file \"" + fileName + "\"...");
            QFile outFile(fileName);
            if (outFile.exists())
            {
                emit errorMessage("File \"" + fileName + "\" already exists. Aborting generating lidar scripts.");
                return;
            }

            std::shared_ptr<AsyncLidarScriptFileWriter> outFileWriter = std::make_shared<AsyncLidarScriptFileWriter>(fileName, params.fileParams);

            outFileWriter->start();

            if (!outFileWriter->isOpenedSuccessfully())
            {
                // Creating the file failed
                emit errorMessage("File \"" + fileName + "\" can't be created. Aborting generating lidar scripts.");
                return;
            }

            fileWriters.insert(device, outFileWriter);
        }

//        minLogUptime = std::min(minLogUptime, mid360Iter.key() / 1000000);
//        maxLogUptime = std::max(maxLogUptime, mid360Iter.key() / 1000000);

        mid360Iter++;
    }

    if (fileWriters.isEmpty())
    {
        emit infoMessage("No data for any device during the selected time period. Quitting generating lidar scripts.");
        return;
    }

//    qint64 currentUptime = std::max(params.uptime_Min, minLogUptime);
//    qint64 maxUptime = std::min(params.uptime_Max, maxLogUptime);

    qint64 currentUptime = params.uptime_Min;
    qint64 maxUptime = params.uptime_Max;

    LidarScriptGeneratorThread::WorkUnit newWorkUnit;
    newWorkUnit.valid = true;
    newWorkUnit.chunkIndex = 0;

    emit infoMessage("Creating work units (for worker threads)...");

    QQueue<LidarScriptGeneratorThread::WorkUnit> workUnitQueue;

    while (currentUptime < maxUptime)
    {
        newWorkUnit.beginningUptime = currentUptime;

        if (maxUptime - currentUptime <= params.maxWorkUnitDuration)
        {
            newWorkUnit.endingUptime = maxUptime;
        }
        else if (maxUptime - currentUptime < params.maxWorkUnitDuration * 2)
        {
            // Split two last shorter work units even
            newWorkUnit.endingUptime = currentUptime + ((maxUptime - currentUptime) / 2);
        }
        else
        {
            newWorkUnit.endingUptime = currentUptime + params.maxWorkUnitDuration;
        }

        newWorkUnit.beginningITOWTime_ns = getITOW(params.threadConstData.averagedSync, newWorkUnit.beginningUptime) * 1000000ULL;
        newWorkUnit.endingITOWTime_ns = getITOW(params.threadConstData.averagedSync, newWorkUnit.endingUptime) * 1000000ULL;

        workUnitQueue.enqueue(newWorkUnit);
        currentUptime = newWorkUnit.endingUptime;
        newWorkUnit.chunkIndex++;
    }

    emit infoMessage("Work units created. Number of items: " + QString::number(workUnitQueue.size()));

    emit infoMessage("Creating " +  QString::number(params.numOfWorkerThreads) + " worker threads...");

    QMutex workUnitQueueMutex;

    auto lambdaGetter = [&workUnitQueue, &workUnitQueueMutex]
    {
        workUnitQueueMutex.lock();
        if (workUnitQueue.isEmpty())
        {
            LidarScriptGeneratorThread::WorkUnit dummyWorkUnit;
            workUnitQueueMutex.unlock();
            return dummyWorkUnit;
        }
        else
        {
            LidarScriptGeneratorThread::WorkUnit newWorkUnit = workUnitQueue.dequeue();
            workUnitQueueMutex.unlock();
            return newWorkUnit;
        }
    };

    bool dontWriteFiles = params.dontWriteFiles;

    auto lambdaProcessor = [&fileWriters, &dontWriteFiles](const LidarDevice& device, const LidarScriptGeneratorThread::Output& out)
    {
        if (!dontWriteFiles)
        {
            Q_ASSERT(fileWriters.contains(device));
            fileWriters.value(device)->addPoints(out);
        }
    };

    QVector<std::shared_ptr<LidarScriptGeneratorThread> > workerThreads;

    for (int i = 0; i < std::max(params.numOfWorkerThreads, 1); i++)
    {
        workerThreads.push_back(std::make_shared<LidarScriptGeneratorThread>(params.threadConstData, lambdaGetter, lambdaProcessor));
    }

    emit infoMessage(QString::number(params.numOfWorkerThreads) + " worker threads created.");

    emit infoMessage("Starting worker threads...");

    for (int i = 0; i < workerThreads.size(); i++)
    {
        workerThreads.at(i)->start();
    }

    emit infoMessage("Worker threads started.");

    int maxProgress = workUnitQueue.size() * 1000;
    QProgressDialog progress("Creating lidar script files...", "Abort", 0, maxProgress);
    if (dontWriteFiles)
    {
        progress.setLabelText("Simulating lidar script creation...");
    }

    progress.setMinimumDuration(0);
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(0);

    int queuedWrites;
    int numOfWorkerThreadsRunning;
    int monotonicProgress = 0;  // As reading progress values from different sources are not perfectly synchronized, show "monotonically rising" value.
    int workUnitQueueSize;

    QVector<LidarDevice> errorPrintedForDevices;

    bool aborted = false;

    do {
        queuedWrites = 0;
        QMap<LidarDevice, std::shared_ptr<AsyncLidarScriptFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
        while (fileIter != fileWriters.constEnd())
        {
            queuedWrites += fileIter.value()->getQueueLength();

            if (!errorPrintedForDevices.contains(fileIter.key()))
            {
                QString errorString;
                if (fileIter.value()->getError(errorString))
                {
                    emit errorMessage(errorString);
                    errorPrintedForDevices.push_back(fileIter.key());
                }
            }

            fileIter++;
        }

        workUnitQueueMutex.lock();
        workUnitQueueSize = workUnitQueue.size();
        workUnitQueueMutex.unlock();

        numOfWorkerThreadsRunning = 0;
        float sumOfThreadProgress = 0;

        for (int i = 0; i < workerThreads.size(); i++)
        {
            float progress;
            LidarScriptGeneratorThread::State threadState = workerThreads.at(i)->getState(&progress);

            if (threadState == LidarScriptGeneratorThread::S_PROCESSING)
            {
                numOfWorkerThreadsRunning++;
                sumOfThreadProgress += progress;
            }
        }

        int newProgress = maxProgress - ((workUnitQueueSize + queuedWrites) * 1000) + ((sumOfThreadProgress * 1000) / numOfWorkerThreadsRunning);
        monotonicProgress = std::max(newProgress, monotonicProgress);
        progress.setValue(monotonicProgress);
        QThread::msleep(100);

        if (progress.wasCanceled())
        {
            for (int i = 0; i < workerThreads.size(); i++)
            {
                workerThreads.at(i)->requestTerminate();
            }

            QMap<LidarDevice, std::shared_ptr<AsyncLidarScriptFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
            while (fileIter != fileWriters.constEnd())
            {
                fileIter.value()->requestTerminate(true);
                fileIter++;
            }

            emit infoMessage("Lidar script creation aborted. Output file contents may not be valid!");
            aborted = true;

            break;
        }
    } while ((workUnitQueueSize != 0) || (queuedWrites != 0) || (numOfWorkerThreadsRunning != 0));

    progress.setValue(maxProgress);

    emit infoMessage("Waiting for worker threads to finish...");
    for (int i = 0; i < workerThreads.size(); i++)
    {
        workerThreads.at(i)->wait();
    }
    emit infoMessage("Worker threads finished.");

    if (!dontWriteFiles)
    {
        emit infoMessage("Waiting for file writer threads to finish...");

        QMap<LidarDevice, std::shared_ptr<AsyncLidarScriptFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
        if (!aborted)
        {
            // No need to request termination here if canceled, since this was done with "more immediate"-flag earlier.
            while (fileIter != fileWriters.constEnd())
            {
                fileIter.value()->requestTerminate();
                fileIter++;
            }
        }

        fileIter = fileWriters.constBegin();
        while (fileIter != fileWriters.constEnd())
        {
            fileIter.value()->wait();
            fileIter++;
        }
        emit infoMessage("File writer threads finished.");
    }

    if (!aborted)
    {
        if (dontWriteFiles)
        {
            emit infoMessage("Lidar script generation simulation finished.");
        }
        else
        {
            emit infoMessage("Lidar script files generated.");
        }
    }
}

UBXMessage_RELPOSNED::ITOW LidarScriptGenerator::getITOW(const QMap<qint64, UBXMessage_RELPOSNED::ITOW> *averagedSync, const quint64& uptime_ms)
{
    // TODO: This whole ITOW/uptime-conversion hassle should be rethough.
    // This function, for example is identical to the one found from PointCloudGeneratorLidar.
    // Maybe create a new class that does the conversion back and forth, init it in PostProcessingForm-level and relay here and there?

    if (averagedSync->isEmpty())
    {
        return -1;
    }

    QMap<qint64, UBXMessage_RELPOSNED::ITOW>::const_iterator timeIter_High = averagedSync->upperBound(uptime_ms);

    if (timeIter_High == averagedSync->constEnd())
    {
        // No greater key available -> extrapolate based on the last item

        return averagedSync->last() + (uptime_ms - averagedSync->lastKey()) / 1000000;
    }

    if (timeIter_High == averagedSync->constBegin())
    {
        // No lower key available -> extrapolate based on the first item (which in this case is already in the iter)

        return timeIter_High.value() - (timeIter_High.key() - uptime_ms) / 1000000;
    }

    QMap<qint64, UBXMessage_RELPOSNED::ITOW>::const_iterator timeIter_Low = timeIter_High - 1;

    return timeIter_Low.value() + (timeIter_High.value() - timeIter_Low.value()) *
                                      (uptime_ms - timeIter_Low.key()) / (timeIter_High.key() - timeIter_Low.key());

}



}; // namespace Lidar

