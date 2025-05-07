/*
    pointcloudgeneratorlidar.cpp (part of GNSS-Stylus)
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

#include <algorithm>
#include <QProgressDialog>
#include <QElapsedTimer>

#include "pointcloudgeneratorlidar.h"
#include "../asyncpointcloudfilewriter.h"

namespace Lidar
{

void PointCloudGenerator::generatePointClouds(const Params& params)
{
    // The comment below was written before changing this to support multi-threading,
    // but leaving this here if Stylus' implementation will be changed also:
    // This function is identical to the one used in Stylus' PointCloudGenerator.
    // I actually first wrote a base class so that this function was implemented there
    // and only "specialized" generatePointCloudPointSet-function was implemented in
    // Lidar's and Stylus' subclasses. This, however felt like an overkill and obfuscated
    // the structure instead of making it clearer.
    // Maybe duplicating 200+ lines of code can be seen as a bad practise, but whatever...

    bool dontWriteFiles = false;
    QString fileExtension;

    switch (params.fileParams.fileFormat)
    {
    case AsyncPointCloudFileWriter::Params::FF_NONE:
        dontWriteFiles = true;
        break;
    case AsyncPointCloudFileWriter::Params::FF_PLY:
        fileExtension = ".ply";
        break;
    case AsyncPointCloudFileWriter::Params::FF_XYZ:
        fileExtension = ".xyz";
        break;
    default:
        qFatal("File format handling not implemented!");
        break;
    }

    if ((!dontWriteFiles) && (!params.directory.exists()))
    {
        emit errorMessage("Directory \"" + params.directory.path() + "\" doesn't exist. Point cloud files not created.");
        return;
    }

    emit infoMessage("Processing...");

    bool objectActive = false;

    qint64 beginningUptime = -1;

    bool ignoreBeginningAndEndingTags = false;

    QString objectName;
    QString baseFileName;
    int fileIndex = 0;

    qint64 uptime = -1;
    PostProcessingForm::Tag beginningTag;

    QQueue<PointCloudGeneratorLidarThread::WorkUnit> workUnitQueue;

    int outFileChunkIndex = 0;

    std::shared_ptr<AsyncPointCloudFileWriter> currentFileWriter = nullptr;
    QMap<QString, std::shared_ptr<AsyncPointCloudFileWriter> > fileWriters;

    emit infoMessage("Creating work units (for worker threads)...");

    int pointSetIndex = 0;

    QMap<qint64, PostProcessingForm::ScanningState>::const_iterator scanningStateIter = params.scanningStateMap->constBegin();

    while (scanningStateIter != params.scanningStateMap->constEnd())
    {
        uptime = scanningStateIter.key();

        if (((scanningStateIter->objectActive) && !objectActive) || (scanningStateIter->objectName != objectName))
        {
            // Starting object

            baseFileName = QDir::cleanPath(params.directory.path() + "/" + scanningStateIter->objectName);

            if ((!params.separateFilesForSubScans) && (!dontWriteFiles))
            {
                // As all "sub scans" should go into the same file, create it now.
                QString fileName = baseFileName + fileExtension;

                outFileChunkIndex = 0;
                currentFileWriter = createNewOutFile(fileName, params.fileParams, scanningStateIter->currentTag, uptime, params.overwriteExistingFiles);

                if (!currentFileWriter)
                {
                    ignoreBeginningAndEndingTags = true;
                    continue;
                }
                else
                {
                    fileWriters.insert(fileName, currentFileWriter);
                    ignoreBeginningAndEndingTags = false;
                }
            }
            else
            {
                ignoreBeginningAndEndingTags = false;
            }

            objectActive = true;
            beginningUptime = -1;
            objectName = scanningStateIter->objectName;

            fileIndex = 0;
        }
        else if ((scanningStateIter->scanningActive) && (beginningUptime == -1) && (!ignoreBeginningAndEndingTags))
        {
            // Begin points

            // Just store the beginning uptime-value and tag. Creation of the work unit is done in ending tag-branch
            beginningUptime = uptime;
            beginningTag = scanningStateIter->currentTag;
        }
        else if ((!scanningStateIter->scanningActive) && (beginningUptime != -1) && (!ignoreBeginningAndEndingTags))
        {
            // End points

            if ((params.separateFilesForSubScans) && (!dontWriteFiles))
            {
                fileIndex++;

                QString fileIndexString = QString::number(fileIndex);

                while (fileIndexString.length() < 4)
                {
                    fileIndexString.prepend("0");
                }

                QString fileName = QDir::cleanPath(baseFileName + "_" + fileIndexString + fileExtension);

                outFileChunkIndex = 0;
                currentFileWriter = createNewOutFile(fileName, params.fileParams, scanningStateIter->currentTag, uptime, params.overwriteExistingFiles);

                if (!currentFileWriter)
                {
                    ignoreBeginningAndEndingTags = true;
                    continue;
                }
                else
                {
                    fileWriters.insert(fileName, currentFileWriter);
                    objectActive = true;
                    ignoreBeginningAndEndingTags = false;
                }
            }

            PointCloudGeneratorLidarThread::WorkUnit newWorkUnit;

            newWorkUnit.valid = true;
            if (!dontWriteFiles)
            {
                newWorkUnit.outFileName = currentFileWriter->getFileName();
            }
            else
            {
                newWorkUnit.outFileName.clear();
            }
            newWorkUnit.sourceFileName = beginningTag.sourceFile;
            newWorkUnit.beginningTagLine = beginningTag.sourceFileLine;
            newWorkUnit.endingTagLine = scanningStateIter->currentTag.sourceFileLine;
            newWorkUnit.pointSetIndex = pointSetIndex++;

            qint64 currentUptime = beginningUptime;

            while (currentUptime < uptime)
            {
                newWorkUnit.beginningUptime = currentUptime;

                if (uptime - currentUptime <= params.maxWorkUnitDuration)
                {
                    newWorkUnit.endingUptime = uptime;
                }
                else if (uptime - currentUptime < params.maxWorkUnitDuration * 2)
                {
                    // Split two last shorter work units even
                    newWorkUnit.endingUptime = currentUptime + ((uptime - currentUptime) / 2);
                }
                else
                {
                    newWorkUnit.endingUptime = currentUptime + params.maxWorkUnitDuration;
                }

                newWorkUnit.chunkIndex = outFileChunkIndex;

                newWorkUnit.beginningITOWTime_ns = getITOW(params.threadConstData.averagedSync, newWorkUnit.beginningUptime) * 1000000ULL;
                newWorkUnit.endingITOWTime_ns = getITOW(params.threadConstData.averagedSync, newWorkUnit.endingUptime) * 1000000ULL;

                workUnitQueue.enqueue(newWorkUnit);
                currentUptime = newWorkUnit.endingUptime;
                outFileChunkIndex++;
            }
            if (params.separateFilesForSubScans)
            {
                currentFileWriter = nullptr;
            }

            beginningUptime = -1;
        }
        scanningStateIter++;
    }

    if (beginningUptime != -1)
    {
        emit warningMessage("File \"" + beginningTag.sourceFile + "\", line " +
                   QString::number(beginningTag.sourceFileLine) +
                   ", iTOW " + QString::number(beginningUptime) +
                   ", iTOW " + QString::number(beginningTag.iTOW) +
                   " (beginning tag): File ended before ending tag. Points after beginning tag ignored.");
    }

    emit infoMessage("Work units created. Number of items: " + QString::number(workUnitQueue.size()));

    emit infoMessage("Creating " +  QString::number(params.numOfWorkerThreads) + " worker threads...");

    QMutex workUnitQueueMutex;

    auto lambdaGetter = [&workUnitQueue, &workUnitQueueMutex]
    {
        workUnitQueueMutex.lock();
        if (workUnitQueue.isEmpty())
        {
            PointCloudGeneratorLidarThread::WorkUnit dummyWorkUnit;
            workUnitQueueMutex.unlock();
            return dummyWorkUnit;
        }
        else
        {
            PointCloudGeneratorLidarThread::WorkUnit newWorkUnit = workUnitQueue.dequeue();
            workUnitQueueMutex.unlock();
            return newWorkUnit;
        }
    };

    auto lambdaProcessor = [&fileWriters, &dontWriteFiles](const PointCloudGeneratorLidarThread::Output& out)
    {
        if (!dontWriteFiles)
        {
            Q_ASSERT(fileWriters.contains(out.workUnit.outFileName));
            fileWriters.value(out.workUnit.outFileName)->addPoints(out);
        }
    };

    QVector<std::shared_ptr<PointCloudGeneratorLidarThread> > workerThreads;

    for (int i = 0; i < std::max(params.numOfWorkerThreads, 1); i++)
    {
        workerThreads.push_back(std::make_shared<PointCloudGeneratorLidarThread>(params.threadConstData, lambdaGetter, lambdaProcessor));
    }

    emit infoMessage(QString::number(params.numOfWorkerThreads) + " worker threads created.");

    emit infoMessage("Starting worker threads...");

    for (int i = 0; i < workerThreads.size(); i++)
    {
        workerThreads.at(i)->start();
    }

    emit infoMessage("Worker threads started.");

    int maxProgress = workUnitQueue.size() * 1000;
    QProgressDialog progress("Creating point cloud files...", "Abort", 0, maxProgress);
    if (dontWriteFiles)
    {
        progress.setLabelText("Simulating point cloud creation...");
    }

    progress.setMinimumDuration(0);
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(0);

    int queuedWrites;
    int numOfWorkerThreadsRunning;
    int monotonicProgress = 0;  // As reading progress values from different sources are not perfectly synchronized, show "monotonically rising" value.
    int workUnitQueueSize;

    QVector<int> errorPrintedForPointSets;

    bool aborted = false;

    do {
        queuedWrites = 0;
        QMap<QString, std::shared_ptr<AsyncPointCloudFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
        while (fileIter != fileWriters.constEnd())
        {
            queuedWrites += fileIter.value()->getQueueLength();

            QMap<int, QString> errors = fileIter.value()->getErrors();

            QMap<int, QString>::const_iterator errorIter = errors.constBegin();

            while (errorIter != errors.constEnd())
            {
                if (!errorPrintedForPointSets.contains(errorIter.key()))
                {
                    // Print error as it wasn't printed before.
                    // Error handling goes through file writer because it handles ordering of the output data packages.

                    emit warningMessage(errorIter.value());
                    errorPrintedForPointSets.push_back(errorIter.key());

                    workUnitQueueMutex.lock();

                    // As the first error aborts outputting any data, we can remove all work units corresponding to the same output file from the queue.
                    // (threads will still process the work units in their queues, but they will be discarded by the file writer).

                    for (int i = 0; i < workUnitQueue.size();)
                    {
                        if (workUnitQueue.at(i).pointSetIndex == errorIter.key())
                        {
                            workUnitQueue.removeAt(i);
                        }
                        else
                        {
                            i++;
                        }
                    }

                    workUnitQueueMutex.unlock();
                }
                errorIter++;
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
            PointCloudGeneratorLidarThread::State threadState = workerThreads.at(i)->getState(&progress);

            if (threadState == PointCloudGeneratorLidarThread::S_PROCESSING)
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

            QMap<QString, std::shared_ptr<AsyncPointCloudFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
            while (fileIter != fileWriters.constEnd())
            {
                fileIter.value()->requestTerminate(true);
                fileIter++;
            }

            emit infoMessage("Point cloud creation aborted. Output file contents may not be valid!");
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

        QMap<QString, std::shared_ptr<AsyncPointCloudFileWriter> >::const_iterator fileIter = fileWriters.constBegin();
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
            emit infoMessage("Point cloud generation simulation finished.");
        }
        else
        {
            emit infoMessage("Point cloud files generated.");
        }
    }
}


std::shared_ptr<AsyncPointCloudFileWriter> PointCloudGenerator::createNewOutFile(const QString fileName, const AsyncPointCloudFileWriter::Params &params, const PostProcessingForm::Tag &currentTag, const qint64 uptime, const bool overwriteExisting)
{
    QFile outFile(fileName);

    if (!overwriteExisting && outFile.exists())
    {
        // File already exists -> Not allowed

        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                   QString::number(currentTag.sourceFileLine)+
                   ", uptime " + QString::number(uptime) +
                   ", iTOW " + QString::number(currentTag.iTOW) +
                   ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

        return nullptr;
    }

    if (outFile.exists())
    {
        emit infoMessage("Overwriting file \"" + fileName + "\"...");
    }
    else
    {
        emit infoMessage("Creating file \"" + fileName + "\"...");
    }

    std::shared_ptr<AsyncPointCloudFileWriter> outFileWriter = std::make_shared<AsyncPointCloudFileWriter>(fileName, params);

    outFileWriter->start();

    if (!outFileWriter->isOpenedSuccessfully())
    {
        // Creating the file failed

        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                   QString::number(currentTag.sourceFileLine)+
                   ", uptime " + QString::number(uptime) +
                   ", iTOW " + QString::number(currentTag.iTOW) +
                   ": File \"" + fileName + "\" can't be created. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

        return nullptr;
    }

    return outFileWriter;
}

UBXMessage_RELPOSNED::ITOW PointCloudGenerator::getITOW(const QMap<qint64, UBXMessage_RELPOSNED::ITOW> *averagedSync, const quint64& uptime_ms)
{
    // TODO: This whole ITOW/uptime-conversion hassle should be rethough.
    // This function, for example is identical to the one found from LidarScriptGenerator.
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
