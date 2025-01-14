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

#include <QProgressDialog>
#include <QElapsedTimer>

#include "pointcloudgeneratorlidar.h"
#include "../asyncpointcloudfilewriter.h"

namespace Lidar
{

void PointCloudGenerator::generatePointClouds(const Params& params)
{
    // The comment above was written before changing this to support multi-threading,
    // but leaving this here if Stylus' implementation will be changed also:
    // This function is identical to the one used in Stylus' PointCloudGenerator.
    // I actually first wrote a base class so that this function was implemented there
    // and only "specialized" generatePointCloudPointSet-function was implemented in
    // Lidar's and Stylus' subclasses. This, however felt like an overkill and obfuscated
    // the structure instead of making it clearer.
    // Maybe duplicating 200+ lines of code can be seen as a bad practise, but whatever...

    if (!params.directory.exists())
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

    while (params.tags->upperBound(uptime) != params.tags->end())
    {
        uptime = params.tags->upperBound(uptime).key();

        QList<PostProcessingForm::Tag> tagItems = params.tags->values(uptime);

        // Since "The items that share the same key are available from most recently to least recently inserted."
        // (taken from QMultiMap's doc), iterate in "reverse order" here

        for (int i = tagItems.size() - 1; i >= 0; i--)
        {
            const PostProcessingForm::Tag& currentTag = tagItems[i];

            if (!(currentTag.ident.compare(params.tagIdent_BeginNewObject)))
            {
                // Tag type: new object

                if (objectActive)
                {
                    currentFileWriter = nullptr;
                    objectActive = false;
                }

                objectName = currentTag.text;

                if (currentTag.text.length() == 0)
                {
                    // Empty name for the new object not allowed

                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": New object without a name. Ending previous object, but not beginning new nor creating a new file. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    continue;
                }

                baseFileName = QDir::cleanPath(params.directory.path() + "/" + currentTag.text);

                QString fileName = baseFileName + ".xyz";

                if (!params.separateFilesForSubScans)
                {
                    outFileChunkIndex = 0;
                    currentFileWriter = createNewOutFile(fileName, currentTag, uptime);

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
                    emit infoMessage("Starting new object \"" +  currentTag.text + "\".");
                    ignoreBeginningAndEndingTags = false;
                }

                objectActive = true;
                beginningUptime = -1;

                fileIndex = 0;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: Begin points

                if (!objectActive)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": Beginning tag outside object. Skipped.");
                    continue;
                }

                if (beginningUptime != -1)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": Duplicate beginning tag. Skipped.");
                    continue;
                }

                // Just store the beginning uptime-value and tag. Writing of the points is done in ending tag-branch
                beginningUptime = uptime;
                beginningTag = currentTag;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: end points

                if (!objectActive)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": End tag outside object. Skipped.");
                    continue;
                }

                if (beginningUptime == -1)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": End tag without beginning tag. Skipped.");
                    continue;
                }

                const PostProcessingForm::Tag& endingTag = currentTag;

                if (endingTag.sourceFile != beginningTag.sourceFile)
                {
                    emit warningMessage("Starting and ending tags belong to different files. Starting tag file \"" +
                               beginningTag.sourceFile + "\", line " +
                               QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                               endingTag.sourceFile + "\", line " +
                               QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                    continue;
                }

                if (params.separateFilesForSubScans)
                {
                    fileIndex++;

                    QString fileIndexString = QString::number(fileIndex);

                    while (fileIndexString.length() < 4)
                    {
                        fileIndexString.prepend("0");
                    }

                    QString fileName = QDir::cleanPath(baseFileName + "_" + fileIndexString + ".xyz");

                    outFileChunkIndex = 0;
                    currentFileWriter = createNewOutFile(fileName, currentTag, uptime);

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
                newWorkUnit.outFileName = currentFileWriter->getFileName();
                newWorkUnit.sourceFileName = beginningTag.sourceFile;
                newWorkUnit.beginningTagLine = beginningTag.sourceFileLine;
                newWorkUnit.endingTagLine = endingTag.sourceFileLine;

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
        }
    }

    if (beginningUptime != -1)
    {
        emit warningMessage("File \"" + beginningTag.sourceFile + "\", line " +
                   QString::number(beginningTag.sourceFileLine) +
                   ", iTOW " + QString::number(beginningUptime) +
                   ", iTOW " + QString::number(beginningTag.iTOW) +
                   " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
    }

    emit infoMessage("work units created. Number of items: " + QString::number(workUnitQueue.size()));

    emit infoMessage("Creating worker threads (" + QString::number(params.numOfWorkerThreads) + ")...");

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

    auto lambdaProcessor = [&fileWriters](const PointCloudGeneratorLidarThread::Output& out)
    {
        Q_ASSERT(fileWriters.contains(out.workUnit.outFileName));
        fileWriters.value(out.workUnit.outFileName)->addPoints(out);
    };

    QVector<std::shared_ptr<PointCloudGeneratorLidarThread> > workerThreads;

    for (int i = 0; i < std::max(params.numOfWorkerThreads, 1); i++)
    {
        workerThreads.push_back(std::make_shared<PointCloudGeneratorLidarThread>(params.threadConstData, lambdaGetter, lambdaProcessor));
    }

    emit infoMessage(QString::number(params.numOfWorkerThreads) + " worker threads Created.");

    int maxProgress = workUnitQueue.size();
    QProgressDialog progress("Creating point cloud files...", "Abort", 0, maxProgress);
    progress.setWindowModality(Qt::WindowModal);

    emit infoMessage("Starting worker threads...");

    for (int i = 0; i < workerThreads.size(); i++)
    {
        workerThreads.at(i)->start();
    }

    emit infoMessage("Worker threads started.");

    while (workUnitQueue.size() != 0)
    {
        int currentProgress = maxProgress - workUnitQueue.size();
        progress.setValue(currentProgress);
        QThread::msleep(100);

        if (progress.wasCanceled())
            break;
    }

    progress.setValue(maxProgress);

    emit infoMessage("Waiting for worker threads to end...");

    for (int i = 0; i < workerThreads.size(); i++)
    {
        workerThreads.at(i)->wait();
    }

    emit infoMessage("Worker threads finished.");

    // TODO: Handle pending buffered writes.

    emit infoMessage("Point cloud files generated.");
}


std::shared_ptr<AsyncPointCloudFileWriter> PointCloudGenerator::createNewOutFile(const QString fileName, const PostProcessingForm::Tag &currentTag, const qint64 uptime)
{
    QFile outFile(fileName);

    if (outFile.exists())
    {
        // File already exists -> Not allowed

        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                   QString::number(currentTag.sourceFileLine)+
                   ", uptime " + QString::number(uptime) +
                   ", iTOW " + QString::number(currentTag.iTOW) +
                   ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

        return nullptr;
    }

    emit infoMessage("Creating file \"" + fileName + "\"...");

    std::shared_ptr<AsyncPointCloudFileWriter> outFileWriter = std::make_unique<AsyncPointCloudFileWriter>(fileName);

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


}; // namespace Lidar
