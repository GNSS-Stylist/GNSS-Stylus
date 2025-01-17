/*
    asyncpointcloudfilewriter.cpp (part of GNSS-Stylus)
    Copyright (C) 2024-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
#include <memory>
#include "asyncpointcloudfilewriter.h"

AsyncPointCloudFileWriter::AsyncPointCloudFileWriter(const QString& fileName, const Params& params)
{
    this->fileName = fileName;
    this->params = params;
    fileHandleMutex.lock();     // This mutex is used to synchronize isOpenedSuccessfully, therefore lock it already
}

AsyncPointCloudFileWriter::~AsyncPointCloudFileWriter()
{
    terminateRequest = true;
    wait();
}

bool AsyncPointCloudFileWriter::isOpenedSuccessfully(void)
{
    fileHandleMutex.lock();
    bool retval = fileOpenedSuccesfully;
    fileHandleMutex.unlock();
    return retval;
}

void AsyncPointCloudFileWriter::run()
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
    case Params::FF_PLY:    // TODO: Implement ply
        run_None();
        break;

    case Params::FF_XYZ:
        run_XYZ();
        break;

    default:
        qFatal("Unimplemented file format.");
        break;
    }
}


void AsyncPointCloudFileWriter::run_None(void)
{
    // This only simulates opening/writing files.
    // Implemented for testing purposes so that there's no need to delete files before each test run
    // (also to not wear out SSDs as the point cloud files can easily be several GBs).

    fileOpenedSuccesfully = true;

    fileHandleMutex.unlock(); // Allow reading of the opened state

    while (!terminateRequest)
    {
        waitConditionMutex.lock();
        waitCondition.wait(&waitConditionMutex, 100);

        outBufferMutex.lock();
        while (outBuffer.contains(nextChunkToWrite))
        {
            // Just throw away all chunks, but do it anyway "in the right order" to simulate keeping progress-calculation about right
            PointCloudGeneratorLidarThread::Output outputData = outBuffer.take(nextChunkToWrite);

            outBufferMutex.unlock();

            firstErrorsMutex.lock();
            if (firstErrors.contains(outputData.workUnit.pointSetIndex))
            {
                // Error detected before this chunk.
                firstErrorsMutex.unlock();
                nextChunkToWrite++;
                outBufferMutex.lock();
                continue;
            }
            firstErrorsMutex.unlock();

            if (outputData.result == PointCloudGeneratorLidarThread::Output::R_ERROR)
            {
                // Flag the error so that subsequent chunks wont be processed
                firstErrorsMutex.lock();
                firstErrors.insert(outputData.workUnit.pointSetIndex, outputData.errorString);
                firstErrorsMutex.unlock();
            }

            outBufferMutex.lock();
            nextChunkToWrite++;
        }
        outBufferMutex.unlock();

        waitConditionMutex.unlock();
    }
}

void AsyncPointCloudFileWriter::run_XYZ(void)
{
    std::unique_ptr<QFile> file = std::make_unique<QFile>(fileName);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        fileOpenedSuccesfully = false;
        fileHandleMutex.unlock(); // Allow reading of the opened state
        return;
    }

    std::unique_ptr<QTextStream>textStream = std::make_unique<QTextStream>(file.get());

    fileOpenedSuccesfully = true;

    fileHandleMutex.unlock(); // Allow reading of the opened state

    while (!terminateRequest)
    {
        waitConditionMutex.lock();
        waitCondition.wait(&waitConditionMutex, 100);

        outBufferMutex.lock();
        while (outBuffer.contains(nextChunkToWrite))
        {
            PointCloudGeneratorLidarThread::Output outputData = outBuffer.take(nextChunkToWrite);
            outBufferMutex.unlock();

            firstErrorsMutex.lock();
            if (firstErrors.contains(outputData.workUnit.pointSetIndex))
            {
                // Error detected before this chunk. Just remove from the buffer.
                firstErrorsMutex.unlock();
                outBufferMutex.lock();
                nextChunkToWrite++;
                continue;
            }
            firstErrorsMutex.unlock();

            for (auto item : *outputData.points.get())
            {
                QString lineOut = QString::number(item.hitPoint.x(), 'f', 4) +
                                  "\t" + QString::number(item.hitPoint.y(), 'f', 4) +
                                  "\t" + QString::number(item.hitPoint.z(), 'f', 4) +
                                  "\t" + QString::number(item.normal.x(), 'f', 4) +
                                  "\t" + QString::number(item.normal.y(), 'f', 4) +
                                  "\t" + QString::number(item.normal.z(), 'f', 4);

                textStream->operator<<(lineOut + "\n");
            }

            // As these writes are already done in this dedicated thread, it's better to write data to file straight away.
            // (buffer PointCloudGeneratorLidarThread::Outputs rather than written bytes).
            textStream->flush();
            file->flush();

            if (outputData.result == PointCloudGeneratorLidarThread::Output::R_ERROR)
            {
                // Flag the error so that subsequent chunks wont be processed
                firstErrorsMutex.lock();
                firstErrors.insert(outputData.workUnit.pointSetIndex, outputData.errorString);
                firstErrorsMutex.unlock();
            }

            outBufferMutex.lock();
            nextChunkToWrite++;
        }
        outBufferMutex.unlock();

        waitConditionMutex.unlock();
    }

    textStream->flush();
    file->close();
}

void AsyncPointCloudFileWriter::addPoints(const PointCloudGeneratorLidarThread::Output& out)
{
    outBufferMutex.lock();
    outBuffer.insert(out.workUnit.chunkIndex, out);
    outBufferMutex.unlock();

    waitCondition.wakeOne();
}

int AsyncPointCloudFileWriter::getQueueLength(void)
{
    int retval;

    outBufferMutex.lock();
    retval = outBuffer.size();
    outBufferMutex.unlock();
    return retval;
}

QMap<int, QString> AsyncPointCloudFileWriter::getErrors(void)
{
    QMap<int, QString> retval;
    firstErrorsMutex.lock();
    retval = firstErrors;
    firstErrorsMutex.unlock();
    return retval;
}

