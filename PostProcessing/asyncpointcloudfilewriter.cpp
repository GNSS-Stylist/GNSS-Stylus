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
#include "asyncpointcloudfilewriter.h"

AsyncPointCloudFileWriter::AsyncPointCloudFileWriter(const QString& fileName)
{
    this->fileName = fileName;
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
    file = new QFile(fileName);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        fileOpenedSuccesfully = false;
        fileHandleMutex.unlock(); // Allow reading of the opened state
        return;
    }

    textStream = new QTextStream(file);

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

            // As these writes are already done in dedicated thread, it's better to write data to file straight away.
            // (buffer PointCloudGeneratorLidarThread::Outputs rather than written bytes).
            textStream->flush();
            file->flush();

            outBufferMutex.lock();
            nextChunkToWrite++;
        }
        outBufferMutex.unlock();

        waitConditionMutex.unlock();
    }

    textStream->flush();
    delete textStream;

    file->close();
    delete file;
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
