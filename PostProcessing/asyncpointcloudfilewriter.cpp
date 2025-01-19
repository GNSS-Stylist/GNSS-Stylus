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

AsyncPointCloudFileWriter::AsyncPointCloudFileWriter(const QString& fileName, const Params& params)
{
    this->fileName = fileName;
    this->params = params;
    fileHandleMutex.lock();     // This mutex is used to synchronize isOpenedSuccessfully, therefore lock it already
}

AsyncPointCloudFileWriter::~AsyncPointCloudFileWriter()
{
    if (terminateRequest == TR_NONE)
    {
        terminateRequest = TR_WRITEPENDINGDATA;
    }
    wait();
}

bool AsyncPointCloudFileWriter::isOpenedSuccessfully(void)
{
    fileHandleMutex.lock();
    bool retval = fileOpenedSuccesfully;
    fileHandleMutex.unlock();
    return retval;
}

bool AsyncPointCloudFileWriter::openFile(void)
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
        return true;
    case Params::FF_XYZ:
    case Params::FF_PLY:
        return file.open(QIODevice::WriteOnly);
    default:
        qFatal("Unimplemented file format.");
        return false;
    }
}

void AsyncPointCloudFileWriter::writePLYHeader(void)
{
    // TODO: Write ply-header
    QByteArray dataToWrite = "TODO: Write ply-header!\n";
    file.write(dataToWrite);
}

void AsyncPointCloudFileWriter::writeHeader(void)
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
    case Params::FF_XYZ:
        break;
    case Params::FF_PLY:
        writePLYHeader();
        break;

    default:
        qFatal("Unimplemented file format.");
        break;
    }
}

void AsyncPointCloudFileWriter::writePoint(const PointCloudGeneratorLidarThread::Output::Point& point)
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
        break;
    case Params::FF_XYZ:
    {
        QString lineOut = QString::number(point.hitPoint.x(), 'f', params.xyz.numberOfDecimals_Coords) +
            "\t" + QString::number(point.hitPoint.y(), 'f', params.xyz.numberOfDecimals_Coords) +
            "\t" + QString::number(point.hitPoint.z(), 'f', params.xyz.numberOfDecimals_Coords);

        if (params.xyz.includeNormals)
        {
            if (params.xyz.normalLengthAsQuality)
            {
                lineOut +=
                    "\t" + QString::number(point.normal.x() * point.quality, 'f', params.xyz.numberOfDecimals_Normal) +
                    "\t" + QString::number(point.normal.y() * point.quality, 'f', params.xyz.numberOfDecimals_Normal) +
                    "\t" + QString::number(point.normal.z() * point.quality, 'f', params.xyz.numberOfDecimals_Normal);
            }
            else
            {
                lineOut +=
                    "\t" + QString::number(point.normal.x(), 'f', params.xyz.numberOfDecimals_Normal) +
                    "\t" + QString::number(point.normal.y(), 'f', params.xyz.numberOfDecimals_Normal) +
                    "\t" + QString::number(point.normal.z(), 'f', params.xyz.numberOfDecimals_Normal);
            }
        }

        lineOut += params.xyz.endOfLine;
        QByteArray bytesToWrite = lineOut.toLatin1();
        file.write(bytesToWrite);

        break;
    }
    case Params::FF_PLY:
        break;
    default:
        qFatal("Unimplemented file format.");
        break;
    }
}


void AsyncPointCloudFileWriter::finalizeFile(void)
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
    case Params::FF_XYZ:
        break;
    case Params::FF_PLY:
    {
        // TODO: Add writing of number of points into the header etc.
        break;
    }
    default:
        qFatal("Unimplemented file format.");
        break;
    }
}


void AsyncPointCloudFileWriter::run()
{
    file.setFileName(fileName);

    fileOpenedSuccesfully = openFile();

    if (!fileOpenedSuccesfully)
    {
        fileHandleMutex.unlock(); // Allow reading of the opened state
        return;
    }

    fileHandleMutex.unlock(); // Allow reading of the opened state

    writeHeader();

    while (true)
    {
        if (terminateRequest == TR_ABANDONPENDINGDATA)
        {
            break;
        }

        waitConditionMutex.lock();
        waitCondition.wait(&waitConditionMutex, 100);

        outBufferMutex.lock();

        if (outBuffer.isEmpty() && terminateRequest == TR_WRITEPENDINGDATA)
        {
            outBufferMutex.unlock();
            break;
        }

        while ((outBuffer.contains(nextChunkToWrite)) && (terminateRequest != TR_ABANDONPENDINGDATA))
        {
            if (terminateRequest == TR_ABANDONPENDINGDATA)
            {
                break;
            }

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

            numberOfPointsWrittenMutex.lock();

            for (const PointCloudGeneratorLidarThread::Output::Point& item : *outputData.points.get())
            {
                if (terminateRequest == TR_ABANDONPENDINGDATA)
                {
                    break;
                }

                writePoint(item);

                numberOfPointsWritten++;
            }

            numberOfPointsWrittenMutex.unlock();

            // As these writes are already done in this dedicated thread, it's better to write data to file straight away.
            // (buffer PointCloudGeneratorLidarThread::Outputs rather than written bytes).
            if (file.isOpen())
            {
                file.flush();
            }

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

    finalizeFile();

    if (file.isOpen())
    {
        file.close();
    }
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

unsigned int AsyncPointCloudFileWriter::getNumberOfPointsWritten(void)
{
    unsigned int retval;
    numberOfPointsWrittenMutex.lock();
    retval = numberOfPointsWritten;
    numberOfPointsWrittenMutex.unlock();
    return retval;
}

void AsyncPointCloudFileWriter::requestTerminate(bool abandonPendingWrites)
{
    if (abandonPendingWrites)
    {
        terminateRequest = TR_ABANDONPENDINGDATA;
    }
    else
    {
        terminateRequest = TR_WRITEPENDINGDATA;
    }

    waitCondition.wakeOne();
};

