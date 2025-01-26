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

#include <QtCore> // QtEndian (below) seems to need this...
#include <QtEndian>
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

void AsyncPointCloudFileWriter::writePLYHeader(void)
{
    QByteArray eol = params.ply.endOfLine;

    QByteArray dataToWrite = "ply" + eol;

    if (params.ply.binary)
    {
        dataToWrite += "format binary_little_endian 1.0" + eol;
    }
    else
    {
        dataToWrite += "format ascii 1.0" + eol;
    }

    dataToWrite += "comment File created with GNSS-Stylus on (dd.mm.yyyy hh:mm): " + QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm").toLatin1() + eol;

    // rows "comment pad" and "element vertex (N/A) added here this way to allow updating them later with simple overwriting some bytes.
    // Vertex count is not known when creating the file so it needs to be updated as one of the last steps when finalizing the file.
    // (Seems that Meshlab actually allows '0'-padded length, but the original paper doesn't say anything about padding, so better to be safe).
    dataToWrite += "comment pad";
    plyVertexCountFirstByte = dataToWrite.length();
    dataToWrite += "     " + eol + "element vertex (N/A)";
    plyVertexCountLastByte = dataToWrite.length();
    dataToWrite += eol;

    QByteArray coordType;
    if (params.ply.doublePrecisionCoords)
    {
        coordType = "double";
    }
    else
    {
        coordType = "float";
    }

    dataToWrite += "property " + coordType + " x" + eol;
    dataToWrite += "property " + coordType + " y" + eol;
    dataToWrite += "property " + coordType + " z" + eol;

    if (params.ply.includeNormals)
    {
        dataToWrite += "property float nx" + eol;
        dataToWrite += "property float ny" + eol;
        dataToWrite += "property float nz" + eol;
    }

    if (params.ply.includeQuality)
    {
        dataToWrite += "property float quality" + eol;
    }

    dataToWrite += "end_header" + eol;

    file.write(dataToWrite);
}

void AsyncPointCloudFileWriter::writePoint(const PointCloudGeneratorLidarThread::Output::Point * const point)
{
    switch (params.fileFormat)
    {
    case Params::FF_NONE:
        break;
    case Params::FF_XYZ:
        writePoint_XYZ(point);
        break;
    case Params::FF_PLY:
        writePoint_PLY(point);
        break;
    default:
        qFatal("Unimplemented file format.");
        break;
    }
}

void AsyncPointCloudFileWriter::writePoint_XYZ(const PointCloudGeneratorLidarThread::Output::Point * const point)
{
    QString lineOut = QString::number(point->hitPoint.x(), 'f', params.xyz.numberOfDecimals_Coords) +
                      "\t" + QString::number(point->hitPoint.y(), 'f', params.xyz.numberOfDecimals_Coords) +
                      "\t" + QString::number(point->hitPoint.z(), 'f', params.xyz.numberOfDecimals_Coords);

    if (params.xyz.includeNormals)
    {
        if (params.xyz.normalLengthAsQuality)
        {
            lineOut +=
                "\t" + QString::number(point->normal.x() * point->quality, 'f', params.xyz.numberOfDecimals_Normal) +
                "\t" + QString::number(point->normal.y() * point->quality, 'f', params.xyz.numberOfDecimals_Normal) +
                "\t" + QString::number(point->normal.z() * point->quality, 'f', params.xyz.numberOfDecimals_Normal);
        }
        else
        {
            lineOut +=
                "\t" + QString::number(point->normal.x(), 'f', params.xyz.numberOfDecimals_Normal) +
                "\t" + QString::number(point->normal.y(), 'f', params.xyz.numberOfDecimals_Normal) +
                "\t" + QString::number(point->normal.z(), 'f', params.xyz.numberOfDecimals_Normal);
        }
    }

    lineOut += params.xyz.endOfLine;
    QByteArray bytesToWrite = lineOut.toLatin1();
    file.write(bytesToWrite);
}

static inline void writeDouble(QFile& file, const double src)
{
    char buf[8];
    qToLittleEndian(src, buf);
    file.write(buf, 8);
}

static inline void writeFloat(QFile& file, const float src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

void AsyncPointCloudFileWriter::writePoint_PLY(const PointCloudGeneratorLidarThread::Output::Point * const point)
{
    if (params.ply.binary)
    {
        if (params.ply.doublePrecisionCoords)
        {
            writeDouble(file, point->hitPoint.x());
            writeDouble(file, point->hitPoint.y());
            writeDouble(file, point->hitPoint.z());
        }
        else
        {
            writeFloat(file, point->hitPoint.x());
            writeFloat(file, point->hitPoint.y());
            writeFloat(file, point->hitPoint.z());
        }

        if (params.ply.includeNormals)
        {
            if (params.ply.normalLengthAsQuality)
            {
                writeFloat(file, point->normal.x() * point->quality);
                writeFloat(file, point->normal.y() * point->quality);
                writeFloat(file, point->normal.z() * point->quality);
            }
            else
            {
                writeFloat(file, point->normal.x());
                writeFloat(file, point->normal.y());
                writeFloat(file, point->normal.z());
            }
        }

        if (params.ply.includeQuality)
        {
            writeFloat(file, point->quality);
        }
    }
    else
    {
        QString lineOut = QString::number(point->hitPoint.x(), 'f', params.ply.numberOfDecimals_Coords) +
                          " " + QString::number(point->hitPoint.y(), 'f', params.ply.numberOfDecimals_Coords) +
                          " " + QString::number(point->hitPoint.z(), 'f', params.ply.numberOfDecimals_Coords);

        if (params.ply.includeNormals)
        {
            if (params.ply.normalLengthAsQuality)
            {
                lineOut +=
                    " " + QString::number(point->normal.x() * point->quality, 'f', params.ply.numberOfDecimals_Normal) +
                    " " + QString::number(point->normal.y() * point->quality, 'f', params.ply.numberOfDecimals_Normal) +
                    " " + QString::number(point->normal.z() * point->quality, 'f', params.ply.numberOfDecimals_Normal);
            }
            else
            {
                lineOut +=
                    " " + QString::number(point->normal.x(), 'f', params.ply.numberOfDecimals_Normal) +
                    " " + QString::number(point->normal.y(), 'f', params.ply.numberOfDecimals_Normal) +
                    " " + QString::number(point->normal.z(), 'f', params.ply.numberOfDecimals_Normal);
            }
        }

        if (params.ply.includeQuality)
        {
            lineOut += " " + QString::number(point->quality, 'f', params.ply.numberOfDecimals_Quality);
        }

        lineOut += params.ply.endOfLine;
        QByteArray bytesToWrite = lineOut.toLatin1();
        file.write(bytesToWrite);
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
        finalizeFile_PLY();
        break;
    }
    default:
        qFatal("Unimplemented file format.");
        break;
    }
}


void AsyncPointCloudFileWriter::finalizeFile_PLY(void)
{
    // Need to write number of vertices as one of the last steps as it isn't known before.
    numberOfPointsWrittenMutex.lock();
    QByteArray stringToWrite = QString::number(numberOfPointsWritten).toLatin1();
    numberOfPointsWrittenMutex.unlock();
    stringToWrite = params.ply.endOfLine + "element vertex " + QString::number(numberOfPointsWritten).toLatin1();

    int numOfBytesToWrite = plyVertexCountLastByte - plyVertexCountFirstByte;
    while (stringToWrite.length() < numOfBytesToWrite)
    {
        stringToWrite = QByteArray(" ") + stringToWrite;
    }

    file.seek(plyVertexCountFirstByte);
    file.write(stringToWrite);
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
            waitConditionMutex.unlock();
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

            auto pointIter = outputData.points->constBegin();

            while (pointIter != outputData.points->constEnd())
            {
                if (terminateRequest == TR_ABANDONPENDINGDATA)
                {
                    break;
                }

                writePoint(pointIter);

                numberOfPointsWritten++;

                pointIter++;
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

