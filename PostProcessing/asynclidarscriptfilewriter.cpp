/*
    asyncpointcloudfilewriter.cpp (part of GNSS-Stylus)
    Copyright (C) 2025-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include <limits> // QtEndian (below) seems to need this... See https://bugreports.qt.io/browse/QTBUG-90395
#include <QtEndian>
#include "asynclidarscriptfilewriter.h"

AsyncLidarScriptFileWriter::AsyncLidarScriptFileWriter(const QString& fileName, const Params& params)
{
    this->fileName = fileName;
    this->params = params;
    fileHandleMutex.lock();     // This mutex is used to synchronize isOpenedSuccessfully, therefore lock it already
}

AsyncLidarScriptFileWriter::~AsyncLidarScriptFileWriter()
{
    if (terminateRequest == TR_NONE)
    {
        terminateRequest = TR_WRITEPENDINGDATA;
    }
    wait();
}

bool AsyncLidarScriptFileWriter::isOpenedSuccessfully(void)
{
    fileHandleMutex.lock();
    bool retval = fileOpenedSuccesfully;
    fileHandleMutex.unlock();
    return retval;
}

bool AsyncLidarScriptFileWriter::openFile(void)
{
    return file.open(QIODevice::WriteOnly);
}

void AsyncLidarScriptFileWriter::writeHeader(void)
{
    QByteArray eol = params.endOfLine;

    QByteArray dataToWrite = "ply" + eol;

    if (params.binary)
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

    // StartTime is also not known at this time (read the comment above how this is supposed to work)
    dataToWrite += "comment pad";
    startTimeFirstByte = dataToWrite.length();
    dataToWrite += "               " + eol + "comment starttime (N/A)";
    startTimeLastByte = dataToWrite.length();
    dataToWrite += eol;

    if (params.coordsFormat_HitPoint != Params::CF_NONE)
    {
        QByteArray coordFormatString = getCoordFormatString(params.coordsFormat_HitPoint);
        dataToWrite += "property " + coordFormatString + " x" + eol;
        dataToWrite += "property " + coordFormatString + " y" + eol;
        dataToWrite += "property " + coordFormatString + " z" + eol;
    }

    if (params.coordsFormat_Origin != Params::CF_NONE)
    {
        QByteArray coordFormatString = getCoordFormatString(params.coordsFormat_Origin);
        dataToWrite += "property " + coordFormatString + " origin_x" + eol;
        dataToWrite += "property " + coordFormatString + " origin_y" + eol;
        dataToWrite += "property " + coordFormatString + " origin_z" + eol;
    }

    if (params.timeFormat != Params::TF_NONE)
    {
        switch (params.timeFormat)
        {
        case Params::TF_NANOSECONDS:
            dataToWrite += "property uint time_ns_low" + eol;
            dataToWrite += "property uint time_ns_high" + eol;
            break;
        case Params::TF_MICROSECONDS_DELTA:
            dataToWrite += "property ushort time_us_delta" + eol;
            break;
        default:
            qFatal("Unimplemented time format.");
            break;
        }
    }

    dataToWrite += "end_header" + eol;

    file.write(dataToWrite);
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

void AsyncLidarScriptFileWriter::writePoint(const LidarScriptGeneratorThread::Output::Point * const point)
{
    // TODO: This is just a quick test:
    writeFloat(file, point->hitPoint.x());
    writeFloat(file, point->hitPoint.y());
    writeFloat(file, point->hitPoint.z());

#if 0
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
#endif
}

void AsyncLidarScriptFileWriter::writePoint_XYZ(const LidarScriptGeneratorThread::Output::Point * const point)
{
#if 0
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

void AsyncLidarScriptFileWriter::writePoint_PLY(const PointCloudGeneratorLidarThread::Output::Point * const point)
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
#endif
}

void AsyncLidarScriptFileWriter::finalizeFile(void)
{
    // Need to write number of vertices as one of the last steps as it isn't known before.
    numberOfPointsWrittenMutex.lock();
    QByteArray stringToWrite = QString::number(numberOfPointsWritten).toLatin1();
    numberOfPointsWrittenMutex.unlock();
    stringToWrite = params.endOfLine + "element vertex " + QString::number(numberOfPointsWritten).toLatin1();

    int numOfBytesToWrite = plyVertexCountLastByte - plyVertexCountFirstByte;
    while (stringToWrite.length() < numOfBytesToWrite)
    {
        stringToWrite = QByteArray(" ") + stringToWrite;
    }

    file.seek(plyVertexCountFirstByte);
    file.write(stringToWrite);
}


void AsyncLidarScriptFileWriter::run()
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

            LidarScriptGeneratorThread::Output outputData = outBuffer.take(nextChunkToWrite);
            outBufferMutex.unlock();

            errorMutex.lock();

            if (errorDetected)
            {
                // Error detected before this chunk. Just remove from the buffer.
                errorMutex.unlock();
                outBufferMutex.lock();
                nextChunkToWrite++;
                continue;
            }
            errorMutex.unlock();

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

            if (outputData.result == LidarScriptGeneratorThread::Output::R_ERROR)
            {
                // Flag the error so that subsequent chunks wont be processed
                errorMutex.lock();
                errorString = outputData.errorString;
                errorDetected = true;
                errorMutex.unlock();
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

void AsyncLidarScriptFileWriter::addPoints(const LidarScriptGeneratorThread::Output &out)
{
    outBufferMutex.lock();
    outBuffer.insert(out.workUnit.chunkIndex, out);
    outBufferMutex.unlock();

    waitCondition.wakeOne();
}

int AsyncLidarScriptFileWriter::getQueueLength(void)
{
    int retval;

    outBufferMutex.lock();
    retval = outBuffer.size();
    outBufferMutex.unlock();
    return retval;
}

bool AsyncLidarScriptFileWriter::getError(QString& errorString)
{
    errorMutex.lock();
    errorString = this->errorString;
    bool errorFlag = errorDetected;
    errorMutex.unlock();
    return errorFlag;
}

unsigned int AsyncLidarScriptFileWriter::getNumberOfPointsWritten(void)
{
    unsigned int retval;
    numberOfPointsWrittenMutex.lock();
    retval = numberOfPointsWritten;
    numberOfPointsWrittenMutex.unlock();
    return retval;
}

void AsyncLidarScriptFileWriter::requestTerminate(bool abandonPendingWrites)
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

QByteArray AsyncLidarScriptFileWriter::getCoordFormatString(const Params::CoordsFormat format)
{
    QByteArray coordsFormatString;

    switch (format)
    {
    case Params::CF_NONE:
        coordsFormatString = "N/A";
        break;
    case Params::CF_FLOAT:
        coordsFormatString = "float";
        break;
    case Params::CF_DOUBLE:
        coordsFormatString = "double";
        break;
    case Params::CF_SHORT:
        coordsFormatString = "short";
        break;
    default:
        qFatal("Unimplemented coordinate format.");
        break;
    }

    return coordsFormatString;
}
