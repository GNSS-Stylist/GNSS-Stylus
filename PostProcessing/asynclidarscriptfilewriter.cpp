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

    QByteArray coordFormatString = getCoordFormatString(params.coordsFormat_HitPoint);

    switch (params.coordsFormat_HitPoint)
    {
    case Params::CF_NONE:
        break;
    case Params::CF_SHORT_DELTA:
        dataToWrite += "property " + coordFormatString + " dx" + eol;
        dataToWrite += "property " + coordFormatString + " dy" + eol;
        dataToWrite += "property " + coordFormatString + " dz" + eol;
        break;
    case Params::CF_FLOAT:
    case Params::CF_DOUBLE:
    case Params::CF_SHORT:
    default:
        dataToWrite += "property " + coordFormatString + " x" + eol;
        dataToWrite += "property " + coordFormatString + " y" + eol;
        dataToWrite += "property " + coordFormatString + " z" + eol;
        break;
    }

    coordFormatString = getCoordFormatString(params.coordsFormat_SourcePoint);

    switch (params.coordsFormat_SourcePoint)
    {
    case Params::CF_NONE:
        break;
    case Params::CF_SHORT_DELTA:
        dataToWrite += "property " + coordFormatString + " origin_dx" + eol;
        dataToWrite += "property " + coordFormatString + " origin_dy" + eol;
        dataToWrite += "property " + coordFormatString + " origin_dz" + eol;
        break;
    case Params::CF_FLOAT:
    case Params::CF_DOUBLE:
    case Params::CF_SHORT:
    default:
        dataToWrite += "property " + coordFormatString + " origin_x" + eol;
        dataToWrite += "property " + coordFormatString + " origin_y" + eol;
        dataToWrite += "property " + coordFormatString + " origin_z" + eol;
        break;
    }

    dataToWrite += "property uchar type" + eol;

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

    if (params.qualityFormat != Params::QF_NONE)
    {
        switch (params.qualityFormat)
        {
        case Params::QF_FLOAT:
            dataToWrite += "property float quality" + eol;
            break;
        case Params::QF_UCHAR_SCALED:
        case Params::QF_UCHAR_RAW:
            dataToWrite += "property uchar quality" + eol;
            break;
        default:
            qFatal("Unimplemented quality format.");
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

static inline void writeShort(QFile& file, const short src)
{
    char buf[2];
    qToLittleEndian(src, buf);
    file.write(buf, 2);
}

static inline void writeInt(QFile& file, const int src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

static inline void writeUChar(QFile& file, const unsigned char src)
{
    file.write((const char*)&src, 1);
}

bool AsyncLidarScriptFileWriter::writePoint(const LidarScriptGeneratorThread::Output::Point * const point, const PointType pointTypeOverride, const bool forceWrite)
{
    // Checking (and storing) ranges of coordinates first (need to discard the whole point if exceeded)

    Eigen::Vector3i intHitPoint;
    Eigen::Vector3i intSourcePoint;

    if ((params.coordsFormat_HitPoint == Params::CF_SHORT) || (params.coordsFormat_HitPoint == Params::CF_SHORT_DELTA))
    {
        intHitPoint = (1000 * point->hitPoint).cast<int>();

        if (params.coordsFormat_HitPoint == Params::CF_SHORT)
        {
            constexpr int valmin = -32768;
            constexpr int valmax = 32767;

            if ((intHitPoint.x() < valmin) || (intHitPoint.x() > valmax) || (intHitPoint.y() < valmin) || (intHitPoint.y() > valmax) || (intHitPoint.z() < valmin) || (intHitPoint.z() > valmax))
            {
                if (forceWrite)
                {
                    // Write zero coords if they are over range. ForceWrite is used only when writing deltas that are not directly used as coordinates.
                    intHitPoint.setZero();
                }
                else
                {
                    return false;
                }
            }
        }
    }

    if ((params.coordsFormat_SourcePoint == Params::CF_SHORT) || (params.coordsFormat_SourcePoint == Params::CF_SHORT_DELTA))
    {
        intSourcePoint = (1000 * point->sourcePoint).cast<int>();

        if (params.coordsFormat_SourcePoint == Params::CF_SHORT)
        {
            constexpr int valmin = -32768;
            constexpr int valmax = 32767;

            if ((intSourcePoint.x() < valmin) || (intSourcePoint.x() > valmax) || (intSourcePoint.y() < valmin) || (intSourcePoint.y() > valmax) || (intSourcePoint.z() < valmin) || (intSourcePoint.z() > valmax))
            {
                if (forceWrite)
                {
                    // Write zero coords if they are over range. ForceWrite is used only when writing deltas that are not directly used as coordinates.
                    intSourcePoint.setZero();
                }
                else
                {
                    return false;
                }
            }
        }
    }

    if (!startTimeDetected)
    {
        startTime = point->iTOW_ns;
        startTimeDetected = true;
        lastWrittenTime_us = point->iTOW_ns / 1000;
    }

    if ((params.timeFormat == Params::TF_MICROSECONDS_DELTA) || (params.coordsFormat_HitPoint == Params::CF_SHORT_DELTA) || (params.coordsFormat_SourcePoint == Params::CF_SHORT_DELTA))
    {
        // If using deltas, they may exceeed their maximum values.
        // Therefore calling this kind of recursively if needed.

        LidarScriptGeneratorThread::Output::Point fillPoint;

        bool deltaLimitExceeded;

        do
        {
            deltaLimitExceeded = false;

            if (params.timeFormat == Params::TF_MICROSECONDS_DELTA)
            {
                qint64 deltaTime_Unclamped = (point->iTOW_ns / 1000) - lastWrittenTime_us;
                if (deltaTime_Unclamped > 65535)
                {
    //                deltaTime = 65535;
                    deltaLimitExceeded = true;
                    fillPoint.iTOW_ns = (lastWrittenTime_us + 65535) * 1000; // lastWrittenTime will be updated in recursive call
                }
                else
                {
                    fillPoint.iTOW_ns = point->iTOW_ns;
                }
            }
            else
            {
                fillPoint.iTOW_ns = point->iTOW_ns;
            }

            if (params.coordsFormat_HitPoint == Params::CF_SHORT_DELTA)
            {
                Eigen::Vector3i deltaHitPoint = intHitPoint - lastWrittenHitPoint;
                Eigen::Vector3i deltaHitPoint_Clamped(std::clamp(deltaHitPoint.x(), -32768, 32767), std::clamp(deltaHitPoint.y(), -32768, 32767), std::clamp(deltaHitPoint.z(), -32768, 32767));

                if (deltaHitPoint_Clamped != deltaHitPoint)
                {
                    deltaLimitExceeded = true;
                    fillPoint.hitPoint = 0.001 * ((lastWrittenHitPoint + deltaHitPoint_Clamped).cast<double>()); // lastWrittenHitPoint will be updated in recursive call
                }
                else
                {
                    fillPoint.hitPoint = point->hitPoint;
                }
            }
            else
            {
                fillPoint.hitPoint = point->hitPoint;
            }

            if (params.coordsFormat_SourcePoint == Params::CF_SHORT_DELTA)
            {
                Eigen::Vector3i deltaSourcePoint = intSourcePoint - lastWrittenSourcePoint;
                Eigen::Vector3i deltaSourcePoint_Clamped(std::clamp(deltaSourcePoint.x(), -32768, 32767), std::clamp(deltaSourcePoint.y(), -32768, 32767), std::clamp(deltaSourcePoint.z(), -32768, 32767));

                if (deltaSourcePoint_Clamped != deltaSourcePoint)
                {
                    deltaLimitExceeded = true;
                    fillPoint.sourcePoint = 0.001* ((lastWrittenSourcePoint + deltaSourcePoint_Clamped).cast<double>()); // lastWrittenSourcePoint will be updated in recursive call
                }
                else
                {
                    fillPoint.sourcePoint = point->sourcePoint;
                }
            }
            else
            {
                fillPoint.sourcePoint = point->sourcePoint;
            }

            if (deltaLimitExceeded)
            {
                fillPoint.quality = point->quality;
                fillPoint.type = point->type;
                if (writePoint(&fillPoint, PT_DELTA_EXCEEDED, true))
                {
                    // This should not actually be reached (TODO: Remove after testing/debugging).
                    numberOfPointsWritten++;
                }
            }
        } while (deltaLimitExceeded);
    }

    if (params.binary)
    {
        switch (params.coordsFormat_HitPoint)
        {
        case Params::CF_NONE:
            break;
        case Params::CF_FLOAT:
            writeFloat(file, point->hitPoint.x());
            writeFloat(file, point->hitPoint.y());
            writeFloat(file, point->hitPoint.z());
            break;
        case Params::CF_DOUBLE:
            writeDouble(file, point->hitPoint.x());
            writeDouble(file, point->hitPoint.y());
            writeDouble(file, point->hitPoint.z());
            break;
        case Params::CF_SHORT:
            writeShort(file, intHitPoint.x());
            writeShort(file, intHitPoint.y());
            writeShort(file, intHitPoint.z());
            break;
        case Params::CF_SHORT_DELTA:
            Eigen::Vector3i delta = intHitPoint - lastWrittenHitPoint;
            writeShort(file, delta.x());
            writeShort(file, delta.y());
            writeShort(file, delta.z());
            lastWrittenHitPoint = intHitPoint;
            break;
        }

        switch (params.coordsFormat_SourcePoint)
        {
        case Params::CF_NONE:
            break;
        case Params::CF_FLOAT:
            writeFloat(file, point->sourcePoint.x());
            writeFloat(file, point->sourcePoint.y());
            writeFloat(file, point->sourcePoint.z());
            break;
        case Params::CF_DOUBLE:
            writeDouble(file, point->sourcePoint.x());
            writeDouble(file, point->sourcePoint.y());
            writeDouble(file, point->sourcePoint.z());
            break;
        case Params::CF_SHORT:
            writeShort(file, intSourcePoint.x());
            writeShort(file, intSourcePoint.y());
            writeShort(file, intSourcePoint.z());
            break;
        case Params::CF_SHORT_DELTA:
            Eigen::Vector3i delta = intSourcePoint - lastWrittenSourcePoint;
            writeShort(file, delta.x());
            writeShort(file, delta.y());
            writeShort(file, delta.z());
            lastWrittenSourcePoint = intSourcePoint;
            break;
        }

        if (pointTypeOverride != PT_UNDEFINED)
        {
            writeUChar(file, (unsigned char) pointTypeOverride);
        }
        else
        {
            writeUChar(file, (unsigned char) point->type);
        }

        switch (params.timeFormat)
        {
        case Params::TF_NONE:
            break;
        case Params::TF_NANOSECONDS:
            writeInt(file, point->iTOW_ns >> 32);
            writeInt(file, point->iTOW_ns);
            break;
        case Params::TF_MICROSECONDS_DELTA:
            qint64 deltaTime = (point->iTOW_ns / 1000) - lastWrittenTime_us;
//            Q_ASSERT((deltaTime < 65536) && (deltaTime > 0));
            writeShort(file, deltaTime);
            lastWrittenTime_us = lastWrittenTime_us + deltaTime;
            break;
        }

        switch (params.qualityFormat)
        {
        case Params::QF_NONE:
            break;
        case Params::QF_FLOAT:
            writeFloat(file, point->quality);
            break;
        case Params::QF_UCHAR_SCALED:
            writeUChar(file, (unsigned char)(std::clamp(int(point->quality * 255), 0, 255)));
            break;
        case Params::QF_UCHAR_RAW:
            writeUChar(file, (unsigned char)(point->quality));
            break;
        }
    }
    else
    {
        QString lineOut;

        switch (params.coordsFormat_HitPoint)
        {
        case Params::CF_NONE:
            break;
        case Params::CF_FLOAT:
        case Params::CF_DOUBLE:
            lineOut = QString::number(point->hitPoint.x(), 'f', params.numberOfDecimals_Hitpoints) +
                       " " + QString::number(point->hitPoint.y(), 'f', params.numberOfDecimals_Hitpoints) +
                       " " + QString::number(point->hitPoint.z(), 'f', params.numberOfDecimals_Hitpoints);
            break;
        case Params::CF_SHORT:
            lineOut = QString::number(intHitPoint.x()) +
                       " " + QString::number(intHitPoint.y()) +
                       " " + QString::number(intHitPoint.z());
            break;
        case Params::CF_SHORT_DELTA:
            Eigen::Vector3i delta = intHitPoint - lastWrittenHitPoint;
            lineOut = QString::number(delta.x()) +
                       " " + QString::number(delta.y()) +
                       " " + QString::number(delta.z());
            lastWrittenHitPoint = intHitPoint;
            break;
        }

        switch (params.coordsFormat_SourcePoint)
        {
        case Params::CF_NONE:
            break;
        case Params::CF_FLOAT:
        case Params::CF_DOUBLE:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(point->sourcePoint.x(), 'f', params.numberOfDecimals_SourcePoint) +
                       " " + QString::number(point->sourcePoint.y(), 'f', params.numberOfDecimals_SourcePoint) +
                       " " + QString::number(point->sourcePoint.z(), 'f', params.numberOfDecimals_SourcePoint);
            break;
        case Params::CF_SHORT:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(intSourcePoint.x()) +
                       " " + QString::number(intSourcePoint.y()) +
                       " " + QString::number(intSourcePoint.z());
            break;
        case Params::CF_SHORT_DELTA:
            if (lineOut.length() != 0) lineOut += " ";
            Eigen::Vector3i delta = intSourcePoint - lastWrittenSourcePoint;
            lineOut += QString::number(delta.x()) +
                       " " + QString::number(delta.y()) +
                       " " + QString::number(delta.z());
            lastWrittenSourcePoint = intSourcePoint;
            break;
        }

        if (lineOut.length() != 0) lineOut += " ";

        if (pointTypeOverride != PT_UNDEFINED)
        {
            lineOut += QString::number((unsigned char)pointTypeOverride);
        }
        else
        {
            lineOut += QString::number((unsigned char) point->type);
        }

        switch (params.timeFormat)
        {
        case Params::TF_NONE:
            break;
        case Params::TF_NANOSECONDS:
            lineOut += " " + QString::number((unsigned int)(point->iTOW_ns));
            lineOut += " " + QString::number((unsigned int)(point->iTOW_ns >> 32));
            break;
        case Params::TF_MICROSECONDS_DELTA:
            qint64 deltaTime = (point->iTOW_ns / 1000) - lastWrittenTime_us;
//            Q_ASSERT((deltaTime < 65536) && (deltaTime > 0));
            lineOut += " " + QString::number((unsigned int)(deltaTime));
            lastWrittenTime_us = lastWrittenTime_us + deltaTime;
            break;
        }

        switch (params.qualityFormat)
        {
        case Params::QF_NONE:
            break;
        case Params::QF_FLOAT:
            lineOut += " " + QString::number(point->quality, 'f', params.numberOfDecimals_Quality);
            break;
        case Params::QF_UCHAR_SCALED:
            lineOut += " " + QString::number((unsigned char)(std::clamp(int(point->quality * 255), 0, 255)));
            break;
        case Params::QF_UCHAR_RAW:
            lineOut += " " + QString::number((unsigned char)point->quality);
            break;
        }

        lineOut += params.endOfLine;
        QByteArray bytesToWrite = lineOut.toLatin1();
        file.write(bytesToWrite);
    }

    return true;
}


void AsyncLidarScriptFileWriter::finalizeFile(void)
{
    // Need to write number of vertices as one of the last steps as it isn't known before.
    numberOfPointsWrittenMutex.lock();
    unsigned int pointCount = numberOfPointsWritten;
    numberOfPointsWrittenMutex.unlock();
    QByteArray stringToWrite = params.endOfLine + "element vertex " + QString::number(pointCount).toLatin1();

    int numOfBytesToWrite = plyVertexCountLastByte - plyVertexCountFirstByte;
    while (stringToWrite.length() < numOfBytesToWrite)
    {
        stringToWrite = QByteArray(" ") + stringToWrite;
    }

    file.seek(plyVertexCountFirstByte);
    file.write(stringToWrite);

    // Need to write starttime as one of the last steps as it isn't known when opening the file.

    if (startTimeDetected)
    {
        stringToWrite = params.endOfLine + "comment starttime " + QString::number(startTime).toLatin1();

        int numOfBytesToWrite = startTimeLastByte - startTimeFirstByte;
        while (stringToWrite.length() < numOfBytesToWrite)
        {
            stringToWrite = QByteArray(" ") + stringToWrite;
        }

        file.seek(startTimeFirstByte);
        file.write(stringToWrite);
    }
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

    PointFilter::PostFilter postFilter(params.postFilterParams);

    checkPrevPoint = false;

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

            QVector<LidarScriptGeneratorThread::Output::Point>::const_iterator pointIter = outputData.points->constBegin();

            QVector<LidarScriptGeneratorThread::Output::Point>::const_iterator prevPointIter = pointIter; // AFAIK this should not need initialization here, but clang nags if it's not initialized...

            if (prevPoints != nullptr)
            {
                prevPointIter = &prevPoints->constLast();
            }
            else
            {
                checkPrevPoint = false;
            }

//            file.write(QString(QString(params.endOfLine + "Dbg. Chunk: " + QString::number(outputData.workUnit.chunkIndex) + ", points: " + QString::number(outputData.points->count()) + ", Duration (uptime, ms): " + QString::number(outputData.workUnit.endingUptime - outputData.workUnit.beginningUptime)) + ", Duration (itow, ns): " + QString::number(outputData.workUnit.endingITOWTime_ns - outputData.workUnit.beginningITOWTime_ns) + params.endOfLine).toLatin1());

            while (pointIter != outputData.points->constEnd())
            {
                if (terminateRequest == TR_ABANDONPENDINGDATA)
                {
                    break;
                }

                if (pointIter->type != prevPointType)
                {
                    // Always write point when type changes.

                    if (writePoint(pointIter))
                    {
                        numberOfPointsWritten++;
                    }

                    if (checkPrevPoint)
                    {
                        // Always write also the previous point when type changes (if not already written)
                        if (writePoint(prevPointIter))
                        {
                            numberOfPointsWritten++;
                        }
                    }

                    // Current point was already written so no need to check it on the next round.
                    checkPrevPoint = false;
                }
                else if (postFilter.filter(pointIter->hitPoint, pointIter->iTOW_ns))
                {
                    if (writePoint(pointIter))
                    {
                        numberOfPointsWritten++;
                    }

                    if (checkPrevPoint && (postFilter.checkBack(prevPointIter->hitPoint, prevPointIter->iTOW_ns)))
                    {
                        if (writePoint(prevPointIter))
                        {
                            numberOfPointsWritten++;
                        }
                    }

                    // Current point was already written so no need to check it on the next round.
                    checkPrevPoint = false;
                }
                else
                {
                    // As this point was not written, flag it to be checked on the next round (if it should be written then).
                    checkPrevPoint = true;
                }

                prevPointType = pointIter->type;
                prevPointIter = pointIter;

                pointIter++;
            }

            if (!outputData.points->isEmpty())
            {
                prevPoints = outputData.points;
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
    case Params::CF_SHORT_DELTA:
        coordsFormatString = "short";
        break;
    default:
        qFatal("Unimplemented coordinate format.");
        break;
    }

    return coordsFormatString;
}
