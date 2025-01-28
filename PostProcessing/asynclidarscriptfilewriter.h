/*
    asyncpointcloudfilewriter.h (part of GNSS-Stylus)
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
#ifndef ASYNCLIDARSCRIPTFILEWRITER_H
#define ASYNCLIDARSCRIPTFILEWRITER_H

#include <QThread>
#include <QString>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>
//#include "PostProcessing/Lidar/pointcloudgeneratorlidarthread.h"
#include "PostProcessing/Lidar/lidarscriptgeneratorthread.h"
#include "Eigen/Geometry"

class AsyncLidarScriptFileWriter : public QThread
{
public:
    class Params
    {
    public:
        enum CoordsFormat
        {
            CF_NONE = 0,    // Do not write these coords
            CF_FLOAT,       // Metres
            CF_DOUBLE,      // Metres
            CF_SHORT,       // 16-bit millimetres (note range -32768...32767, points outside will be discarded)
        };

        enum TimeFormat
        {
            TF_NONE = 0,            // Do not write time
            TF_NANOSECONDS,         // 64-bit ns (in two 32-bit fields)
            TF_MICROSECONDS_DELTA,  // 16-bit microsecond delta time to the previous one
        };

        CoordsFormat coordsFormat_HitPoint = CF_FLOAT;
        CoordsFormat coordsFormat_Origin = CF_NONE;
        TimeFormat timeFormat = TF_NONE;
        bool binary = true;
        QByteArray endOfLine = "\n";
    };

    AsyncLidarScriptFileWriter(const QString &fileName, const Params &params);
    ~AsyncLidarScriptFileWriter();
    QString getFileName(void) { return fileName; };
    bool isOpenedSuccessfully(void);
    void run() override;
    void addPoints(const LidarScriptGeneratorThread::Output& out);
    void requestTerminate(bool abandonPendingWrites = false);
    int getQueueLength(void);
    bool getError(QString& errorString);
    unsigned int getNumberOfPointsWritten(void);

private:
    QFile file;
    bool openFile(void);
    void writeHeader(void);
    void writePoint(const LidarScriptGeneratorThread::Output::Point* const point);
    void writePoint_XYZ(const LidarScriptGeneratorThread::Output::Point* const point);
    void writePoint_PLY(const LidarScriptGeneratorThread::Output::Point* const point);
    void finalizeFile(void);
    QByteArray getCoordFormatString(const Params::CoordsFormat format);

    Params params;
    int nextChunkToWrite = 0;

    bool errorDetected = false;
    QString errorString;
    QMutex errorMutex;

    QString fileName;
    bool fileOpenedSuccesfully = false;
    QMutex fileHandleMutex;

    QMap<int, LidarScriptGeneratorThread::Output> outBuffer;
    QMutex outBufferMutex;
    QWaitCondition waitCondition;
    QMutex waitConditionMutex;

    unsigned int numberOfPointsWritten = 0;
    QMutex numberOfPointsWrittenMutex;

    unsigned int plyVertexCountFirstByte = 0;
    unsigned int plyVertexCountLastByte = 0;

    unsigned int startTimeFirstByte = 0;
    unsigned int startTimeLastByte = 0;

    enum TerminateRequest
    {
        TR_NONE = 0,
        TR_WRITEPENDINGDATA,
        TR_ABANDONPENDINGDATA,
    };
    volatile TerminateRequest terminateRequest = TR_NONE;
};

#endif // ASYNCLIDARSCRIPTFILEWRITER_H
