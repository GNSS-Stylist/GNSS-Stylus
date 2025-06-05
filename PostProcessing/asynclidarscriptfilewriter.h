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
#include "PostProcessing/Lidar/lidarscriptgeneratorthread.h"
#include "PostProcessing/Lidar/PointFilter/postfilter.h"
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
            CF_SHORT,       // 16-bit millimetres (range -32768...32767, points outside will be discarded)
            CF_SHORT_DELTA, // 16-bit delta millimeters (range -16384...16383, points outside will be discarded)
        };

        enum TimeFormat
        {
            TF_NONE = 0,            // Do not write time
            TF_NANOSECONDS,         // 64-bit ns (in two 32-bit fields)
            TF_MICROSECONDS_DELTA,  // 16-bit microsecond delta time to the previous one
        };

        enum Qualityformat
        {
            QF_NONE = 0,            // Do not write quality
            QF_FLOAT,               // 32-bit float
            QF_UCHAR_SCALED,        // Unsigned char, float range 0...1 mapped to 0...255
            QF_UCHAR_RAW,           // Unsigned char, float range 0...255 mapped to 0...255
        };

        bool binary = true;
        CoordsFormat coordsFormat_HitPoint = CF_FLOAT;
        CoordsFormat coordsFormat_SourcePoint = CF_NONE;
        int numberOfDecimals_Hitpoints = 3;
        int numberOfDecimals_SourcePoint = 3;
        int numberOfDecimals_Quality = 2;
        TimeFormat timeFormat = TF_NONE;
        Qualityformat qualityFormat = QF_FLOAT;
        QByteArray endOfLine = "\n";
        PointFilter::PostFilter::Params postFilterParams;
        qint64 uptime_Min = 0;
        qint64 uptime_Max = 1e18;
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
    enum PointType
    {
        PT_SCANNING_NOT_ACTIVE = LidarScriptGeneratorThread::Output::PT_SCANNING_NOT_ACTIVE,
        PT_MISS = LidarScriptGeneratorThread::Output::PT_MISS,
        PT_HIT = LidarScriptGeneratorThread::Output::PT_HIT,

        PT_DELTA_EXCEEDED = 10,

        PT_UNDEFINED = 255,
    };

    // Flags for writePoint->flags
    static constexpr int WP_FORCE_WRITE = 1 << 0;
    static constexpr int WP_HIT_POINT_IN_MM = 1 << 1;
    static constexpr int WP_SOURCE_POINT_IN_MM = 1 << 2;

    QFile file;
    bool openFile(void);
    void writeHeader(void);
    bool writePoint(const LidarScriptGeneratorThread::Output::Point* const point, const PointType pointTypeOverride = PT_UNDEFINED, const unsigned char flags = 0);
//    void writePoint_XYZ(const LidarScriptGeneratorThread::Output::Point* const point);
//    void writePoint_PLY(const LidarScriptGeneratorThread::Output::Point* const point);
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
    bool startTimeDetected = false;
    qint64 startTime = 0;

    Eigen::Vector3i lastWrittenIntHitPoint = Eigen::Vector3i::Identity();
    Eigen::Vector3i lastWrittenIntSourcePoint = Eigen::Vector3i::Identity();
    qint64 lastWrittenTime_us = 0;

    std::shared_ptr<QVector<LidarScriptGeneratorThread::Output::Point> > prevPoints = nullptr;
    bool checkPrevPoint = false;
    LidarScriptGeneratorThread::Output::PointType prevPointType = LidarScriptGeneratorThread::Output::PT_UNDEFINED;

    enum TerminateRequest
    {
        TR_NONE = 0,
        TR_WRITEPENDINGDATA,
        TR_ABANDONPENDINGDATA,
    };
    volatile TerminateRequest terminateRequest = TR_NONE;
};

#endif // ASYNCLIDARSCRIPTFILEWRITER_H
