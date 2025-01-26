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
#ifndef ASYNCPOINTCLOUDFILEWRITER_H
#define ASYNCPOINTCLOUDFILEWRITER_H

#include <QThread>
#include <QString>
#include <QMutex>
#include <QQueue>
#include <QWaitCondition>
#include "PostProcessing/Lidar/pointcloudgeneratorlidarthread.h"

class AsyncPointCloudFileWriter : public QThread
{
public:
    class Params
    {
    public:
        enum FileFormat
        {
            FF_NONE = 0,    // Don't create/write files (can be used to check data validity without wearing the disk out)
            FF_XYZ,
            FF_PLY,
        };

        FileFormat fileFormat = FF_NONE;

        struct
        {
            bool includeNormals = true;
            bool normalLengthAsQuality = false;
            QByteArray endOfLine = "\n";
            int numberOfDecimals_Coords = 4;
            int numberOfDecimals_Normal = 4;
        } xyz;

        struct
        {
            bool includeNormals = true;
            bool normalLengthAsQuality = false;
            bool binary = false;
            bool includeQuality = false;
            bool doublePrecisionCoords = false;
            QByteArray endOfLine = "\r";
            int numberOfDecimals_Coords = 4;
            int numberOfDecimals_Normal = 3;
            int numberOfDecimals_Quality = 3;
        } ply;
    };

    AsyncPointCloudFileWriter(const QString &fileName, const Params &params);
    ~AsyncPointCloudFileWriter();
    QString getFileName(void) { return fileName; };
    bool isOpenedSuccessfully(void);
    void run() override;
    void addPoints(const PointCloudGeneratorLidarThread::Output& out);
    void requestTerminate(bool abandonPendingWrites = false);
    int getQueueLength(void);
    QMap<int, QString> getErrors(void);
    unsigned int getNumberOfPointsWritten(void);

private:
    QFile file;
    bool openFile(void);
    void writeHeader(void);
    void writePLYHeader(void);
    void writePoint(const PointCloudGeneratorLidarThread::Output::Point* const point);
    void writePoint_XYZ(const PointCloudGeneratorLidarThread::Output::Point* const point);
    void writePoint_PLY(const PointCloudGeneratorLidarThread::Output::Point* const point);
    void finalizeFile(void);
    void finalizeFile_PLY(void);

    Params params;
    int nextChunkToWrite = 0;

    QMap<int, QString> firstErrors;
    QMutex firstErrorsMutex;

    QString fileName;
    bool fileOpenedSuccesfully = false;
    QMutex fileHandleMutex;

    QMap<int, PointCloudGeneratorLidarThread::Output> outBuffer;
    QMutex outBufferMutex;
    QWaitCondition waitCondition;
    QMutex waitConditionMutex;

    unsigned int numberOfPointsWritten = 0;
    QMutex numberOfPointsWrittenMutex;

    unsigned int plyVertexCountFirstByte = 0;
    unsigned int plyVertexCountLastByte = 0;

    enum TerminateRequest
    {
        TR_NONE = 0,
        TR_WRITEPENDINGDATA,
        TR_ABANDONPENDINGDATA,
    };
    volatile TerminateRequest terminateRequest = TR_NONE;
};

#endif // ASYNCPOINTCLOUDFILEWRITER_H
