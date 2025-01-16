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
            FF_NONE,    // Don't create/write files (can be used to check data validity without wearing the disk out)
            FF_XYZ,
            FF_PLY,
        };

        FileFormat fileFormat = FF_NONE;

        // TODO: Add number of decimals, whether to include normals into xyz, doubles/floats/binary for ply, normal lengths as quality etc.
    };

    AsyncPointCloudFileWriter(const QString &fileName, const Params &params);
    ~AsyncPointCloudFileWriter();
    QString getFileName(void) { return fileName; };
    bool isOpenedSuccessfully(void);
//    void close(void);
    void run() override;
    void run_None(void);
    void run_XYZ(void);
    void addPoints(const PointCloudGeneratorLidarThread::Output& out);
    void requestTerminate(void) { terminateRequest = true; };
    int getQueueLength(void);

private:
    Params params;
    int nextChunkToWrite = 0;
    QString fileName;
//    QFile* file = nullptr;
//    QTextStream* textStream = nullptr;
    bool fileOpenedSuccesfully = false;
    QMutex fileHandleMutex;

    QMap<int, PointCloudGeneratorLidarThread::Output> outBuffer;
    QMutex outBufferMutex;
    QWaitCondition waitCondition;
    QMutex waitConditionMutex;

    volatile bool terminateRequest = false;
};

#endif // ASYNCPOINTCLOUDFILEWRITER_H
