/*
    pointcloudgeneratorlidar.h (part of GNSS-Stylus)
    Copyright (C) 2019-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef POINTCLOUDGENERATORLIDAR_H
#define POINTCLOUDGENERATORLIDAR_H

#include <QQueue>

#include "../postprocessingform.h"
#include "pointcloudgeneratorlidarthread.h"
#include "../asyncpointcloudfilewriter.h"

namespace Lidar
{

class PointCloudGenerator : public QObject
{
    Q_OBJECT

public:
    class Params
    {
    public:
        QDir directory;
        const QMap<qint64, PostProcessingForm::ScanningState>* scanningStateMap = nullptr;
        bool separateFilesForSubScans = false;

        const QMultiMap<qint64, PostProcessingForm::Tag>* tags = nullptr;

        int maxWorkUnitDuration = 1000;
        int numOfWorkerThreads = 1;

        PointCloudGeneratorLidarThread::ConstData threadConstData;
        AsyncPointCloudFileWriter::Params fileParams;
    };

    void generatePointClouds(const Params& params);

private:

    std::shared_ptr<AsyncPointCloudFileWriter> createNewOutFile(const QString fileName, const AsyncPointCloudFileWriter::Params &params, const PostProcessingForm::Tag& currentTag, const qint64 uptime);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

};

}; // namespace Lidar

#endif // POINTCLOUDGENERATORLIDAR_H
