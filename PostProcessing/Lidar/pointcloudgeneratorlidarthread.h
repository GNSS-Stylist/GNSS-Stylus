/*
    pointcloudgeneratorlidarthread.h (part of GNSS-Stylus)
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

#ifndef POINTCLOUDGENERATORLIDARTHREAD_H
#define POINTCLOUDGENERATORLIDARTHREAD_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include "PostProcessing/lointerpolator.h"
#include "PostProcessing/postprocessingform.h"
#include "../Lidar/PointFilter/expressionfilter_base.h"

class PointCloudGeneratorLidarThread : public QThread
{
    Q_OBJECT
public:

    class ConstData
    {
    public:
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        const QMap<LidarDevice, Eigen::Transform<double, 3, Eigen::Affine> >* transforms_AfterRotation = nullptr;
        const QMap<LidarDevice, std::shared_ptr<PointFilter::ExpressionFilter_Base>>* expressionMap_Source = nullptr;

        const PostProcessingForm::Rover* rovers = nullptr;
        const QVector<QString>* lidarFileNames = nullptr;
        const QMap<qint64, UBXMessage_RELPOSNED::ITOW>* averagedSync;

        const LOSolver* loSolver_Base;

        // TODO: Remove boundingSphere
        const Eigen::Vector3d* boundingSphere_Center;
        double boundingSphere_Radius = 1e9;

        struct
        {
//            bool normalLengthsAsQuality = false;
            int timeShift = 0;
            const QMap<qint64, PostProcessingForm::LidarRound>* rounds = nullptr;
            const RPLidarPlausibilityFilter::Settings* filteringSettings = nullptr;
            const Eigen::Transform<double, 3, Eigen::Affine>* transform_BeforeRotation = nullptr;
        } rpLidar;

        struct
        {
            const QMultiMap<qint64, PostProcessingForm::Mid360Datagram>* datagrams = nullptr;
        } mid360;
    };

    class WorkUnit
    {
    public:
        bool valid = false;
        QString sourceFileName = "";
        QString outFileName = "";
        int pointSetIndex = -1;
        int chunkIndex = -1;
        int beginningTagLine = -1;
        int endingTagLine = -1;
        qint64 beginningUptime = -1;
        qint64 endingUptime = -1;
    };

    class Output
    {
    public:
        class Point
        {
        public:
            Eigen::Vector3d hitPoint;
            Eigen::Vector3d normal;
            float quality;
        };

        enum Result
        {
            R_OK,
            R_ERROR,
//            R_PROCESSING_ABORTED,
//            R_THREAD_TERMINATED,
        };

        Result result = R_OK;
        QString errorString;

        WorkUnit workUnit;
        std::shared_ptr<QVector<Point> > points;
    };

    enum State
    {
        S_INITIALIZING, // Not yet processing work units
        S_PROCESSING,   // Processing work unit
        S_DONE,         // Thread exited (no more work unit to process)
    };

    explicit PointCloudGeneratorLidarThread(const ConstData &cData, std::function< WorkUnit(void) > workUnitGetter, std::function<void (const Output &)> workUnitProcessor);
    void run() override;

    State getState(float* progressFraction = nullptr);
    void requestTerminate(void) { terminateRequest = true; }

private:
    volatile bool terminateRequest = false;
    ConstData constData;
    QMap<LidarDevice, std::shared_ptr<PointFilter::ExpressionFilter_Base> > expressionMap_Local;

    bool generatePointCloudPointSet(LOInterpolator &loInterpolator);

    WorkUnit workUnitInProgress;
    std::function<WorkUnit ()> getWorkUnit;
    std::function<void(const Output&)> workUnitProcessed;
    State state = S_INITIALIZING;
    QMutex stateMutex;
    float progressFraction = 0;
    QMutex progressFractionMutex;
};

#endif // POINTCLOUDGENERATORLIDARTHREAD_H
