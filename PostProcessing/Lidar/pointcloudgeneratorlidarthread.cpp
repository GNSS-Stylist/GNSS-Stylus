/*
    pointcloudgeneratorlidarthread.cpp (part of GNSS-Stylus)
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

#include "pointcloudgeneratorlidarthread.h"
#include "../Lidar/PointFilter/expressionfilter_mid360.h"
#include "../Lidar/PointFilter/expressionfilter_rplidar.h"

//#define MID360_HARDCODED_FILTERING
/* Use the define above if compiling special version with hardcoded filtering for Mid-360.
This can be a _lot_ faster, in quick tests calculation took below 1 s instead of 17 s
when using code performing the following expression:
mid360 192.168.40.100
{
        not
        (
        ((lidar.mid360.properties & 0x3f) != 0)
        ||
        (
        (lidar.coord.x < -0.45) && (lidar.coord.x > -3.5) &&
        (abs(lidar.coord.y / lidar.coord.x) < 1.0) &&
        (abs(lidar.coord.z / lidar.coord.x) < (3.0 / 1.2))
        )
        ||
        (
        (lidar.coord.x < 0.0) && (lidar.coord.x > -3.5) &&
        (lidar.coord.z < 0.0) && (abs(lidar.coord.y) < 0.1)
        )
        ||
        (lidar.distance < 0.15)
        )
}
{ 1 }

Writing to files took a lot longer, but as it is buffered, it can be separated from processing time.
Device: Laptop with 32Gb of RAM and Intel© Core™ i7-10750H CPU @ 2.60GHz using 12 threads.

There were some tiny (1 mm) rounding differences is point cloud files, though
(mostly not visible, although differences also caused some more points to appear in "hardcoded files")

Random thought: TinyExpr++ does not short circuit logical operations (for example it always calculates all orred "sub-expressions" in the expression above),
which may contribute to the huge performance difference as the hardcoded-version just continues when ((properties & 0x3f) != 0).
Could be wortwhile to investigate if TinyExpr++ could be modified to support short-circuiting.
*/

PointCloudGeneratorLidarThread::PointCloudGeneratorLidarThread(const ConstData& cData, std::function<WorkUnit ()> workUnitGetter, std::function<void(const Output&)> workUnitProcessor)
{
    constData = cData;
    getWorkUnit = workUnitGetter;
    workUnitProcessed = workUnitProcessor;
}

void PointCloudGeneratorLidarThread::run()
{
    // Need to make local copies of expressions
    QMap<LidarDevice, std::shared_ptr<PointFilter::ExpressionFilter_Base> >::const_iterator item = constData.expressionMap_Source->constBegin();
    while (item != constData.expressionMap_Source->constEnd())
    {
        // This way to copy these feels a bit clumsy (works, though). Maybe there is a neater way?
        PointFilter::ExpressionFilter_Mid360* mid360 = dynamic_cast<PointFilter::ExpressionFilter_Mid360*>(item.value().get());
        PointFilter::ExpressionFilter_RPLidar* rpLidar = dynamic_cast<PointFilter::ExpressionFilter_RPLidar*>(item.value().get());

        if (mid360)
        {
            expressionMap_Local.insert(item.key(), std::make_shared<PointFilter::ExpressionFilter_Mid360>(PointFilter::ExpressionFilter_Mid360(*mid360)));
        }
        else if (rpLidar)
        {
            expressionMap_Local.insert(item.key(), std::make_shared<PointFilter::ExpressionFilter_RPLidar>(PointFilter::ExpressionFilter_RPLidar(*rpLidar)));
        }
        else
        {
            qFatal("Unsupported device type.");
        }

        item++;
    }

    const QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED> *relposnedMessages[3];
    for (int i = 0; i < 3; i++)
    {
        relposnedMessages[i] = &constData.rovers[i].relposnedMessages;
    }
    LOInterpolator loInterpolator(relposnedMessages);
    loInterpolator.loSolver = *constData.loSolver_Base;

    workUnitInProgress = getWorkUnit();

    while ((workUnitInProgress.valid) && (!terminateRequest))
    {
        generatePointCloudPointSet(loInterpolator);
        workUnitInProgress = getWorkUnit();
    }
}


bool PointCloudGeneratorLidarThread::generatePointCloudPointSet(LOInterpolator& loInterpolator)
{
    Output output;

    output.workUnit = workUnitInProgress;
    output.points = std::make_shared<QVector<Output::Point> >();
    QVector<Output::Point>* points = output.points.get();

    QMap<qint64, PostProcessingForm::LidarRound>::const_iterator rpLidarIter = constData.rpLidar.rounds->upperBound(workUnitInProgress.beginningUptime);

    // As RPLidar rounds are "mapped" according to their arriving (=end) timestamps,
    // roll here to the first one with a bigger starting timestamp
    // to prevent taking "past" measurements into account

    while ((rpLidarIter != constData.rpLidar.rounds->end()) && (rpLidarIter.value().startTime < workUnitInProgress.beginningUptime))
    {
        rpLidarIter++;
    }

    QVector<RPLidarPlausibilityFilter::FilteredItem> rpLidarFilteredItems;
    rpLidarFilteredItems.reserve(10000);

    RPLidarPlausibilityFilter rpLidarPlausibilityFilter;

    LidarDevice rpLidarDevice(LidarDevice::DT_RPLIDAR);
    Q_ASSERT(constData.transforms_AfterRotation->contains(rpLidarDevice));

    Eigen::Transform<double, 3, Eigen::Affine> rpLidarTransform_BeforeRotation = *constData.rpLidar.transform_BeforeRotation;
    auto rpLidarTransform_AfterRotation = constData.transforms_AfterRotation->value(rpLidarDevice);

    rpLidarPlausibilityFilter.setSettings(*constData.rpLidar.filteringSettings);

    Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

    for (auto exprIter = expressionMap_Local.begin(); exprIter != expressionMap_Local.end(); exprIter++)
    {
        exprIter.value()->initBuffer();
    }

    while ((rpLidarIter != constData.rpLidar.rounds->end()) && (rpLidarIter.value().startTime < workUnitInProgress.endingUptime))
    {
        rpLidarPlausibilityFilter.filter(rpLidarIter.value().distanceItems, rpLidarFilteredItems);

        // Q_ASSERT(lidarIter.value().distanceItems.count() == filteredItems.count());

        const PostProcessingForm::LidarRound& round = rpLidarIter.value();

        for (int i = 0; i < rpLidarFilteredItems.count(); i++)
        {
            const RPLidarPlausibilityFilter::FilteredItem& currentItem = rpLidarFilteredItems[i];

            if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
            {
                // Rover coordinates interpolated according to distance timestamps.

                qint64 itemUptime = round.startTime + (round.endTime - round.startTime) * i / rpLidarIter.value().distanceItems.count();
                UBXMessage_RELPOSNED interpolated_Rovers[3];

                qint64 roverUptime = itemUptime + constData.rpLidar.timeShift;

                // TODO: Implement "uptime cache" (only calculate transform when uptime changes)

                try
                {
                    loInterpolator.getInterpolatedLocationOrientationTransformMatrix_Uptime(roverUptime, *constData.averagedSync, transform_LoSolver);
                }
                catch (QString& stringThrown)
                {
                    Q_ASSERT(constData.lidarFileNames);
                    Q_ASSERT(constData.lidarFileNames->size() > rpLidarIter.value().fileNameIndex);

                    output.errorString = "File \"" + constData.lidarFileNames->at(rpLidarIter.value().fileNameIndex) + "\", chunk index " +
                                        QString::number(rpLidarIter.value().chunkIndex)+
                                        " (RPLidar), uptime " + QString::number(rpLidarIter.key()) +
                                        ": " + stringThrown + " Skipped the rest of this set of points " +
                                        "between tags in lines " + QString::number(workUnitInProgress.beginningTagLine) + " and " +
                                        QString::number(workUnitInProgress.endingTagLine) +
                                        " in file \"" + workUnitInProgress.sourceFileName + "\".";

                    output.result = Output::R_ERROR;
                    workUnitProcessed(output);

                    return(false);
                }

                Eigen::Transform<double, 3, Eigen::Affine> transform_LaserRotation;
                transform_LaserRotation = Eigen::AngleAxisd(currentItem.item.angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = *constData.transform_NEDToXYZ * (transform_LoSolver * (rpLidarTransform_AfterRotation * (transform_LaserRotation * (rpLidarTransform_BeforeRotation * Eigen::Vector3d::Zero()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserOriginBeforeRotation = transform_BeforeRotation * Eigen::Vector3d::Zero();
                Eigen::Vector3d laserOriginAfterRotation = transform_LaserRotation * laserOriginBeforeRotation;
                Eigen::Vector3d laserOriginAfterPostRotationTransform = transform_AfterRotation * laserOriginAfterRotation;
                Eigen::Vector3d laserOriginAfterLOSolverTransform = transform_LoSolver * laserOriginAfterPostRotationTransform;
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserOriginAfterLOSolverTransform;
                */

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (rpLidarTransform_AfterRotation * (transform_LaserRotation * (rpLidarTransform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserVectorBeforeRotation = transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX());
                Eigen::Vector3d laserVectorAfterRotation = transform_LaserRotation * laserVectorBeforeRotation;
                Eigen::Vector3d laserVectorAfterPostRotationTransform = transform_AfterRotation * laserVectorAfterRotation;
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * laserVectorAfterPostRotationTransform;
                */

                if ((laserHitPosAfterLOSolverTransform - *constData.boundingSphere_Center).norm() <= constData.boundingSphere_Radius)
                {
                    Eigen::Vector3d laserHitPosAfterLOSolverTransformXYZ = *constData.transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;

                    Eigen::Vector3d normal = (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).normalized();

                    if (constData.rpLidar.normalLengthsAsQuality)
                    {
                        normal = (1. / (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).norm()) * normal;
                    }

                    // TODO: Add data to output
/*
                    QString lineOut;
                    if (constData.includeNormals)
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                  "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                  "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4) +
                                  "\t" + QString::number(normal(0), 'f', 4) +
                                  "\t" + QString::number(normal(1), 'f', 4) +
                                  "\t" + QString::number(normal(2), 'f', 4);
                    }
                    else
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                  "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                  "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4);
                    }

                    outStream->operator<<(lineOut + "\n");
*/
                }
            }
        }

        rpLidarIter++;
    }

    // Just rely on the timestamps of the datagrams to make splitting the data into chunks (for threads to digest) easier.
    // The uptime range will be from beginningUptime (inclusive) to endingUptime (exclusive).
    // Filtering needs some adjustments due to buffering/delay,
    // this is done now by feeding bufferLength (now 16) samples from the datagram preceding the one found using the timestamp.
    // This adds a tiny time inaccuracy (8/200000s ("delay" of 8 samples)), so doesn't matter.

    QMultiMap<qint64, PostProcessingForm::Mid360Datagram>::const_iterator mid360MultiMapIter = constData.mid360.datagrams->lowerBound(workUnitInProgress.beginningUptime);

    UBXMessage_RELPOSNED::ITOW lastInterpolatedITOWUptime_ms = -1;

    while ((mid360MultiMapIter != constData.mid360.datagrams->end()) && (mid360MultiMapIter.key() < workUnitInProgress.endingUptime))
    {
        qint64 uptime = mid360MultiMapIter.key();
        QList<PostProcessingForm::Mid360Datagram> datagrams = constData.mid360.datagrams->values(uptime);

        for (int datagramIndex = datagrams.size() - 1; datagramIndex >= 0; datagramIndex--)
        {
            const PostProcessingForm::Mid360Datagram mid360Datagram = datagrams[datagramIndex];
            LivoxMid360::PointCloudAndIMUDataHeader header(mid360Datagram.datagram);

            if ((header.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) || (
                    (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL) &&
                    (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
                    (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT)))
            {
                continue;
            }
            LivoxMid360::PointCloudData pcData(header, mid360Datagram.datagram);

            if ((pcData.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) ||
                (pcData.time_type != LivoxMid360::PointCloudAndIMUDataHeader::TimeSyncType::TIME_SYNC_GPS))
            {
                continue;
            }

            quint64 pointStartTime_ns = pcData.timestamp;
            quint64 pointChunkTime_ns = quint64(pcData.time_interval) * 100;

            quint32 ipAddress = mid360Datagram.datagram.senderAddress().toIPv4Address();
            LidarDevice device(LidarDevice::DT_LIVOX_MID360, ipAddress);

            if (!expressionMap_Local.contains(device))
            {
                output.errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram.fileNameIndex) + "\", chunk index " +
                                    QString::number(mid360Datagram.chunkIndex)+
                                    " (Mid-360), IP: " + mid360Datagram.datagram.senderAddress().toString() +
                                    ", uptime " + QString::number(uptime) +
                                    ", ITOW " + QString::number(pointStartTime_ns / 1000000) +
                                    ": Filter expression not defined. Skipped the rest of this set of points " +
                                    "between tags in lines " + QString::number(workUnitInProgress.beginningTagLine) + " and " +
                                    QString::number(workUnitInProgress.endingTagLine) +
                                    " in file \"" + workUnitInProgress.sourceFileName + "\".";

                output.result = Output::R_ERROR;
                workUnitProcessed(output);

                return(false);
            }

#if !defined(MID360_HARDCODED_FILTERING)
            PointFilter::ExpressionFilter_Mid360* exprFilter = dynamic_cast<PointFilter::ExpressionFilter_Mid360*> (expressionMap_Local.value(device).get());

            if ((exprFilter->getNumOfAddedPoints() < exprFilter->bufferLength) && (mid360MultiMapIter != constData.mid360.datagrams->begin()))
            {
                // To allow chunks to be split for different threads to handle, the starting and ending times of subsequent chunks must match exactly.
                // Therefore "prefilling" the filter with the data (last samples) from the previous datagram for this device.
                // This code is quite similar to the "real" filtering code later. Will not combine these since the "real" filtering should be as fast as possible.
                // (This part is only ran once per "point set", so doesn't need to be very optimized).

                auto backIter = constData.mid360.datagrams->lowerBound(workUnitInProgress.beginningUptime);

                while ((backIter != constData.mid360.datagrams->begin()) && (exprFilter->getNumOfAddedPoints() < exprFilter->bufferLength))
                {
                    backIter--;

                    qint64 uptime_Back = backIter.key();
                    QList<PostProcessingForm::Mid360Datagram> datagrams_Back = constData.mid360.datagrams->values(uptime_Back);

                    for (int datagramIndex_Back = 0; datagramIndex_Back < datagrams_Back.size(); datagramIndex_Back++)
                    {
                        const PostProcessingForm::Mid360Datagram& mid360Datagram_Back = datagrams_Back[datagramIndex_Back];

                        quint32 ipAddress_Back = mid360Datagram_Back.datagram.senderAddress().toIPv4Address();

                        if (ipAddress_Back != ipAddress)
                        {
                            continue;
                        }

                        LivoxMid360::PointCloudAndIMUDataHeader header_Back(mid360Datagram_Back.datagram);

                        if ((header_Back.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) || (
                                (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL) &&
                                (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
                                (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT)))
                        {
                            continue;
                        }

                        LivoxMid360::PointCloudData pcData_Back(header_Back, mid360Datagram_Back.datagram);

                        if ((pcData_Back.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) ||
                            (pcData_Back.time_type != LivoxMid360::PointCloudAndIMUDataHeader::TimeSyncType::TIME_SYNC_GPS))
                        {
                            continue;
                        }

                        quint16 pointNum_Back = pcData_Back.dot_num;
                        quint64 pointStartTime_ns_Back = pcData_Back.timestamp;
                        quint64 pointChunkTime_ns_Back = quint64(pcData_Back.time_interval) * 100;

                        for (int i = pointNum_Back - exprFilter->bufferLength; i < pointNum_Back; i++)
                        {
                            LivoxMid360::PointCloudData::Point* currentPoint = &pcData_Back.points[i];
                            UBXMessage_RELPOSNED::ITOW pointITOWUptime_ms = (pointStartTime_ns_Back + ((pointChunkTime_ns_Back * i) / (pointNum_Back - 1))) / 1000000;

                            if (pointITOWUptime_ms != lastInterpolatedITOWUptime_ms)
                            {
                                try
                                {
                                    loInterpolator.getInterpolatedLocationOrientationTransformMatrix_ITOW(pointITOWUptime_ms, transform_LoSolver);
                                }
                                catch (QString& stringThrown)
                                {
                                    Q_ASSERT(constData.lidarFileNames);
                                    Q_ASSERT(constData.lidarFileNames->size() >  mid360Datagram_Back.fileNameIndex);

                                    output.errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram_Back.fileNameIndex) + "\", chunk index " +
                                                        QString::number(mid360Datagram_Back.chunkIndex)+
                                                        " (Mid-360), IP: " + mid360Datagram_Back.datagram.senderAddress().toString() +
                                                        ", uptime " + QString::number(uptime_Back) +
                                                        ", ITOW " + QString::number(pointITOWUptime_ms) +
                                                        ": " + stringThrown + " Skipped the rest of this set of points " +
                                                        "between tags in lines " + QString::number(workUnitInProgress.beginningTagLine) + " and " +
                                                        QString::number(workUnitInProgress.endingTagLine) +
                                                        " in file \"" + workUnitInProgress.sourceFileName + "\".";

                                    output.result = Output::R_ERROR;
                                    workUnitProcessed(output);

                                    return(false);
                                }

                                exprFilter->setTransform_RigToNED(transform_LoSolver);

                                lastInterpolatedITOWUptime_ms = pointITOWUptime_ms;
                            }

                            exprFilter->addPoint(*currentPoint, pointITOWUptime_ms);
                        }
                        break;
                    }
                }
            }
#endif
            quint16 pointNum = pcData.dot_num;

            if (!constData.transforms_AfterRotation->contains(device))
            {
                output.errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram.fileNameIndex) + "\", chunk index " +
                                    QString::number(mid360Datagram.chunkIndex)+
                                    " (Mid-360), IP: " + mid360Datagram.datagram.senderAddress().toString() +
                                    ", uptime " + QString::number(uptime) +
                                    ", ITOW " + QString::number(pointStartTime_ns / 1000000) +
                                    ": Operation after rotation not defined. Skipped the rest of this set of points " +
                                    "between tags in lines " + QString::number(workUnitInProgress.beginningTagLine) + " and " +
                                    QString::number(workUnitInProgress.endingTagLine) +
                                    " in file \"" + workUnitInProgress.sourceFileName + "\".";

                output.result = Output::R_ERROR;
                workUnitProcessed(output);

                return(false);
            }

            auto transform_AfterRotation = constData.transforms_AfterRotation->value(device);

            //        Eigen::Transform<double, 3, Eigen::Affine> transform_BeforeRotation = *params.rpLidar.transform_BeforeRotation;

            for (int i = 0; i < pointNum; i++)
            {
                LivoxMid360::PointCloudData::Point* currentPoint = &pcData.points[i];

#if defined(MID360_HARDCODED_FILTERING)

                if (currentPoint->properties & 0x3f)
                {
                    // Discard all points whose confidence level is not "normal" (read Mid-360 docs)
                    continue;
                }

                // Filter for now just using hard-coded operator/rig-discarding limits.
                // TODO: Add configurable params/zones.

                if ((currentPoint->x < -0.45) && (currentPoint->x > -3.5) &&    // Only take "farther" part of the rig into account (to be able to scan a bit "behind" the lidar unit)
                    (fabs(currentPoint->y / currentPoint->x) < (1.0)) &&        // 45-deg "fan" up/down (in rig coords)
                    (fabs(currentPoint->z / currentPoint->x) < (3.0 / 1.2)))    // "fan" left/right (in rig coords)
                {
                    continue;
                }

                // Filter out the tube (a bit lossy filtering here...)
                if ((currentPoint->x < 0.0) && (currentPoint->x > -3.5) &&
                    (currentPoint->z < 0.0) && fabs(currentPoint->y) < 0.1)
                {
                    continue;
                }

                double distance = sqrt(currentPoint->x * currentPoint->x + currentPoint->y * currentPoint->y + currentPoint->z * currentPoint->z);

                if (distance < 0.15)
                {
                    // Discard points too close to lidar's origin
                    continue;
                }
#endif

                UBXMessage_RELPOSNED::ITOW pointITOWUptime_ms = (pointStartTime_ns + ((pointChunkTime_ns * i) / (pointNum - 1))) / 1000000;

                if (pointITOWUptime_ms != lastInterpolatedITOWUptime_ms)
                {
                    try
                    {
                        loInterpolator.getInterpolatedLocationOrientationTransformMatrix_ITOW(pointITOWUptime_ms, transform_LoSolver);
                    }
                    catch (QString& stringThrown)
                    {
                        Q_ASSERT(constData.lidarFileNames);
                        Q_ASSERT(constData.lidarFileNames->size() > mid360Datagram.fileNameIndex);

                        output.errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram.fileNameIndex) + "\", chunk index " +
                                            QString::number(mid360Datagram.chunkIndex)+
                                            " (Mid-360), IP: " + mid360Datagram.datagram.senderAddress().toString() +
                                            ", uptime " + QString::number(uptime) +
                                            ", ITOW " + QString::number(pointITOWUptime_ms) +
                                            ": " + stringThrown + " Skipped the rest of this set of points " +
                                            "between tags in lines " + QString::number(workUnitInProgress.beginningTagLine) + " and " +
                                            QString::number(workUnitInProgress.endingTagLine) +
                                            " in file \"" + workUnitInProgress.sourceFileName + "\".";

                        output.result = Output::R_ERROR;
                        workUnitProcessed(output);

                        return(false);
                    }

#if !defined(MID360_HARDCODED_FILTERING)
                    exprFilter->setTransform_RigToNED(transform_LoSolver);
#endif
                    lastInterpolatedITOWUptime_ms = pointITOWUptime_ms;
                }

#if !defined(MID360_HARDCODED_FILTERING)
                exprFilter->addPoint(*currentPoint, pointITOWUptime_ms);

                PointFilter::ExpressionFilter_Mid360::OutItem exprOutItem;

                if(!(exprFilter->getFilteredPoint(exprOutItem)))
                {
                    continue;
                }
                if (!exprOutItem.valid)
                {
                    continue;
                }
                if (exprOutItem.filterResult != 1.0)
                {
                    continue;
                }
#endif

                Eigen::Vector3d lidarPoint(currentPoint->x, currentPoint->y, currentPoint->z);

                // Lot of parentheses here to keep all calculations as matrix * vector
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = *constData.transform_NEDToXYZ * (transform_LoSolver * (transform_AfterRotation * Eigen::Vector3d::Zero()));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                    Eigen::Vector3d laserOriginBeforeRotation = transform_BeforeRotation * Eigen::Vector3d::Zero();
                    Eigen::Vector3d laserOriginAfterRotation = transform_LaserRotation * laserOriginBeforeRotation;
                    Eigen::Vector3d laserOriginAfterPostRotationTransform = transform_AfterRotation * laserOriginAfterRotation;
                    Eigen::Vector3d laserOriginAfterLOSolverTransform = transform_LoSolver * laserOriginAfterPostRotationTransform;
                    Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserOriginAfterLOSolverTransform;
                    */

#if defined(MID360_HARDCODED_FILTERING)
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (transform_AfterRotation * lidarPoint);
#else
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = exprOutItem.coords;
#endif
                if ((laserHitPosAfterLOSolverTransform - *constData.boundingSphere_Center).norm() <= constData.boundingSphere_Radius)
                {
                    Output::Point newPoint;

                    newPoint.hitPoint = *constData.transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;
                    newPoint.normal = (laserOriginAfterLOSolverTransformXYZ - newPoint.hitPoint).normalized();

                    // TODO: Own quality calculation for Mid-360
                    if (constData.rpLidar.normalLengthsAsQuality)
                    {
                        newPoint.normal = (1. / (laserOriginAfterLOSolverTransformXYZ - newPoint.hitPoint).norm()) * newPoint.normal;
                    }

                    newPoint.quality = 1;

                    points->push_back(newPoint);
                }
            }
        }
        mid360MultiMapIter++;
    }

    output.result = Output::R_OK;

    workUnitProcessed(output);

    return true;
}
