/*
    lidarscriptgeneratorthread.cpp (part of GNSS-Stylus)
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

#include "lidarscriptgeneratorthread.h"
#include "../Lidar/PointFilter/expressionfilter_mid360.h"
#include "../Lidar/PointFilter/expressionfilter_rplidar.h"

LidarScriptGeneratorThread::LidarScriptGeneratorThread(const ConstData &cData, std::function< WorkUnit(void) > workUnitGetter, std::function<void (const LidarDevice&, const Output &)> workUnitProcessor)
{
    constData = cData;
    getWorkUnit = workUnitGetter;
    workUnitProcessed = workUnitProcessor;
}


void LidarScriptGeneratorThread::run()
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
    stateMutex.lock();
    state = S_PROCESSING;
    stateMutex.unlock();

    while ((workUnitInProgress.valid) && (!terminateRequest))
    {
        processWorkUnit(loInterpolator);
        workUnitInProgress = getWorkUnit();
    }

    stateMutex.lock();
    state = S_DONE;
    stateMutex.unlock();
    progressFractionMutex.lock();
    progressFraction = 0;
    progressFractionMutex.unlock();
}

LidarScriptGeneratorThread::State LidarScriptGeneratorThread::getState(float* progressFraction)
{
    State retval;
    stateMutex.lock();
    retval = state;
    stateMutex.unlock();

    if (progressFraction)
    {
        progressFractionMutex.lock();
        *progressFraction = this->progressFraction;
        progressFractionMutex.unlock();
    }

    return retval;
}

bool LidarScriptGeneratorThread::processWorkUnit(LOInterpolator &loInterpolator)
{
    progressFractionMutex.lock();
    progressFraction = 0.0;
    progressFractionMutex.unlock();

    Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

    for (auto exprIter = expressionMap_Local.begin(); exprIter != expressionMap_Local.end(); exprIter++)
    {
        exprIter.value()->initBuffer();
    }

    // TODO: Add RPLidar

    QMap<LidarDevice, std::shared_ptr<Output> > deviceOutputs;

    // Just rely on the timestamps of the datagrams to make splitting the data into chunks (for threads to digest) easier.
    // The uptime range will be from beginningUptime (inclusive) to endingUptime (exclusive).
    // Filtering needs some adjustments due to buffering/delay,
    // this is done now by feeding bufferLength (now 16) samples from the datagram preceding the one found using the timestamp.
    // This adds a tiny time inaccuracy (8/200000s ("delay" of 8 samples)), so doesn't matter.

//    qint64 iTOWTime_ns_Begin = getITOW(workUnitInProgress.beginningUptime) * 1000000ULL;
//    qint64 iTOWTime_ns_End = getITOW(workUnitInProgress.endingUptime) * 1000000ULL;
    qint64 iTOWTime_ns_Begin = workUnitInProgress.beginningITOWTime_ns;
    qint64 iTOWTime_ns_End = workUnitInProgress.endingITOWTime_ns;

    QMap<qint64, PostProcessingForm::Mid360Datagram* >::const_iterator mid360MapIter = constData.mid360.datagrams->lowerBound(iTOWTime_ns_Begin);

    UBXMessage_RELPOSNED::ITOW lastInterpolatedITOWUptime_ms = -1;

    while ((mid360MapIter != constData.mid360.datagrams->end()) && (mid360MapIter.key() < iTOWTime_ns_End) && !terminateRequest)
    {
        if (progressFractionMutex.try_lock())
        {
            // As estimated number of points from Mid-360 is in the order of *10 of RPLidar's, lets scale progress here to about 0.1-1
            progressFraction = 0.1 + ((float(mid360MapIter.key() - iTOWTime_ns_Begin) / (iTOWTime_ns_End - iTOWTime_ns_Begin)) * 0.9);
            progressFractionMutex.unlock();
        }

        qint64 iTOWTime_ns = mid360MapIter.key();

        auto scanningStateIter = constData.scanningStateMap->lowerBound(iTOWTime_ns);
        bool scanningActive = false;

        if (scanningStateIter != constData.scanningStateMap->end())
        {
            scanningActive = scanningStateIter->scanningActive;
        }

        const PostProcessingForm::Mid360Datagram* mid360Datagram = mid360MapIter.value();
        LivoxMid360::PointCloudAndIMUDataHeader header(mid360Datagram->datagram);

        if ((header.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) || (
                (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL) &&
                (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
                (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT)))
        {
            continue;
        }
        LivoxMid360::PointCloudData pcData(header, mid360Datagram->datagram);

        if ((pcData.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) ||
            (pcData.time_type != LivoxMid360::PointCloudAndIMUDataHeader::TimeSyncType::TIME_SYNC_GPS))
        {
            continue;
        }

        quint64 pointStartTime_ns = pcData.timestamp;
        quint64 pointChunkTime_ns = quint64(pcData.time_interval) * 100;

        quint32 ipAddress = mid360Datagram->datagram.senderAddress().toIPv4Address();
        LidarDevice device(LidarDevice::DT_LIVOX_MID360, ipAddress);

        Output* output;

        if (!deviceOutputs.contains(device))
        {
            // First datagram for this device -> create output
            auto newItem = deviceOutputs.insert(device, std::make_shared<Output>());
            newItem.value()->workUnit = workUnitInProgress;
            newItem.value()->points = std::make_shared<QVector<Output::Point> >();
        }
        output = deviceOutputs.value(device).get();

        if (!expressionMap_Local.contains(device))
        {
            output->errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram->fileNameIndex) + "\", chunk index " +
                                 QString::number(mid360Datagram->chunkIndex)+
                                 " (Mid-360), IP: " + mid360Datagram->datagram.senderAddress().toString() +
                                 ", uptime " + QString::number(iTOWTime_ns) +
                                 ", ITOW " + QString::number(pointStartTime_ns / 1000000) +
                                 ": Filter expression not defined. Quitting generating script for this device.";

            output->result = Output::R_ERROR;
//                workUnitProcessed(output);

            return(false);
        }

        PointFilter::ExpressionFilter_Mid360* exprFilter = dynamic_cast<PointFilter::ExpressionFilter_Mid360*> (expressionMap_Local.value(device).get());

        exprFilter->setTransform_NEDToXYZ(*constData.transform_NEDToXYZ);

        if ((exprFilter->getNumOfAddedPoints() < exprFilter->bufferLength) && (mid360MapIter != constData.mid360.datagrams->begin()))
        {
            // To allow chunks to be split for different threads to handle, the starting and ending times of subsequent chunks must match exactly.
            // Therefore "prefilling" the filter with the data (last samples) from the previous datagram for this device.
            // This code is quite similar to the "real" filtering code later. Will not combine these since the "real" filtering should be as fast as possible.
            // (This part is only ran once per "point set", so doesn't need to be very optimized).

            auto backIter = constData.mid360.datagrams->lowerBound(iTOWTime_ns_Begin);

            while ((backIter != constData.mid360.datagrams->begin()) && (exprFilter->getNumOfAddedPoints() < exprFilter->bufferLength))
            {
                backIter--;

                const PostProcessingForm::Mid360Datagram* mid360Datagram_Back = backIter.value();

                quint32 ipAddress_Back = mid360Datagram_Back->datagram.senderAddress().toIPv4Address();

                if (ipAddress_Back != ipAddress)
                {
                    continue;
                }

                LivoxMid360::PointCloudAndIMUDataHeader header_Back(mid360Datagram_Back->datagram);

                if ((header_Back.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) || (
                        (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL) &&
                        (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
                        (header_Back.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT)))
                {
                    continue;
                }

                LivoxMid360::PointCloudData pcData_Back(header_Back, mid360Datagram_Back->datagram);

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
                    qint64 pointITOWTime_ns = (pointStartTime_ns_Back + ((pointChunkTime_ns_Back * i) / (pointNum_Back - 1)));
                    UBXMessage_RELPOSNED::ITOW pointITOWTime_ms = pointITOWTime_ns / 1000000;

                    if (pointITOWTime_ms != lastInterpolatedITOWUptime_ms)
                    {
                        try
                        {
                            loInterpolator.getInterpolatedLocationOrientationTransformMatrix_ITOW(pointITOWTime_ms, transform_LoSolver);
                        }
                        catch (QString& stringThrown)
                        {
                            Q_ASSERT(constData.lidarFileNames);
                            Q_ASSERT(constData.lidarFileNames->size() >  mid360Datagram_Back->fileNameIndex);

                            output->errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram_Back->fileNameIndex) + "\", chunk index " +
                                                 QString::number(mid360Datagram_Back->chunkIndex)+
                                                 " (Mid-360), IP: " + mid360Datagram_Back->datagram.senderAddress().toString() +
//                                                 ", uptime " + QString::number(uptime_Back) +
                                                 ", ITOW " + QString::number(pointITOWTime_ms) +
                                                 ": " + stringThrown + " Quitting generating script for this device.";

                            output->result = Output::R_ERROR;
//                                    workUnitProcessed(output);

                            return(false);
                        }

                        exprFilter->setTransform_RigToNED(transform_LoSolver);

                        lastInterpolatedITOWUptime_ms = pointITOWTime_ms;
                    }

                    exprFilter->addPoint(*currentPoint, pointITOWTime_ns);
                }
            }
        }

        quint16 pointNum = pcData.dot_num;

        if (!constData.transforms_AfterRotation->contains(device))
        {
            output->errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram->fileNameIndex) + "\", chunk index " +
                                 QString::number(mid360Datagram->chunkIndex)+
                                 " (Mid-360), IP: " + mid360Datagram->datagram.senderAddress().toString() +
                                 ", uptime " + QString::number(iTOWTime_ns) +
                                 ", ITOW " + QString::number(pointStartTime_ns / 1000000) +
                                 ": Operation after rotation not defined. Quitting generating script for this device.";

            output->result = Output::R_ERROR;
//                workUnitProcessed(output);

            return(false);
        }

        auto transform_AfterRotation = constData.transforms_AfterRotation->value(device);

        //        Eigen::Transform<double, 3, Eigen::Affine> transform_BeforeRotation = *params.rpLidar.transform_BeforeRotation;

        QVector<Output::Point>* points = output->points.get();

        for (int i = 0; i < pointNum; i++)
        {
            LivoxMid360::PointCloudData::Point* currentPoint = &pcData.points[i];

            qint64 pointITOWUptime_ns = pointStartTime_ns + ((pointChunkTime_ns * i) / (pointNum -1));

            UBXMessage_RELPOSNED::ITOW pointITOWUptime_ms = pointStartTime_ns / 1000000;

            if (pointITOWUptime_ms != lastInterpolatedITOWUptime_ms)
            {
                try
                {
                    loInterpolator.getInterpolatedLocationOrientationTransformMatrix_ITOW(pointITOWUptime_ms, transform_LoSolver);
                }
                catch (QString& stringThrown)
                {
                    Q_ASSERT(constData.lidarFileNames);
                    Q_ASSERT(constData.lidarFileNames->size() > mid360Datagram->fileNameIndex);

                    output->errorString = "File \"" + constData.lidarFileNames->at(mid360Datagram->fileNameIndex) + "\", chunk index " +
                                         QString::number(mid360Datagram->chunkIndex)+
                                         " (Mid-360), IP: " + mid360Datagram->datagram.senderAddress().toString() +
                                         ", uptime " + QString::number(iTOWTime_ns) +
                                         ", ITOW " + QString::number(pointITOWUptime_ms) +
                                         ": " + stringThrown + " Quitting generating script for this device.";

                    output->result = Output::R_ERROR;
//                        workUnitProcessed(output);

                    return(false);
                }

                exprFilter->setTransform_RigToNED(transform_LoSolver);
                lastInterpolatedITOWUptime_ms = pointITOWUptime_ms;
            }

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

            Output::Point newPoint;

            newPoint.iTOW_ns = pointITOWUptime_ns;
            newPoint.hitPoint = exprOutItem.coords;
            newPoint.sourcePoint = laserOriginAfterLOSolverTransformXYZ;

            if (!scanningActive)
            {
                newPoint.type = LidarScriptGeneratorThread::Output::PT_SCANNING_NOT_ACTIVE;
            }
            else if (exprOutItem.filterResult == 0)
            {
                newPoint.type = LidarScriptGeneratorThread::Output::PT_MISS;
            }
            else
            {
                newPoint.type = LidarScriptGeneratorThread::Output::PT_HIT;
            }
            newPoint.quality = exprOutItem.quality;

            points->push_back(newPoint);
        }
        mid360MapIter++;
    }

    progressFractionMutex.lock();
    progressFraction = 1.0;
    progressFractionMutex.unlock();

//    output->result = Output::R_OK;

    auto outputIter = deviceOutputs.constBegin();
    while (outputIter != deviceOutputs.constEnd())
    {
        workUnitProcessed(outputIter.key(), *outputIter.value().get());
        outputIter++;
    }

    return true;
}
