#ifndef LOINTERPOLATOR_H
#define LOINTERPOLATOR_H

#include <qglobal.h>
#include <QMap>
#include "Eigen/Geometry"
#include "gnssmessage.h"
#include "losolver.h"

class LOInterpolator
{
public:
    inline LOInterpolator(const QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED> *relposnedMessages[3]);

    inline void getInterpolatedLocationOrientationTransformMatrix_Uptime(
        const qint64 uptime, const QMap<qint64, UBXMessage_RELPOSNED::ITOW>& averagedRoverUptimeSync,
        Eigen::Transform<double, 3, Eigen::Affine>& transform,
        const unsigned int maxInterpolationTimeRange = 500);

    inline void getInterpolatedLocationOrientationTransformMatrix_ITOW(
        const UBXMessage_RELPOSNED::ITOW iTOW,
        Eigen::Transform<double, 3, Eigen::Affine>& transform,
        const unsigned int maxInterpolationTimeRange = 500);

    LOSolver loSolver;  // This must be initialized by user of this class before using the interpolation function!

private:
    qint64 roverUptimeLimit_Low = -1;
    qint64 roverUptimeLimit_High = -1;
    Eigen::Vector3d roverUptimeBasedLocation_Low;
    Eigen::Vector3d roverUptimeBasedLocation_High;
    Eigen::Quaterniond roverUptimeBasedOrientation_Low;
    Eigen::Quaterniond roverUptimeBasedOrientation_High;

    UBXMessage_RELPOSNED::ITOW roverITOWLimit_Low = -1;
    UBXMessage_RELPOSNED::ITOW roverITOWLimit_High = -1;
    Eigen::Vector3d roverITOWBasedLocation_Low;
    Eigen::Vector3d roverITOWBasedLocation_High;
    Eigen::Quaterniond roverITOWBasedOrientation_Low;
    Eigen::Quaterniond roverITOWBasedOrientation_High;

    const QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>* relposnedMessages[3];

    static QString getRoverIdentString(const unsigned int roverId);
};

inline LOInterpolator::LOInterpolator(const QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>* relposnedMessages[3])
{
    for (int i = 0; i < 3; i++)
    {
        this->relposnedMessages [i]= relposnedMessages[i];
    }
}

inline void LOInterpolator::getInterpolatedLocationOrientationTransformMatrix_Uptime(
    const qint64 uptime, const QMap<qint64, UBXMessage_RELPOSNED::ITOW>& averagedRoverUptimeSync,
    Eigen::Transform<double, 3, Eigen::Affine>& transform,
    const unsigned int maxInterpolationTimeRange)
{
    UBXMessage_RELPOSNED roverRELPOSNEDS_Low[3];
    UBXMessage_RELPOSNED roverRELPOSNEDS_High[3];

    if ((uptime < roverUptimeLimit_Low) || (uptime >= roverUptimeLimit_High))
    {
        // "Cache miss" -> Find new limiting values

        auto roverUptimeIter = averagedRoverUptimeSync.upperBound(uptime);

        if (roverUptimeIter == averagedRoverUptimeSync.end())
        {
            roverUptimeLimit_Low = -1;
            roverUptimeLimit_High = -1;
            throw QString("Can not find corresponding rover uptime-averaged sync data (higher limit).");
        }

        if (roverUptimeIter == averagedRoverUptimeSync.begin())
        {
            roverUptimeLimit_Low = -1;
            roverUptimeLimit_High = -1;
            throw QString("Can not find corresponding rover uptime-averaged sync data (lower limit).");
        }

        roverUptimeLimit_High = roverUptimeIter.key();
        UBXMessage_RELPOSNED::ITOW roverITOWLimit_High = roverUptimeIter.value();
        roverUptimeIter--;
        roverUptimeLimit_Low = roverUptimeIter.key();
        UBXMessage_RELPOSNED::ITOW roverITOWLimit_Low = roverUptimeIter.value();

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator roverRELPOSNEDS_High[3];
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator roverRELPOSNEDS_Low[3];

        for (int i = 0; i < 3; i++)
        {
            roverRELPOSNEDS_Low[i] = relposnedMessages[i]->lowerBound(roverITOWLimit_Low);
            roverRELPOSNEDS_High[i] = relposnedMessages[i]->lowerBound(roverITOWLimit_High);
            Q_ASSERT(roverRELPOSNEDS_Low[i].key() == roverITOWLimit_Low);
            Q_ASSERT(roverRELPOSNEDS_High[i].key() == roverITOWLimit_High);
        }

        Eigen::Vector3d points_Low[3] =
            {
             { roverRELPOSNEDS_Low[0]->relPosN, roverRELPOSNEDS_Low[0]->relPosE, roverRELPOSNEDS_Low[0]->relPosD },
             { roverRELPOSNEDS_Low[1]->relPosN, roverRELPOSNEDS_Low[1]->relPosE, roverRELPOSNEDS_Low[1]->relPosD },
             { roverRELPOSNEDS_Low[2]->relPosN, roverRELPOSNEDS_Low[2]->relPosE, roverRELPOSNEDS_Low[2]->relPosD },
             };

        if (!loSolver.setPoints(points_Low))
        {
            throw QString("LOSolver.setPoints (low limit) failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        Eigen::Transform<double, 3, Eigen::Affine> transform_Low;

        if (!loSolver.getTransformMatrix(transform_Low))
        {
            throw QString("LOSolver.getTransformMatrix failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        roverUptimeBasedLocation_Low = transform_Low.translation();
        roverUptimeBasedOrientation_Low = transform_Low.linear();

        Eigen::Vector3d points_High[3] =
            {
             { roverRELPOSNEDS_High[0]->relPosN, roverRELPOSNEDS_High[0]->relPosE, roverRELPOSNEDS_High[0]->relPosD },
             { roverRELPOSNEDS_High[1]->relPosN, roverRELPOSNEDS_High[1]->relPosE, roverRELPOSNEDS_High[1]->relPosD },
             { roverRELPOSNEDS_High[2]->relPosN, roverRELPOSNEDS_High[2]->relPosE, roverRELPOSNEDS_High[2]->relPosD },
             };

        if (!loSolver.setPoints(points_High))
        {
            throw QString("LOSolver.setPoints (high limit) failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        Eigen::Transform<double, 3, Eigen::Affine> transform_High;

        if (!loSolver.getTransformMatrix(transform_High))
        {
            throw QString("LOSolver.getTransformMatrix failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        roverUptimeBasedLocation_High = transform_High.translation();
        roverUptimeBasedOrientation_High = transform_High.linear();
    }

    if (roverUptimeLimit_High - roverUptimeLimit_Low > maxInterpolationTimeRange)
    {
        throw("Maximum allowed time (" + QString::number(maxInterpolationTimeRange) +
              "ms) for interpolation exceeded. (Interpolation time: " +
              QString::number(roverUptimeLimit_High - roverUptimeLimit_Low) + ").");
    }

    double fraction = double(uptime - roverUptimeLimit_Low) / (roverUptimeLimit_High - roverUptimeLimit_Low);

    Q_ASSERT(fraction >= 0);
    Q_ASSERT(fraction <= 1);

    Eigen::Quaterniond slerpedQuat = roverUptimeBasedOrientation_Low.slerp(fraction, roverUptimeBasedOrientation_High);
    Eigen::Vector3d interpolatedCoords = roverUptimeBasedLocation_Low + fraction * (roverUptimeBasedLocation_High - roverUptimeBasedLocation_Low);

    transform.linear() = slerpedQuat.toRotationMatrix();
    transform.translation() = interpolatedCoords;
}

inline void LOInterpolator::getInterpolatedLocationOrientationTransformMatrix_ITOW(
    const UBXMessage_RELPOSNED::ITOW iTOW,
    Eigen::Transform<double, 3, Eigen::Affine>& transform,
    const unsigned int maxInterpolationTimeRange)
{
    if ((iTOW < roverITOWLimit_Low) || (iTOW >= roverITOWLimit_High))
    {
        // "Cache miss" -> Find new limiting values

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator roverRELPOSNEDS_Low[3];
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator roverRELPOSNEDS_High[3];

        for (int i = 0; i < 3; i++)
        {
            roverRELPOSNEDS_Low[i] = relposnedMessages[i]->lowerBound(iTOW);
            roverRELPOSNEDS_High[i] = relposnedMessages[i]->lowerBound(iTOW);
        }

        bool iTOWMatch = false;

        // Find next ITOW-value found for all rovers
        while (!iTOWMatch)
        {
            // Highest ITOW found from rovers' data is the lowest possible common one
            UBXMessage_RELPOSNED::ITOW highestITOW = -1;

            for (int i = 0; i < 3; i++)
            {
                if (roverRELPOSNEDS_High[i] == relposnedMessages[i]->end())
                {
                    roverITOWLimit_Low = -1;
                    roverITOWLimit_High = -1;
                    throw QString("Can not find higher limit interpolation value for rover" + getRoverIdentString(i) +
                                  ", ITOW: " + QString::number(iTOW));
                }

                if (roverRELPOSNEDS_High[i].key() > highestITOW)
                {
                    highestITOW = roverRELPOSNEDS_High[i].key();
                }
            }

            iTOWMatch = true;

            for (int i = 0; i < 3; i++)
            {
                if (roverRELPOSNEDS_High[i].key() != highestITOW)
                {
                    // Try to find item matching the highest ITOW
                    // Or jump to the next higher if match not found
                    roverRELPOSNEDS_High[i] = relposnedMessages[i]->lowerBound(highestITOW);
                    iTOWMatch = false;
                }
            }
        }

        roverITOWLimit_High = roverRELPOSNEDS_High[0].key();

        for (int i = 0; i < 3; i++)
        {
            // Jump to the messages preceeding the ITOW (if possible)
            if (roverRELPOSNEDS_Low[i] == relposnedMessages[i]->begin())
            {
                roverITOWLimit_Low = -1;
                roverITOWLimit_High = -1;
                throw QString("Can not find lower limit interpolation value for rover" + getRoverIdentString(i) +
                              ", ITOW: " + QString::number(iTOW));
            }

            roverRELPOSNEDS_Low[i]--;
        }

        iTOWMatch = false;

        while (!iTOWMatch)
        {
            UBXMessage_RELPOSNED::ITOW lowestITOW = 2e9;

            for (int i = 0; i < 3; i++)
            {
                if (roverRELPOSNEDS_Low[i].key() < lowestITOW)
                {
                    lowestITOW = roverRELPOSNEDS_Low[i].key();
                }
            }

            iTOWMatch = true;

            for (int i = 0; i < 3; i++)
            {
                roverRELPOSNEDS_Low[i] = relposnedMessages[i]->lowerBound(lowestITOW);

                if ((roverRELPOSNEDS_Low[i] == relposnedMessages[i]->end()) || (roverRELPOSNEDS_Low[i].key() != lowestITOW))
                {
                    if ((roverRELPOSNEDS_Low[i] == relposnedMessages[i]->end()) || (roverRELPOSNEDS_Low[i] == relposnedMessages[i]->begin()))
                    {
                        roverITOWLimit_Low = -1;
                        roverITOWLimit_High = -1;
                        throw QString("Can not find lower limit interpolation value for rover" + getRoverIdentString(i) +
                                      ", ITOW: " + QString::number(iTOW));
                    }

                    roverRELPOSNEDS_Low[i]--;
                    iTOWMatch = false;
                }
            }
        }

        roverITOWLimit_Low = roverRELPOSNEDS_Low[0].key();

        Eigen::Vector3d points_Low[3] =
            {
             { roverRELPOSNEDS_Low[0]->relPosN, roverRELPOSNEDS_Low[0]->relPosE, roverRELPOSNEDS_Low[0]->relPosD },
             { roverRELPOSNEDS_Low[1]->relPosN, roverRELPOSNEDS_Low[1]->relPosE, roverRELPOSNEDS_Low[1]->relPosD },
             { roverRELPOSNEDS_Low[2]->relPosN, roverRELPOSNEDS_Low[2]->relPosE, roverRELPOSNEDS_Low[2]->relPosD },
             };

        if (!loSolver.setPoints(points_Low))
        {
            throw QString("LOSolver.setPoints (low limit) failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        Eigen::Transform<double, 3, Eigen::Affine> transform_Low;

        if (!loSolver.getTransformMatrix(transform_Low))
        {
            throw QString("LOSolver.getTransformMatrix failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        roverITOWBasedLocation_Low = transform_Low.translation();
        roverITOWBasedOrientation_Low = transform_Low.linear();

        Eigen::Vector3d points_High[3] =
            {
             { roverRELPOSNEDS_High[0]->relPosN, roverRELPOSNEDS_High[0]->relPosE, roverRELPOSNEDS_High[0]->relPosD },
             { roverRELPOSNEDS_High[1]->relPosN, roverRELPOSNEDS_High[1]->relPosE, roverRELPOSNEDS_High[1]->relPosD },
             { roverRELPOSNEDS_High[2]->relPosN, roverRELPOSNEDS_High[2]->relPosE, roverRELPOSNEDS_High[2]->relPosD },
             };

        if (!loSolver.setPoints(points_High))
        {
            throw QString("LOSolver.setPoints (high limit) failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        Eigen::Transform<double, 3, Eigen::Affine> transform_High;

        if (!loSolver.getTransformMatrix(transform_High))
        {
            throw QString("LOSolver.getTransformMatrix failed. Error code: " + QString::number(loSolver.getLastError()) + ".");
        }

        roverITOWBasedLocation_High = transform_High.translation();
        roverITOWBasedOrientation_High = transform_High.linear();
    }

    if (roverITOWLimit_High - roverITOWLimit_Low > int(maxInterpolationTimeRange))
    {
        throw("Maximum allowed time (" + QString::number(maxInterpolationTimeRange) +
              "ms) for interpolation exceeded. (Interpolation time: " +
              QString::number(roverITOWLimit_High - roverITOWLimit_Low) + ").");
    }

    double fraction = double(iTOW - roverITOWLimit_Low) / (roverITOWLimit_High - roverITOWLimit_Low);

    Q_ASSERT(fraction >= 0);
    Q_ASSERT(fraction <= 1);

    Eigen::Quaterniond slerpedQuat = roverITOWBasedOrientation_Low.slerp(fraction, roverITOWBasedOrientation_High);
    Eigen::Vector3d interpolatedCoords = roverITOWBasedLocation_Low + fraction * (roverITOWBasedLocation_High - roverITOWBasedLocation_Low);

    transform.linear() = slerpedQuat.toRotationMatrix();
    transform.translation() = interpolatedCoords;
}

inline QString LOInterpolator::getRoverIdentString(const unsigned int roverId)
{
    if (roverId < ('X' - 'A'))
    {
        return QString(char('A' + (char)roverId));
    }
    else
    {
        // Should not happen
        return("X");
    }
}

#endif // LOINTERPOLATOR_H
