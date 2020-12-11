/*
    rplidarplausibilityfilter.cpp (part of GNSS-Stylus)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include <QDebug>
#include "rplidarplausibilityfilter.h"

RPLidarPlausibilityFilter::RPLidarPlausibilityFilter()
{

}

void RPLidarPlausibilityFilter::filter(const QVector<RPLidarThread::DistanceItem>& source, QVector<FilteredItem>& dest)
{
/*
    qDebug() << "Angles:";

    for (int i = 0; i < source.count(); i++)
    {
        qDebug() << source[i].angle;
    }
*/

    int dbg_DistanceRejects = 0;
    int dbg_SlopeRejects = 0;

    dest.clear();

    for (int i = 0; i < source.count(); i++)
    {
        const RPLidarThread::DistanceItem& item = source[i];

        FilteredItem filteredItem;
        filteredItem.item = item;
        filteredItem.type = FilteredItem::FIT_PASSED;

        if ((item.angle < settings.startAngle) || (item.angle > settings.endAngle))
        {
            filteredItem.type = FilteredItem::FIT_REJECTED_ANGLE;
        }
        else if (item.quality < settings.qualityLimit_PreFiltering)
        {
            filteredItem.type = FilteredItem::FIT_REJECTED_QUALITY_PRE;
        }
        else if (item.distance < settings.distanceLimit_Near)
        {
            filteredItem.type = FilteredItem::FIT_REJECTED_DISTANCE_NEAR;
        }
        else if (item.distance > settings.distanceLimit_Far)
        {
            filteredItem.type = FilteredItem::FIT_REJECTED_DISTANCE_FAR;
        }

        dest.append(filteredItem);
    }

    bool filterDistanceDelta = settings.distanceDeltaLimit != 0;
    bool filterRelativeSlope = settings.relativeSlopeLimit != 0;

    for (int i = 0; i < dest.count(); i++)
    {
        FilteredItem& currentItem = dest[i];

        if (currentItem.type != FilteredItem::FIT_PASSED)
        {
            // Sample already filtered out, nothing to do here
            continue;
        }

        if (filterDistanceDelta)
        {
            // Try to filter out samples that are taken in a very shallow angle
            // Takes into account samples on both sides of the sample in question
            // and filters it out only if the direction of the slope on both directions match
            // and are over the changing speed limit.
            // This is done this way to prevent small details from being filtered out collaterally.
            // For example some small details (like handles) might be filtered out otherwise.

            bool prevDeltaOverLimit_Lowering = false;
            bool prevDeltaOverLimit_Rising = false;
            bool nextDeltaOverLimit_Lowering = false;
            bool nextDeltaOverLimit_Rising = false;

            if (i > 0)
            {
                const FilteredItem& prevItem = dest[i - 1];

                if (prevItem.type == FilteredItem::FIT_PASSED ||
                prevItem.type == FilteredItem::FIT_REJECTED_ANGLE ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_NEAR ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_FAR ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_DELTA)
                {
                    float prevDelta = (currentItem.item.distance - prevItem.item.distance) /
                            (currentItem.item.angle - prevItem.item.angle);
                    prevDeltaOverLimit_Lowering = prevDelta < -settings.distanceDeltaLimit;
                    prevDeltaOverLimit_Rising = prevDelta > settings.distanceDeltaLimit;
                }
            }

            if (i < dest.count() - 1)
            {
                const FilteredItem& nextItem = dest[i + 1];

                if (nextItem.type == FilteredItem::FIT_PASSED ||
                nextItem.type == FilteredItem::FIT_REJECTED_ANGLE ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_NEAR ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_FAR ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_DELTA)
                {
                    float nextDelta = (nextItem.item.distance - currentItem.item.distance) /
                            (nextItem.item.angle - currentItem.item.angle);
                    nextDeltaOverLimit_Lowering = nextDelta < -settings.distanceDeltaLimit;
                    nextDeltaOverLimit_Rising = nextDelta > settings.distanceDeltaLimit;
                }
            }

            if ((prevDeltaOverLimit_Rising && nextDeltaOverLimit_Rising) ||
                     (prevDeltaOverLimit_Lowering && nextDeltaOverLimit_Lowering))
            {
                currentItem.type = FilteredItem::FIT_REJECTED_DISTANCE_DELTA;
                dbg_DistanceRejects++;
            }
        }

        if (filterRelativeSlope)
        {
            // Try to filter out samples that are taken in a very shallow angle
            // Takes into account samples on both sides of the sample in question
            // and filters it out only if the direction of the slope on both directions match
            // and are over the changing speed limit. This is for relative change.
            // This is done this way to prevent small details from being filtered out collaterally.
            // For example some small details (like handles) might be filtered out otherwise.

            bool prevSlopeOverLimit_Lowering = false;
            bool prevSlopeOverLimit_Rising = false;
            bool nextSlopeOverLimit_Lowering = false;
            bool nextSlopeOverLimit_Rising = false;

            if (i > 0)
            {
                const FilteredItem& prevItem = dest[i - 1];

                if (prevItem.type == FilteredItem::FIT_PASSED ||
                prevItem.type == FilteredItem::FIT_REJECTED_ANGLE ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_NEAR ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_FAR ||
                prevItem.type == FilteredItem::FIT_REJECTED_DISTANCE_DELTA ||
                prevItem.type == FilteredItem::FIT_REJECTED_SLOPE)
                {
                    if (prevItem.item.distance == 0)
                    {
                        prevSlopeOverLimit_Rising = true;
                    }
                    else
                    {
                        float prevSlope = ((prevItem.item.distance - currentItem.item.distance) / currentItem.item.distance) /
                                (prevItem.item.angle - currentItem.item.angle);
                        prevSlopeOverLimit_Lowering = prevSlope < (-1. / (1. + settings.relativeSlopeLimit));
                        prevSlopeOverLimit_Rising = prevSlope > settings.relativeSlopeLimit;
                    }
                }
            }

            if (i < dest.count() - 1)
            {
                const FilteredItem& nextItem = dest[i + 1];

                if (nextItem.type == FilteredItem::FIT_PASSED ||
                nextItem.type == FilteredItem::FIT_REJECTED_ANGLE ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_NEAR ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_FAR ||
                nextItem.type == FilteredItem::FIT_REJECTED_DISTANCE_DELTA ||
                nextItem.type == FilteredItem::FIT_REJECTED_SLOPE)
                {
                    if (currentItem.item.distance == 0)
                    {
                        nextSlopeOverLimit_Rising = true;
                    }
                    else
                    {
                        float nextSlope = ((nextItem.item.distance - currentItem.item.distance) / currentItem.item.distance) /
                                (nextItem.item.angle - currentItem.item.angle);
                        nextSlopeOverLimit_Lowering = nextSlope < (-1. / (1. + settings.relativeSlopeLimit));
                        nextSlopeOverLimit_Rising = nextSlope > settings.relativeSlopeLimit;
                    }
                }
            }

            if ((prevSlopeOverLimit_Rising && nextSlopeOverLimit_Rising) ||
                     (prevSlopeOverLimit_Lowering && nextSlopeOverLimit_Lowering))
            {
                currentItem.type = FilteredItem::FIT_REJECTED_SLOPE;
                dbg_SlopeRejects++;
            }
        }
    }

    // Quality filtering (post)
    for (int i = 0; i < dest.count(); i++)
    {
        FilteredItem& currentItem = dest[i];

        if (currentItem.type != FilteredItem::FIT_PASSED)
        {
            // Sample already filtered out, nothing to do here
            continue;
        }

        if (currentItem.item.quality < settings.qualityLimit_PostFiltering)
        {
            currentItem.type = FilteredItem::FIT_REJECTED_QUALITY_POST;
        }
    }
}
