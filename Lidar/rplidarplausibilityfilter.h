/*
    rplidarplausibilityfilter.h (part of GNSS-Stylus)
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

#ifndef RPLIDARPLAUSIBILITYFILTER_H
#define RPLIDARPLAUSIBILITYFILTER_H

#include "rplidarthread.h"
#include <math.h>

class RPLidarPlausibilityFilter
{
public:

    class Settings
    {
    public:
        float startAngle = 0;
        float endAngle = M_PI * 2;

        float qualityLimit_PreFiltering = 0;
        float qualityLimit_PostFiltering = 0;

        float distanceLimit_Near = 0;
        float distanceLimit_Far = 1e9;

        float distanceDeltaLimit = 0;   //!< Discard sample when changing speed in relation to the previous and next sample of distance are higher than this (meters/radian) 0 -> Don't use
        float relativeSlopeLimit = 0;   //!< Discard sample when relative changing speed in relation to the previous and next sample of distance are higher than this (relative change/radian) 0 -> Don't use

/*        float gapLimit = M_PI;
        float discardAngle_BeforeGap = 0;
        float discardAngle_AfterGap = 0;

        float distanceDiscontinuityLimit_Increasing = 100;
        float distanceDiscontinuityLimit_Decreasing = 100;
        float discardAngle_BeforeDiscontinuity = 0;
        float discardAngle_AfterDiscontinuity = 0;
*/
    };

    class FilteredItem
    {
    public:
        enum Type
        {
            FIT_PASSED = 0,
            FIT_REJECTED_ANGLE,
            FIT_REJECTED_QUALITY_PRE,
            FIT_REJECTED_QUALITY_POST,
            FIT_REJECTED_DISTANCE_NEAR,
            FIT_REJECTED_DISTANCE_FAR,
            FIT_REJECTED_DISTANCE_DELTA,
            FIT_REJECTED_SLOPE,
/*            FIT_REJECTED_GAP_BEFORE,
            FIT_REJECTED_GAP_AFTER,
            FIT_REJECTED_DISCONTINUITY_BEFORE,
            FIT_REJECTED_DISCONTINUITY_AFTER,
*/
        };

        Type type;
        RPLidarThread::DistanceItem item;
    };

    RPLidarPlausibilityFilter();
    RPLidarPlausibilityFilter(const Settings& settings) { setSettings(settings); };

    void setSettings(const Settings& settings) { this->settings = settings; };

    void filter(const QVector<RPLidarThread::DistanceItem>& source, QVector<FilteredItem>& dest);

private:
    Settings settings;
};

#endif // RPLIDARPLAUSIBILITYFILTER_H
