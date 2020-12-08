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

#include "rplidarplausibilityfilter.h"

RPLidarPlausibilityFilter::RPLidarPlausibilityFilter()
{

}

void RPLidarPlausibilityFilter::filter(const QVector<RPLidarThread::DistanceItem>& source, QVector<FilteredItem>& dest)
{
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
}
