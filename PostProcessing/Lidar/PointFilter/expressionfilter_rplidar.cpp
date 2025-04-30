/*
    expressionfilter_rplidar.cpp (part of GNSS-Stylus)
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

#include "expressionfilter_rplidar.h"
#include "PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h"

namespace PointFilter
{

ExpressionFilter_RPLidar::ExpressionFilter_RPLidar()
{
    initBuffer();
    ExpressionFilter_RPLidar::setCustomVariablesAndFunctions();
}

ExpressionFilter_RPLidar::ExpressionFilter_RPLidar(const ExpressionFilter_RPLidar& source) : ExpressionFilter_Base(source)
{
    ExpressionFilter_RPLidar::setCustomVariablesAndFunctions();
    setExpression_Filter(expression_Filter);
    setExpression_Quality(expression_Quality);
}

ExpressionFilter_RPLidar ExpressionFilter_RPLidar::operator=(const ExpressionFilter_RPLidar& source)
{
    copyFields(source, *this);
    setCustomVariablesAndFunctions();
    setExpression_Filter(expression_Filter);
    setExpression_Quality(expression_Quality);
    return *this;
}

ExpressionFilter_RPLidar::~ExpressionFilter_RPLidar()
{
}

void ExpressionFilter_RPLidar::initBuffer(void)
{
    bufferIndex = 0;

    for (BufferItem& item : buffer)
    {
        item.horizontalAngle_RPLidar = 0;
        item.distance_RPLidar = 0;
        item.quality_RPLidar = 0;
        item.point_Lidar.setPrimarySourceVector2D(&item.horizontalAngle_RPLidar, &item.distance_RPLidar);

        item.point_Rig.setSourceEvaluator(&item.point_Lidar);
        item.point_NED.setSourceEvaluator(&item.point_Rig);
        item.point_XYZ.setSourceEvaluator(&item.point_NED);

        item.point_Rig.setTransform(&transformCache_LidarToRig[0]);
        item.point_NED.setTransform(&transformCache_RigToNED[0]);
        item.point_XYZ.setTransform(&transformCache_NEDToXYZ[0]);
    }
}

void ExpressionFilter_RPLidar::setCustomVariablesAndFunctions(void)
{
    std::set<te_variable> customFunctions =
    {
        { "lidar.rplidar.quality", lidar_rplidar_quality, TE_DEFAULT, this },
        { "lidar.rplidar.quality_indexed", lidar_rplidar_quality_indexed, TE_DEFAULT, this },
    };

    customFunctions.merge(getCommonCustomFunctions());

    parser_Filter.set_variables_and_functions(customFunctions);
    parser_Quality.set_variables_and_functions(customFunctions);
}

void ExpressionFilter_RPLidar::addPoint(const RPLidarThread::DistanceItem& lidarPoint, const qint64 timestamp)
{
    buffer[bufferIndex % bufferLength].horizontalAngle_RPLidar = lidarPoint.angle;
    buffer[bufferIndex % bufferLength].distance_RPLidar = lidarPoint.distance;
    buffer[bufferIndex % bufferLength].quality_RPLidar = lidarPoint.quality;

    buffer[bufferIndex % bufferLength].timestamp = timestamp;

    buffer[bufferIndex % bufferLength].point_Lidar.invalidate();
    buffer[bufferIndex % bufferLength].point_Rig.invalidate();
    buffer[bufferIndex % bufferLength].point_NED.invalidate();
    buffer[bufferIndex % bufferLength].point_XYZ.invalidate();

    buffer[bufferIndex % bufferLength].point_Rig.setTransform(&transformCache_LidarToRig[transformCacheIndex_LidarToRig]);
    buffer[bufferIndex % bufferLength].point_NED.setTransform(&transformCache_RigToNED[transformCacheIndex_RigToNED]);
    buffer[bufferIndex % bufferLength].point_XYZ.setTransform(&transformCache_NEDToXYZ[transformCacheIndex_NEDToXYZ]);

    bufferIndex++;
}

}; // namespace PointFilter
