/*
    expressionfilter_mid360.cpp (part of GNSS-Stylus)
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

#include "expressionfilter_mid360.h"
#include "PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h"

namespace PointFilter
{

ExpressionFilter_Mid360::ExpressionFilter_Mid360()
{
    initBuffer();
    ExpressionFilter_Mid360::setCustomVariablesAndFunctions();
}

ExpressionFilter_Mid360::~ExpressionFilter_Mid360()
{
}

void ExpressionFilter_Mid360::initBuffer(void)
{
    bufferIndex = 0;

    for (BufferItem& item : buffer)
    {
        item.lidarSourceVector = Eigen::Vector3d::Zero();
        item.point_Lidar.setPrimarySourceVector(&item.lidarSourceVector);

        item.point_Rig.setSourceEvaluator(&item.point_Lidar);
        item.point_NED.setSourceEvaluator(&item.point_Rig);

        item.point_Rig.setTransform(&transformCache_LidarToRig[0]);
        item.point_NED.setTransform(&transformCache_RigToNED[0]);
    }
}

void ExpressionFilter_Mid360::setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    std::set<te_variable> customFunctions =
    {
         { "lidar.mid360.properties", lidar_properties, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties.other", lidar_properties_other, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties.dust", lidar_properties_dust, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties.glue", lidar_properties_glue, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.reflectivity", lidar_reflectivity, TE_DEFAULT, customFuncHandler },

         { "lidar.mid360.properties_indexed", lidar_properties_indexed, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties_indexed.other", lidar_properties_indexed_other, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties_indexed.dust", lidar_properties_indexed_dust, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.properties_indexed.glue", lidar_properties_indexed_glue, TE_DEFAULT, customFuncHandler },
         { "lidar.mid360.reflectivity_indexed", lidar_reflectivity_indexed, TE_DEFAULT, customFuncHandler },
    };

    customFunctions.merge(getCommonCustomFunctions(newConvexHullFilters));

    parser_Filter.set_variables_and_functions(customFunctions);
    parser_Quality.set_variables_and_functions(customFunctions);
}

void ExpressionFilter_Mid360::addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms)
{
    buffer[bufferIndex % bufferLength].point_Lidar_Source = lidarPoint;
    buffer[bufferIndex % bufferLength].lidarSourceVector = Eigen::Vector3d(lidarPoint.x, lidarPoint.y, lidarPoint.z);
    buffer[bufferIndex % bufferLength].uptime_ms = uptime_ms;

    buffer[bufferIndex % bufferLength].point_Lidar.invalidate();
    buffer[bufferIndex % bufferLength].point_Rig.invalidate();
    buffer[bufferIndex % bufferLength].point_NED.invalidate();

    buffer[bufferIndex % bufferLength].point_Rig.setTransform(&transformCache_LidarToRig[transformCacheIndex_LidarToRig]);
    buffer[bufferIndex % bufferLength].point_NED.setTransform(&transformCache_RigToNED[transformCacheIndex_RigToNED]);

    bufferIndex++;
}

}; // namespace PointFilter
