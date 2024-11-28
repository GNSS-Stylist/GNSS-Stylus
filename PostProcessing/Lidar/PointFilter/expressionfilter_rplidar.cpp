#include "expressionfilter_rplidar.h"
#include "PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h"

namespace PointFilter
{

ExpressionFilter_RPLIDAR::ExpressionFilter_RPLIDAR()
{
    initBuffer();
    ExpressionFilter_RPLIDAR::setCustomVariablesAndFunctions();
}

ExpressionFilter_RPLIDAR::~ExpressionFilter_RPLIDAR()
{
}

void ExpressionFilter_RPLIDAR::initBuffer(void)
{
    bufferIndex = 0;

    for (BufferItem& item : buffer)
    {
        item.angle_RPLidar = 0;
        item.distance_RPLidar = 0;
        item.quality_RPLidar = 0;
        item.point_Lidar.setPrimarySourceVector2D(&item.angle_RPLidar, &item.distance_RPLidar);

        item.point_Rig.setSourceEvaluator(&item.point_Lidar);
        item.point_NED.setSourceEvaluator(&item.point_Rig);

        item.point_Rig.setTransform(&transformCache_LidarToRig[0]);
        item.point_NED.setTransform(&transformCache_RigToNED[0]);
    }
}

void ExpressionFilter_RPLIDAR::setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    std::set<te_variable> customFunctions =
    {
        { "lidar.rplidar.quality", lidar_rplidar_quality, TE_DEFAULT, customFuncHandler },
        { "lidar.rplidar.quality_indexed", lidar_rplidar_quality_indexed, TE_DEFAULT, customFuncHandler },
    };

    customFunctions.merge(getCommonCustomFunctions(newConvexHullFilters));

    parser_Filter.set_variables_and_functions(customFunctions);
    parser_Quality.set_variables_and_functions(customFunctions);
}

void ExpressionFilter_RPLIDAR::addPoint(const RPLidarThread::DistanceItem& lidarPoint, const int uptime_ms)
{
    buffer[bufferIndex % bufferLength].angle_RPLidar = lidarPoint.angle;
    buffer[bufferIndex % bufferLength].distance_RPLidar = lidarPoint.distance;
    buffer[bufferIndex % bufferLength].quality_RPLidar = lidarPoint.quality;

    buffer[bufferIndex % bufferLength].uptime_ms = uptime_ms;

    buffer[bufferIndex % bufferLength].point_Lidar.invalidate();
    buffer[bufferIndex % bufferLength].point_Rig.invalidate();
    buffer[bufferIndex % bufferLength].point_NED.invalidate();

    buffer[bufferIndex % bufferLength].point_Rig.setTransform(&transformCache_LidarToRig[transformCacheIndex_LidarToRig]);
    buffer[bufferIndex % bufferLength].point_NED.setTransform(&transformCache_RigToNED[transformCacheIndex_RigToNED]);

    bufferIndex++;
}

}; // namespace PointFilter
