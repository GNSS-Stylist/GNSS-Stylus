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
    if (convexHullFilterIndexes)
    {
        delete[] convexHullFilterIndexes;
    }
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
         { "rad_to_deg", rad_to_deg, TE_PURE },
         { "deg_to_rad", deg_to_rad, TE_PURE },

         { "lidar.coord.x", lidar_coord_x, TE_DEFAULT, customFuncHandler },
         { "lidar.coord.y", lidar_coord_y, TE_DEFAULT, customFuncHandler },
         { "lidar.coord.z", lidar_coord_z, TE_DEFAULT, customFuncHandler },

         { "lidar.coord_indexed.x", lidar_coord_indexed_x, TE_DEFAULT, customFuncHandler },
         { "lidar.coord_indexed.y", lidar_coord_indexed_y, TE_DEFAULT, customFuncHandler },
         { "lidar.coord_indexed.z", lidar_coord_indexed_z, TE_DEFAULT, customFuncHandler },

         { "lidar.properties", lidar_properties, TE_DEFAULT, customFuncHandler },
         { "lidar.properties.other", lidar_properties_other, TE_DEFAULT, customFuncHandler },
         { "lidar.properties.dust", lidar_properties_dust, TE_DEFAULT, customFuncHandler },
         { "lidar.properties.glue", lidar_properties_glue, TE_DEFAULT, customFuncHandler },
         { "lidar.reflectivity", lidar_reflectivity, TE_DEFAULT, customFuncHandler },

         { "lidar.properties_indexed", lidar_properties_indexed, TE_DEFAULT, customFuncHandler },
         { "lidar.properties_indexed.other", lidar_properties_indexed_other, TE_DEFAULT, customFuncHandler },
         { "lidar.properties_indexed.dust", lidar_properties_indexed_dust, TE_DEFAULT, customFuncHandler },
         { "lidar.properties_indexed.glue", lidar_properties_indexed_glue, TE_DEFAULT, customFuncHandler },
         { "lidar.reflectivity_indexed", lidar_reflectivity_indexed, TE_DEFAULT, customFuncHandler },

         { "lidar.distance", lidar_distance, TE_DEFAULT, customFuncHandler },
         { "lidar.distance_indexed", lidar_distance_indexed, TE_DEFAULT, customFuncHandler },
         { "lidar.angle.horizontal", lidar_angle_horizontal, TE_DEFAULT, customFuncHandler },
         { "lidar.angle_indexed.horizontal", lidar_angle_indexed_horizontal, TE_DEFAULT, customFuncHandler },
         { "lidar.angle.vertical", lidar_angle_vertical, TE_DEFAULT, customFuncHandler },
         { "lidar.angle_indexed.vertical", lidar_angle_indexed_vertical, TE_DEFAULT, customFuncHandler },

         { "rig.coord.x", rig_coord_x, TE_DEFAULT, customFuncHandler },
         { "rig.coord.y", rig_coord_y, TE_DEFAULT, customFuncHandler },
         { "rig.coord.z", rig_coord_z, TE_DEFAULT, customFuncHandler },

         { "rig.coord_indexed.x", rig_coord_indexed_x, TE_DEFAULT, customFuncHandler },
         { "rig.coord_indexed.y", rig_coord_indexed_y, TE_DEFAULT, customFuncHandler },
         { "rig.coord_indexed.z", rig_coord_indexed_z, TE_DEFAULT, customFuncHandler },

         { "ned.coord.x", ned_coord_x, TE_DEFAULT, customFuncHandler },
         { "ned.coord.y", ned_coord_y, TE_DEFAULT, customFuncHandler },
         { "ned.coord.z", ned_coord_z, TE_DEFAULT, customFuncHandler },

         { "ned.coord_indexed.x", ned_coord_indexed_x, TE_DEFAULT, customFuncHandler },
         { "ned.coord_indexed.y", ned_coord_indexed_y, TE_DEFAULT, customFuncHandler },
         { "ned.coord_indexed.z", ned_coord_indexed_z, TE_DEFAULT, customFuncHandler },

         { "lidar.in_convex_hull", lidar_in_convex_hull, TE_DEFAULT, customFuncHandler },
         { "lidar.in_convex_hull_indexed", lidar_in_convex_hull_indexed, TE_DEFAULT, customFuncHandler },

         { "rig.in_convex_hull", rig_in_convex_hull, TE_DEFAULT, customFuncHandler },
         { "rig.in_convex_hull_indexed", rig_in_convex_hull_indexed, TE_DEFAULT, customFuncHandler },

         { "ned.in_convex_hull", ned_in_convex_hull, TE_DEFAULT, customFuncHandler },
         { "ned.in_convex_hull_indexed", ned_in_convex_hull_indexed, TE_DEFAULT, customFuncHandler },

         { "lidar.in_aabb", lidar_in_aabb, TE_DEFAULT, customFuncHandler },
         { "lidar.in_aabb_indexed", lidar_in_aabb_indexed, TE_DEFAULT, customFuncHandler },

         { "rig.in_aabb", rig_in_aabb, TE_DEFAULT, customFuncHandler },
         { "rig.in_aabb_indexed", rig_in_aabb_indexed, TE_DEFAULT, customFuncHandler },

         { "ned.in_aabb", ned_in_aabb, TE_DEFAULT, customFuncHandler },
         { "ned.in_aabb_indexed", ned_in_aabb_indexed, TE_DEFAULT, customFuncHandler },

         { "lidar.in_sphere", lidar_in_sphere, TE_DEFAULT, customFuncHandler },
         { "lidar.in_sphere_indexed", lidar_in_sphere_indexed, TE_DEFAULT, customFuncHandler },

         { "rig.in_sphere", rig_in_sphere, TE_DEFAULT, customFuncHandler },
         { "rig.in_sphere_indexed", rig_in_sphere_indexed, TE_DEFAULT, customFuncHandler },

         { "ned.in_sphere", ned_in_sphere, TE_DEFAULT, customFuncHandler },
         { "ned.in_sphere_indexed", ned_in_sphere_indexed, TE_DEFAULT, customFuncHandler },


         };

    convexHullFilters.clear();

    numOfConvexHullFilters = newConvexHullFilters.size();

    if (convexHullFilterIndexes)
    {
        delete[] convexHullFilterIndexes;
        convexHullFilterIndexes = nullptr;
    }
    if (numOfConvexHullFilters != 0)
    {
        convexHullFilterIndexes = new te_type[numOfConvexHullFilters];
    }

    QVector<QByteArray> convexHullFilterNames; // To keep strings alive while adding.

    for (unsigned int i = 0; i < numOfConvexHullFilters; i++)
    {
        convexHullFilterNames.push_back((QString("chull_") + newConvexHullFilters[i].Name).toLower().toLocal8Bit());
        convexHullFilterIndexes[i] = te_type(i);
        convexHullFilters.push_back(newConvexHullFilters[i].filter);

        te_variable newConstant { convexHullFilterNames[i].constData(), &convexHullFilterIndexes[i], TE_PURE };

        customFunctions.insert(newConstant);
    }

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
