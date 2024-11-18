/*
    expressionfilter.cpp (part of GNSS-Stylus)
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

#include "expressionfilter.h"
#include "PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h"

namespace PointFilter{
ExpressionFilter::ExpressionFilter()
{
    initBuffer();

    transformCacheIndex_LidarToRig = 0;
    transformCache_LidarToRig[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_RigToNED = 0;
    transformCache_RigToNED[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    customFuncHandler = new TinyExprCustomFuncHandler(TE_DEFAULT, this);

    convexHullFilterIndexes = nullptr;
    setCustomVariablesAndFunctions();

    setExpression_Filter("1");
    setExpression_Quality("1");
}

void ExpressionFilter::initBuffer(void)
{
    bufferIndex = 0;

    for (BufferItem& item : buffer)
    {
        item.lidarSourceVector = Eigen::Vector3d::Zero();
        item.point_Lidar.setPrimarySourceVector(&item.lidarSourceVector);
        item.point_Lidar.setTransformedVector(item.lidarSourceVector);

        item.point_Rig.setSourceEvaluator(&item.point_Lidar);
        item.point_NED.setSourceEvaluator(&item.point_Rig);

        item.point_Rig.setTransform(&transformCache_LidarToRig[0]);
        item.point_NED.setTransform(&transformCache_RigToNED[0]);
    }
}

void ExpressionFilter::setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters)
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

ExpressionFilter::~ExpressionFilter()
{
    delete customFuncHandler;

    if (convexHullFilterIndexes)
    {
        delete[] convexHullFilterIndexes;
    }
}

bool ExpressionFilter::setExpression_Filter(const QString newExpression, QString* const errorMessage, int* const errorPosition)
{
    expression_Filter = newExpression;
    parser_Filter.compile(expression_Filter.toUtf8().constData());
    if (parser_Filter.success())
    {
        return true;
    }
    else
    {
        if (errorMessage)
        {
            *errorMessage = QString::fromStdString(parser_Filter.get_last_error_message());
        }
        if (errorPosition)
        {
            *errorPosition = parser_Filter.get_last_error_position();
        }

        return false;
    }
}

bool ExpressionFilter::setExpression_Quality(const QString newExpression, QString* const errorMessage, int* const errorPosition)
{
    expression_Quality = newExpression;
    parser_Quality.compile(expression_Quality.toUtf8().constData());
    if (parser_Quality.success())
    {
        return true;
    }
    else
    {
        if (errorMessage)
        {
            *errorMessage = QString::fromStdString(parser_Quality.get_last_error_message());
        }
        if (errorPosition)
        {
            *errorPosition = parser_Quality.get_last_error_position();
        }

        return false;
    }
}

void ExpressionFilter::setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_LidarToRig++;
    transformCache_LidarToRig[transformCacheIndex_LidarToRig] = newTransform;
}

void ExpressionFilter::setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_RigToNED++;
    transformCache_RigToNED[transformCacheIndex_RigToNED] = newTransform;
}

bool ExpressionFilter::setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    try
    {
        setCustomVariablesAndFunctions(newConvexHullFilters);
    }
    catch (...)
    {
        // tinyexpr++ set_variables_and_functions-function may thow an expection if there's unsupported characters
        // in the function names etc. Not making any deeper error checking here (should be done on a higher level).
        return false;
    }

    return true;
}



void ExpressionFilter::addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms)
{
    buffer[bufferIndex % bufferLength].point_Lidar_Source = lidarPoint;
    buffer[bufferIndex % bufferLength].lidarSourceVector = Eigen::Vector3d(lidarPoint.x, lidarPoint.y, lidarPoint.z);
    buffer[bufferIndex % bufferLength].uptime_ms = uptime_ms;

    buffer[bufferIndex % bufferLength].point_Lidar.invalidate();
    buffer[bufferIndex % bufferLength].point_Rig.invalidate();
    buffer[bufferIndex % bufferLength].point_NED.invalidate();

    buffer[bufferIndex % bufferLength].point_Rig.setTransform(&transformCache_LidarToRig[transformCacheIndex_LidarToRig]);
    buffer[bufferIndex % bufferLength].point_NED.setTransform(&transformCache_RigToNED[transformCacheIndex_RigToNED]);

    buffer[bufferIndex % bufferLength].point_Lidar.setTransformedVector(buffer[bufferIndex % bufferLength].lidarSourceVector);

    bufferIndex++;
}

bool ExpressionFilter::getFilteredPoint(OutItem& outPoint)
{
    if (bufferIndex < bufferLength)
    {
        outPoint.valid = false;
        return false;
    }

    outPoint.valid = true;
    outPoint.filterResult = parser_Filter.evaluate();

    if (outPoint.filterResult == 1.0)
    {
        outPoint.quality = parser_Quality.evaluate();
    }
    else
    {
        outPoint.quality = 0;
    }

    outPoint.uptime_ms = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].uptime_ms;
    outPoint.coords = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].point_NED.getTransformedVector();

    return true;
}


}; // namespace PointFilter



















