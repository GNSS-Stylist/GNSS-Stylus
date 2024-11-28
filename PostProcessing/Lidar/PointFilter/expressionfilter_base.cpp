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

#include "expressionfilter_base.h"
#include "tinyexprcustomfunctions.h"

namespace PointFilter
{

ExpressionFilter_Base::ExpressionFilter_Base()
{
//    initBuffer();

    transformCacheIndex_LidarToRig = 0;
    transformCache_LidarToRig[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_RigToNED = 0;
    transformCache_RigToNED[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    customFuncHandler = new TinyExprCustomFuncHandler(TE_DEFAULT, this);

    convexHullFilterIndexes = nullptr;
//    setCustomVariablesAndFunctions();

    setExpression_Filter("1");
    setExpression_Quality("1");
}

ExpressionFilter_Base::~ExpressionFilter_Base()
{
    if (convexHullFilterIndexes)
    {
        delete[] convexHullFilterIndexes;
    }

    delete customFuncHandler;
}


bool ExpressionFilter_Base::setExpression_Filter(const QString newExpression, QString* const errorMessage, int* const errorPosition)
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

bool ExpressionFilter_Base::setExpression_Quality(const QString newExpression, QString* const errorMessage, int* const errorPosition)
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

void ExpressionFilter_Base::setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_LidarToRig++;
    transformCache_LidarToRig[transformCacheIndex_LidarToRig] = newTransform;
}

void ExpressionFilter_Base::setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_RigToNED++;
    transformCache_RigToNED[transformCacheIndex_RigToNED] = newTransform;
}

bool ExpressionFilter_Base::setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters)
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

bool ExpressionFilter_Base::getFilteredPoint(OutItem& outPoint)
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

std::set<te_variable> ExpressionFilter_Base::getCommonCustomFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    std::set<te_variable> customFunctions =
    {
        { "rad_to_deg", rad_to_deg, TE_PURE },
        { "deg_to_rad", deg_to_rad, TE_PURE },

        { "lidar.coord.x", lidar_coord_x, TE_DEFAULT, customFuncHandler },
        { "lidar.coord.y", lidar_coord_y, TE_DEFAULT, customFuncHandler },
        { "lidar.coord.z", lidar_coord_z, TE_DEFAULT, customFuncHandler },  // Always zero when using 2D-lidar

        { "lidar.coord_indexed.x", lidar_coord_indexed_x, TE_DEFAULT, customFuncHandler },
        { "lidar.coord_indexed.y", lidar_coord_indexed_y, TE_DEFAULT, customFuncHandler },
        { "lidar.coord_indexed.z", lidar_coord_indexed_z, TE_DEFAULT, customFuncHandler },  // Always zero when using 2D-lidar

        { "lidar.distance", lidar_distance, TE_DEFAULT, customFuncHandler },
        { "lidar.distance_indexed", lidar_distance_indexed, TE_DEFAULT, customFuncHandler },

        { "lidar.angle.horizontal", lidar_angle_horizontal, TE_DEFAULT, customFuncHandler },
        { "lidar.angle.vertical", lidar_angle_vertical, TE_DEFAULT, customFuncHandler },    // Always zero when using 2D-lidar
        { "lidar.angle_indexed.horizontal", lidar_angle_indexed_horizontal, TE_DEFAULT, customFuncHandler },
        { "lidar.angle_indexed.vertical", lidar_angle_indexed_vertical, TE_DEFAULT, customFuncHandler },    // Always zero when using 2D-lidar

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

    return customFunctions;
}





}; // namespace PointFilter



















