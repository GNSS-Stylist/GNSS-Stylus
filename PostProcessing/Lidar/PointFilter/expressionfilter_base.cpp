/*
    expressionfilter_base.cpp (part of GNSS-Stylus)
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
#include "Util/textblockparser.h"

namespace PointFilter
{

ExpressionFilter_Base::ExpressionFilter_Base() : te_expr(TE_DEFAULT)
{
//    initBuffer();

    transformCacheIndex_LidarToRig = 0;
    transformCache_LidarToRig[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_RigToNED = 0;
    transformCache_RigToNED[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_NEDToXYZ = 0;
    transformCache_NEDToXYZ[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    if (!convexHullFilterIndexesInitialized)
    {
        for (int i = 0; i < 256; i++)
        {
            convexHullFilterIndexes[i] = te_type(i);
        }

        convexHullFilterIndexesInitialized = true;
    }

//    setCustomVariablesAndFunctions();

    setExpression_Filter("1");
    setExpression_Quality("1");
}

ExpressionFilter_Base::~ExpressionFilter_Base()
{
}

void ExpressionFilter_Base::copyFields(const ExpressionFilter_Base& source, ExpressionFilter_Base& dest)
{
    dest.expression_Filter = source.expression_Filter;
//    dest.parser_Filter = source.parser_Filter;
    dest.expression_Quality = source.expression_Quality;
//    dest.parser_Quality = source.parser_Quality;

    for (unsigned int i = 0; i < bufferLength; i++)
    {
        dest.buffer[i] = source.buffer[i];

        dest.buffer[i].point_Rig.setSourceEvaluator(&dest.buffer[i].point_Lidar);
        dest.buffer[i].point_NED.setSourceEvaluator(&dest.buffer[i].point_Rig);
        dest.buffer[i].point_XYZ.setSourceEvaluator(&dest.buffer[i].point_NED);
    }

    dest.bufferIndex = source.bufferIndex;

    for (unsigned int i = 0; i < sizeof(transformCache_LidarToRig) / sizeof(transformCache_LidarToRig[0]); i++)
    {
        dest.transformCache_LidarToRig[i] = source.transformCache_LidarToRig[i];
    }
    for (unsigned int i = 0; i < sizeof(transformCache_RigToNED) / sizeof(transformCache_RigToNED[0]); i++)
    {
        dest.transformCache_RigToNED[i] = source.transformCache_RigToNED[i];
    }
    for (unsigned int i = 0; i < sizeof(transformCache_NEDToXYZ) / sizeof(transformCache_NEDToXYZ[0]); i++)
    {
        dest.transformCache_NEDToXYZ[i] = source.transformCache_NEDToXYZ[i];
    }

    dest.transformCacheIndex_LidarToRig = source.transformCacheIndex_LidarToRig;
    dest.transformCacheIndex_RigToNED = source.transformCacheIndex_RigToNED;
    dest.transformCacheIndex_NEDToXYZ = source.transformCacheIndex_NEDToXYZ;

    dest.numOfConvexHullFilters = source.numOfConvexHullFilters;
    dest.convexHullFilters = source.convexHullFilters;
}

ExpressionFilter_Base::ExpressionFilter_Base(const ExpressionFilter_Base& source) : te_expr(TE_DEFAULT)
{
    copyFields(source, *this);
}

void ExpressionFilter_Base::setExpression(te_parser& parser, const QString newExpression)
{
    QByteArray expression_8bit;
    TextBlockParser::CommentState cState;

    for (int i = 0; i < newExpression.length(); i++)
    {
        bool inComment = TextBlockParser::isInComment(newExpression, i, cState);

        char character = newExpression.at(i).toLatin1();

        if (character == 0)
        {
            if (inComment)
            {
                character = '?';
            }
            else
            {
                Issue error;
                error.text = "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.";
                error.beginChar = i;
                error.endChar = i + 1;
                throw error;
            }
        }

        expression_8bit += character;
    }

    char* prevLocale = std::setlocale(LC_NUMERIC, "C");
    parser.compile(expression_8bit.constData());
    if (prevLocale)
    {
        setlocale(LC_NUMERIC, prevLocale);
    }

    if (!parser.success())
    {
        Issue error;
        QString qstrErrorMessage = QString::fromStdString(parser.get_last_error_message());

        if (qstrErrorMessage.isEmpty())
        {
            error.text = "TinyExpr error: (empty)";
        }
        else
        {
            error.text = qstrErrorMessage;
        }

        error.beginChar = parser.get_last_error_position();
        error.endChar = parser.get_last_error_position();
        throw error;
    }
}

void ExpressionFilter_Base::setExpression_Filter(const QString newExpression)
{
    expression_Filter = newExpression;
    return setExpression(parser_Filter, newExpression);
}

void ExpressionFilter_Base::setExpression_Quality(const QString newExpression)
{
    expression_Quality = newExpression;
    return setExpression(parser_Quality, newExpression);
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

void ExpressionFilter_Base::setTransform_NEDToXYZ(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_NEDToXYZ++;
    transformCache_NEDToXYZ[transformCacheIndex_NEDToXYZ] = newTransform;
}

bool ExpressionFilter_Base::setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    convexHullFilters = newConvexHullFilters;

    try
    {
        setCustomVariablesAndFunctions();
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

    outPoint.timestamp = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].timestamp;
    outPoint.coords = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].point_XYZ.getTransformedVector();

    return true;
}

std::set<te_variable> ExpressionFilter_Base::getCommonCustomFunctions(void)
{
    std::set<te_variable> customFunctions =
    {
        { "rad_to_deg", rad_to_deg, TE_PURE },
        { "deg_to_rad", deg_to_rad, TE_PURE },

        { "lidar.coord.x", lidar_coord_x, TE_DEFAULT, this },
        { "lidar.coord.y", lidar_coord_y, TE_DEFAULT, this },
        { "lidar.coord.z", lidar_coord_z, TE_DEFAULT, this },  // Always zero when using 2D-lidar

        { "lidar.coord_indexed.x", lidar_coord_indexed_x, TE_DEFAULT, this },
        { "lidar.coord_indexed.y", lidar_coord_indexed_y, TE_DEFAULT, this },
        { "lidar.coord_indexed.z", lidar_coord_indexed_z, TE_DEFAULT, this },  // Always zero when using 2D-lidar

        { "lidar.distance", lidar_distance, TE_DEFAULT, this },
        { "lidar.distance_indexed", lidar_distance_indexed, TE_DEFAULT, this },

        { "lidar.angle.horizontal", lidar_angle_horizontal, TE_DEFAULT, this },
        { "lidar.angle.vertical", lidar_angle_vertical, TE_DEFAULT, this },    // Always zero when using 2D-lidar
        { "lidar.angle_indexed.horizontal", lidar_angle_indexed_horizontal, TE_DEFAULT, this },
        { "lidar.angle_indexed.vertical", lidar_angle_indexed_vertical, TE_DEFAULT, this },    // Always zero when using 2D-lidar

        { "rig.coord.x", rig_coord_x, TE_DEFAULT, this },
        { "rig.coord.y", rig_coord_y, TE_DEFAULT, this },
        { "rig.coord.z", rig_coord_z, TE_DEFAULT, this },

        { "rig.coord_indexed.x", rig_coord_indexed_x, TE_DEFAULT, this },
        { "rig.coord_indexed.y", rig_coord_indexed_y, TE_DEFAULT, this },
        { "rig.coord_indexed.z", rig_coord_indexed_z, TE_DEFAULT, this },

        { "ned.coord.x", ned_coord_x, TE_DEFAULT, this },
        { "ned.coord.y", ned_coord_y, TE_DEFAULT, this },
        { "ned.coord.z", ned_coord_z, TE_DEFAULT, this },

        { "ned.coord_indexed.x", ned_coord_indexed_x, TE_DEFAULT, this },
        { "ned.coord_indexed.y", ned_coord_indexed_y, TE_DEFAULT, this },
        { "ned.coord_indexed.z", ned_coord_indexed_z, TE_DEFAULT, this },

        { "xyz.coord.x", xyz_coord_x, TE_DEFAULT, this },
        { "xyz.coord.y", xyz_coord_y, TE_DEFAULT, this },
        { "xyz.coord.z", xyz_coord_z, TE_DEFAULT, this },

        { "xyz.coord_indexed.x", xyz_coord_indexed_x, TE_DEFAULT, this },
        { "xyz.coord_indexed.y", xyz_coord_indexed_y, TE_DEFAULT, this },
        { "xyz.coord_indexed.z", xyz_coord_indexed_z, TE_DEFAULT, this },

        { "lidar.in_convex_hull", lidar_in_convex_hull, TE_DEFAULT, this },
        { "lidar.in_convex_hull_indexed", lidar_in_convex_hull_indexed, TE_DEFAULT, this },

        { "rig.in_convex_hull", rig_in_convex_hull, TE_DEFAULT, this },
        { "rig.in_convex_hull_indexed", rig_in_convex_hull_indexed, TE_DEFAULT, this },

        { "ned.in_convex_hull", ned_in_convex_hull, TE_DEFAULT, this },
        { "ned.in_convex_hull_indexed", ned_in_convex_hull_indexed, TE_DEFAULT, this },

        { "xyz.in_convex_hull", xyz_in_convex_hull, TE_DEFAULT, this },
        { "xyz.in_convex_hull_indexed", xyz_in_convex_hull_indexed, TE_DEFAULT, this },

        { "lidar.in_aabb", lidar_in_aabb, TE_DEFAULT, this },
        { "lidar.in_aabb_indexed", lidar_in_aabb_indexed, TE_DEFAULT, this },

        { "rig.in_aabb", rig_in_aabb, TE_DEFAULT, this },
        { "rig.in_aabb_indexed", rig_in_aabb_indexed, TE_DEFAULT, this },

        { "ned.in_aabb", ned_in_aabb, TE_DEFAULT, this },
        { "ned.in_aabb_indexed", ned_in_aabb_indexed, TE_DEFAULT, this },

        { "xyz.in_aabb", xyz_in_aabb, TE_DEFAULT, this },
        { "xyz.in_aabb_indexed", xyz_in_aabb_indexed, TE_DEFAULT, this },

        { "lidar.in_sphere", lidar_in_sphere, TE_DEFAULT, this },
        { "lidar.in_sphere_indexed", lidar_in_sphere_indexed, TE_DEFAULT, this },

        { "rig.in_sphere", rig_in_sphere, TE_DEFAULT, this },
        { "rig.in_sphere_indexed", rig_in_sphere_indexed, TE_DEFAULT, this },

        { "ned.in_sphere", ned_in_sphere, TE_DEFAULT, this },
        { "ned.in_sphere_indexed", ned_in_sphere_indexed, TE_DEFAULT, this },

        { "xyz.in_sphere", xyz_in_sphere, TE_DEFAULT, this },
        { "xyz.in_sphere_indexed", xyz_in_sphere_indexed, TE_DEFAULT, this },
    };

    numOfConvexHullFilters = convexHullFilters.size();

    QVector<QByteArray> convexHullFilterNames; // To keep strings alive while adding.

    for (unsigned int i = 0; i < numOfConvexHullFilters; i++)
    {
        convexHullFilterNames.push_back((QString("chull_") + convexHullFilters[i].Name).toLower().toLocal8Bit());

        te_variable newConstant { convexHullFilterNames[i].constData(), &convexHullFilterIndexes[i], TE_PURE };

        customFunctions.insert(newConstant);
    }

    return customFunctions;
}

}; // namespace PointFilter



















