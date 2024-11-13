/*
    expressionfilter.h (part of GNSS-Stylus)
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

#ifndef EXPRESSIONFILTER_H
#define EXPRESSIONFILTER_H

#include <QString>
#include <QVector>
#include <Eigen/Geometry>
#include "lazyevaluator.h"
#include "tinyexpr-plusplus/tinyexpr.h"
#include "livoxmid360pointcloudandimudata.h"
#include "ConvexHull/convexhull.h"

namespace PointFilter
{

class ExpressionFilter;

class TinyExprCustomFuncHandler : public te_expr
{
public:
    explicit TinyExprCustomFuncHandler(const te_variable_flags type, ExpressionFilter* const filter) noexcept :
        te_expr(type) { this->filter = filter; }

    inline te_type lidar_coord_x() const;
    inline te_type lidar_coord_indexed_x(te_type pointIndex) const;
    inline te_type lidar_coord_y() const;
    inline te_type lidar_coord_indexed_y(te_type pointIndex) const;
    inline te_type lidar_coord_z() const;
    inline te_type lidar_coord_indexed_z(te_type pointIndex) const;

    inline te_type lidar_properties() const;
    inline te_type lidar_properties_indexed(te_type pointIndex) const;
    inline te_type lidar_properties_other() const;
    inline te_type lidar_properties_other_indexed(te_type pointIndex) const;
    inline te_type lidar_properties_dust() const;
    inline te_type lidar_properties_dust_indexed(te_type pointIndex) const;
    inline te_type lidar_properties_glue() const;
    inline te_type lidar_properties_glue_indexed(te_type pointIndex) const;
    inline te_type lidar_reflectivity() const;
    inline te_type lidar_reflectivity_indexed(te_type pointIndex) const;
    inline te_type lidar_distance() const;
    inline te_type lidar_distance_indexed(te_type pointIndex) const;

    inline te_type rig_coord_x() const;
    inline te_type rig_coord_indexed_x(te_type pointIndex) const;
    inline te_type rig_coord_y() const;
    inline te_type rig_coord_indexed_y(te_type pointIndex) const;
    inline te_type rig_coord_z() const;
    inline te_type rig_coord_indexed_z(te_type pointIndex) const;

    inline te_type ned_coord_x() const;
    inline te_type ned_coord_indexed_x(te_type pointIndex) const;
    inline te_type ned_coord_y() const;
    inline te_type ned_coord_indexed_y(te_type pointIndex) const;
    inline te_type ned_coord_z() const;
    inline te_type ned_coord_indexed_z(te_type pointIndex) const;

    inline te_type lidar_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type lidar_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;
    inline te_type rig_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type rig_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;
    inline te_type ned_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type ned_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;

private:
    ExpressionFilter* filter;
};

class ExpressionFilter
{
public:
    struct ConvexHullFilter
    {
        QString Name;
        ConvexHull::Filter filter;
    };

    ExpressionFilter();
    ~ExpressionFilter();

    static const unsigned int bufferLength = 16;

    class OutItem
    {
    public:
        bool valid;
        double filterResult;
        unsigned int uptime_ms;
        Eigen::Vector3d coords;
        double quality;
    };

    void initBuffer(void);
    bool setExpression_Filter(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);
    bool setExpression_Quality(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);
    void setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    void setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    bool setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters);

    void addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms);
    bool getFilteredPoint(OutItem& outPoint);

private:
    class BufferItem
    {
    public:
        LivoxMid360::PointCloudData::Point point_Lidar_Source;
        Eigen::Vector3d lidarSourceVector;
        LazyEvaluator point_Lidar;
        LazyEvaluator point_Rig;
        LazyEvaluator point_NED;
        int uptime_ms;
    };

    void setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters = QVector<ConvexHullFilter>());

    QString expression_Filter;
    te_parser parser_Filter;

    QString expression_Quality;
    te_parser parser_Quality;

    BufferItem buffer[bufferLength];
    unsigned int bufferIndex;

    Eigen::Transform<double, 3, Eigen::Affine> transformCache_LidarToRig[256];
    unsigned char transformCacheIndex_LidarToRig;

    Eigen::Transform<double, 3, Eigen::Affine> transformCache_RigToNED[256];
    unsigned char transformCacheIndex_RigToNED;

    TinyExprCustomFuncHandler* customFuncHandler;

    QVector<te_type> convexHullFilterIndexes; // These are needed for tinyexpr++ ("chull_???"-functions need pointers to te_types)
    unsigned int numOfConvexHullFilters; // For speedup.
    QVector<ConvexHull::Filter> convexHullFilters;

    friend class TinyExprCustomFuncHandler;
};

inline te_type TinyExprCustomFuncHandler::lidar_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.x;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.x;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.y;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.y;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.z;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.z;
}

inline te_type TinyExprCustomFuncHandler::lidar_properties() const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.properties;
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_indexed(te_type pointIndex) const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.properties;
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_other() const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.getProperties_other();
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_other_indexed(te_type pointIndex) const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.getProperties_other();
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_dust() const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.getProperties_dust();
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_dust_indexed(te_type pointIndex) const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.getProperties_dust();
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_glue() const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.getProperties_glue();
}

inline te_type TinyExprCustomFuncHandler::lidar_properties_glue_indexed(te_type pointIndex) const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.getProperties_glue();
}

inline te_type TinyExprCustomFuncHandler::lidar_reflectivity() const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar_Source.reflectivity;
}

inline te_type TinyExprCustomFuncHandler::lidar_reflectivity_indexed(te_type pointIndex) const
{
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar_Source.reflectivity;
}

inline te_type TinyExprCustomFuncHandler::lidar_distance() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar.getDistance();
}

inline te_type TinyExprCustomFuncHandler::lidar_distance_indexed(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar.getDistance();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Rig.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Rig.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Rig.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Rig.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Rig.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Rig.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_NED.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_NED.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_NED.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_NED.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_NED.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_NED.getTransformedVector().z();
}


inline te_type TinyExprCustomFuncHandler::lidar_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Lidar.getTransformedVector(), margin);
}


inline te_type TinyExprCustomFuncHandler::lidar_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Lidar.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::rig_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_Rig.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::rig_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_Rig.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::ned_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].point_NED.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::ned_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength].point_NED.getTransformedVector(), margin);
}







}; // namespace PointFilter
#endif // EXPRESSIONFILTER_H
