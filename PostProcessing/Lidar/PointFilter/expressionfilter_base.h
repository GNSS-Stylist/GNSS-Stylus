/*
    expressionfilter_base.h (part of GNSS-Stylus)
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

#ifndef EXPRESSIONFILTER_BASE_H
#define EXPRESSIONFILTER_BASE_H

#include <QString>
#include <QVector>
#include <Eigen/Geometry>
#include "lazyevaluator.h"
#include "tinyexpr-plusplus/tinyexpr.h"
#include "livoxmid360pointcloudandimudata.h"
#include "ConvexHull/convexhull.h"

namespace PointFilter
{

class ExpressionFilter_Base
{
public:
    ExpressionFilter_Base();
    ~ExpressionFilter_Base();

    struct ConvexHullFilter
    {
        QString Name;
        ConvexHull::Filter filter;
    };

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

    bool setExpression_Filter(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);
    bool setExpression_Quality(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);
    void setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    void setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    bool setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters);
    bool getFilteredPoint(OutItem& outPoint);

protected:

    class BufferItem
    {
    public:
        // Livox Mid-360-related data
        // (Only used when data is from Mid-360)
        LivoxMid360::PointCloudData::Point point_Mid360_Source;
        Eigen::Vector3d lidarSourceVector3D;

        // RPLidar-related data
        // (Only used when data is from RPLidar
//        RPLidarThread::DistanceItem point_RPLidar;
        double horizontalAngle_RPLidar;
        double distance_RPLidar;
        double quality_RPLidar;

        LazyEvaluator point_Lidar;
        LazyEvaluator point_Rig;
        LazyEvaluator point_NED;
        int uptime_ms;
    };

//    virtual void initBuffer(void) = 0;
    virtual void setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters = QVector<ConvexHullFilter>()) = 0;

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

    class TinyExprCustomFuncHandler* customFuncHandler;

    te_type* convexHullFilterIndexes; // These are needed for tinyexpr++ ("chull_???"-functions need pointers to te_types)
    unsigned int numOfConvexHullFilters; // For speedup.
    QVector<ConvexHull::Filter> convexHullFilters;

    friend class TinyExprCustomFuncHandler;

    std::set<te_variable> getCommonCustomFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters);
};

class TinyExprCustomFuncHandler : public te_expr
{
public:
    explicit TinyExprCustomFuncHandler(const te_variable_flags type, ExpressionFilter_Base* const filter) noexcept :
        te_expr(type) { this->filter = filter; }

    ExpressionFilter_Base::BufferItem& getCurrentBufferItem(void) const
        { return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength]; };

    ExpressionFilter_Base::BufferItem& getIndexedBufferItem(const te_type& pointIndex) const
        { return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(pointIndex)) % filter->bufferLength]; };

    inline te_type lidar_coord_x() const;
    inline te_type lidar_coord_indexed_x(te_type pointIndex) const;
    inline te_type lidar_coord_y() const;
    inline te_type lidar_coord_indexed_y(te_type pointIndex) const;
    inline te_type lidar_coord_z() const;
    inline te_type lidar_coord_indexed_z(te_type pointIndex) const;

    inline te_type lidar_distance() const;
    inline te_type lidar_distance_indexed(te_type pointIndex) const;
    inline te_type lidar_angle_horizontal() const;
    inline te_type lidar_angle_horizontal_indexed(te_type pointIndex) const;
    inline te_type lidar_angle_vertical() const;
    inline te_type lidar_angle_vertical_indexed(te_type pointIndex) const;

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

    inline te_type lidar_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type lidar_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;
    inline te_type rig_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type rig_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;
    inline te_type ned_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type ned_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;

    inline te_type lidar_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type lidar_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;
    inline te_type rig_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type rig_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;
    inline te_type ned_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type ned_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;

// Livox Mid-360:
    inline te_type lidar_mid360_properties() const;
    inline te_type lidar_mid360_properties_indexed(te_type pointIndex) const;
    inline te_type lidar_mid360_properties_other() const;
    inline te_type lidar_mid360_properties_other_indexed(te_type pointIndex) const;
    inline te_type lidar_mid360_properties_dust() const;
    inline te_type lidar_mid360_properties_dust_indexed(te_type pointIndex) const;
    inline te_type lidar_mid360_properties_glue() const;
    inline te_type lidar_mid360_properties_glue_indexed(te_type pointIndex) const;
    inline te_type lidar_mid360_reflectivity() const;
    inline te_type lidar_mid360_reflectivity_indexed(te_type pointIndex) const;

// RPLidar:
    inline te_type lidar_rplidar_quality() const;
    inline te_type lidar_rplidar_quality_indexed(te_type pointIndex) const;
private:
    ExpressionFilter_Base* filter;
};


inline te_type TinyExprCustomFuncHandler::lidar_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->x();
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->x();
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->y();
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->y();
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->z();
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->z();
}

inline te_type TinyExprCustomFuncHandler::lidar_distance() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getDistance();
}

inline te_type TinyExprCustomFuncHandler::lidar_distance_indexed(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getDistance();
}

inline te_type TinyExprCustomFuncHandler::lidar_angle_horizontal() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getHorizontalAngle();
}

inline te_type TinyExprCustomFuncHandler::lidar_angle_horizontal_indexed(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getHorizontalAngle();
}

inline te_type TinyExprCustomFuncHandler::lidar_angle_vertical() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Lidar.getVerticalAngle();
}

inline te_type TinyExprCustomFuncHandler::lidar_angle_vertical_indexed(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getVerticalAngle();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::rig_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().x();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().y();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_z() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().z();
}

inline te_type TinyExprCustomFuncHandler::ned_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().z();
}


inline te_type TinyExprCustomFuncHandler::lidar_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getCurrentBufferItem().point_Lidar.getTransformedVector(), margin);
}


inline te_type TinyExprCustomFuncHandler::lidar_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getIndexedBufferItem(pointIndex).point_Lidar.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::rig_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getCurrentBufferItem().point_Rig.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::rig_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::ned_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getCurrentBufferItem().point_NED.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::ned_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)filter->numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return filter->convexHullFilters[intHullIndex].isInside(getIndexedBufferItem(pointIndex).point_NED.getTransformedVector(), margin);
}

inline te_type TinyExprCustomFuncHandler::lidar_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Lidar.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
           (point.y() >= minY) &&
           (point.y() <= maxY) &&
           (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::lidar_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Lidar.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
        (point.y() >= minY) &&
        (point.y() <= maxY) &&
        (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::rig_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Rig.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
        (point.y() >= minY) &&
        (point.y() <= maxY) &&
        (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::rig_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
        (point.y() >= minY) &&
        (point.y() <= maxY) &&
        (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::ned_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_NED.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
        (point.y() >= minY) &&
        (point.y() <= maxY) &&
        (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::ned_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_NED.getTransformedVector();
    return (
        (point.x() >= minX) &&
        (point.x() <= maxX) &&
        (point.y() >= minY) &&
        (point.y() <= maxY) &&
        (point.z() >= minZ) &&
        (point.z() <= maxZ));
}

inline te_type TinyExprCustomFuncHandler::lidar_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Lidar.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type TinyExprCustomFuncHandler::lidar_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Lidar.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type TinyExprCustomFuncHandler::rig_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Rig.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type TinyExprCustomFuncHandler::rig_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type TinyExprCustomFuncHandler::ned_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_NED.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type TinyExprCustomFuncHandler::ned_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_NED.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

// Livox Mid-360:
inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties() const
{
    return getCurrentBufferItem().point_Mid360_Source.properties;
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.properties;
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_other() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_other();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_other_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_other();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_dust() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_dust();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_dust_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_dust();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_glue() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_glue();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_properties_glue_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_glue();
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_reflectivity() const
{
    return getCurrentBufferItem().point_Mid360_Source.reflectivity;
}

inline te_type TinyExprCustomFuncHandler::lidar_mid360_reflectivity_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.reflectivity;
}


// RPLidar:
inline te_type TinyExprCustomFuncHandler::lidar_rplidar_quality() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getCurrentBufferItem().quality_RPLidar;
}

inline te_type TinyExprCustomFuncHandler::lidar_rplidar_quality_indexed(te_type pointIndex) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return getIndexedBufferItem(pointIndex).quality_RPLidar;
}


}; // namespace PointFilter
#endif // EXPRESSIONFILTER_BASE_H
