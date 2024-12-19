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

class ExpressionFilter_Base : public te_expr
{
public:
    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    ExpressionFilter_Base();
    ~ExpressionFilter_Base();

    static constexpr unsigned int MAX_NUM_OF_CONVEX_HULL_FILTERS = 256;

    ExpressionFilter_Base(const ExpressionFilter_Base&);

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

    void setExpression_Filter(const QString newExpression);
    QString getExpression_Filter(void) { return expression_Filter; };
    void setExpression_Quality(const QString newExpression);
    QString getExpression_Quality(void) { return expression_Quality; };
    void setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    void setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    bool setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters);
    bool getFilteredPoint(OutItem& outPoint);
    virtual void initBuffer(void) = 0;

protected:

    static void copyFields(const ExpressionFilter_Base& source, ExpressionFilter_Base& dest);

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
    virtual void setCustomVariablesAndFunctions(void) = 0;

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

    unsigned int numOfConvexHullFilters; // For speedup.
    QVector<ConvexHullFilter> convexHullFilters;

    std::set<te_variable> getCommonCustomFunctions(void);

private:
    inline static te_type convexHullFilterIndexes[MAX_NUM_OF_CONVEX_HULL_FILTERS]; // These are needed for tinyexpr++ ("chull_???"-functions need pointers to te_types)
    inline static bool convexHullFilterIndexesInitialized;

    void setExpression(te_parser& parser, const QString newExpression);

    ExpressionFilter_Base::BufferItem getCurrentBufferItem(void) const
    { return buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength]; };

    ExpressionFilter_Base::BufferItem getIndexedBufferItem(const te_type& pointIndex) const
    { return buffer[(bufferIndex - (bufferLength / 2) - 1 + int(pointIndex)) % bufferLength]; };

// Handlers for functions that can be used in expressions:
    inline te_type exprfunc_lidar_coord_x() const;
    inline te_type exprfunc_lidar_coord_indexed_x(te_type pointIndex) const;
    inline te_type exprfunc_lidar_coord_y() const;
    inline te_type exprfunc_lidar_coord_indexed_y(te_type pointIndex) const;
    inline te_type exprfunc_lidar_coord_z() const;
    inline te_type exprfunc_lidar_coord_indexed_z(te_type pointIndex) const;

    inline te_type exprfunc_lidar_distance() const;
    inline te_type exprfunc_lidar_distance_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_angle_horizontal() const;
    inline te_type exprfunc_lidar_angle_horizontal_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_angle_vertical() const;
    inline te_type exprfunc_lidar_angle_vertical_indexed(te_type pointIndex) const;

    inline te_type exprfunc_rig_coord_x() const;
    inline te_type exprfunc_rig_coord_indexed_x(te_type pointIndex) const;
    inline te_type exprfunc_rig_coord_y() const;
    inline te_type exprfunc_rig_coord_indexed_y(te_type pointIndex) const;
    inline te_type exprfunc_rig_coord_z() const;
    inline te_type exprfunc_rig_coord_indexed_z(te_type pointIndex) const;

    inline te_type exprfunc_ned_coord_x() const;
    inline te_type exprfunc_ned_coord_indexed_x(te_type pointIndex) const;
    inline te_type exprfunc_ned_coord_y() const;
    inline te_type exprfunc_ned_coord_indexed_y(te_type pointIndex) const;
    inline te_type exprfunc_ned_coord_z() const;
    inline te_type exprfunc_ned_coord_indexed_z(te_type pointIndex) const;

    inline te_type exprfunc_lidar_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type exprfunc_lidar_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;
    inline te_type exprfunc_rig_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type exprfunc_rig_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;
    inline te_type exprfunc_ned_in_convex_hull(te_type hullIndex, te_type margin) const;
    inline te_type exprfunc_ned_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const;

    inline te_type exprfunc_lidar_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type exprfunc_lidar_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;
    inline te_type exprfunc_rig_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type exprfunc_rig_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;
    inline te_type exprfunc_ned_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const;
    inline te_type exprfunc_ned_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const;

    inline te_type exprfunc_lidar_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type exprfunc_lidar_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;
    inline te_type exprfunc_rig_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type exprfunc_rig_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;
    inline te_type exprfunc_ned_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const;
    inline te_type exprfunc_ned_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const;

    // Livox Mid-360:
    inline te_type exprfunc_lidar_mid360_properties() const;
    inline te_type exprfunc_lidar_mid360_properties_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_mid360_properties_other() const;
    inline te_type exprfunc_lidar_mid360_properties_other_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_mid360_properties_dust() const;
    inline te_type exprfunc_lidar_mid360_properties_dust_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_mid360_properties_glue() const;
    inline te_type exprfunc_lidar_mid360_properties_glue_indexed(te_type pointIndex) const;
    inline te_type exprfunc_lidar_mid360_reflectivity() const;
    inline te_type exprfunc_lidar_mid360_reflectivity_indexed(te_type pointIndex) const;

    // RPLidar:
    inline te_type exprfunc_lidar_rplidar_quality() const;
    inline te_type exprfunc_lidar_rplidar_quality_indexed(te_type pointIndex) const;

    // Handler functions (in global scope) need to be our friends to allow them to manipulate our private parts:
    friend te_type lidar_coord_x(const te_expr* context);
    friend te_type lidar_coord_indexed_x(const te_expr* context, te_type pointIndex);
    friend te_type lidar_coord_y(const te_expr* context);
    friend te_type lidar_coord_indexed_y(const te_expr* context, te_type pointIndex);
    friend te_type lidar_coord_z(const te_expr* context);
    friend te_type lidar_coord_indexed_z(const te_expr* context, te_type pointIndex);
    friend te_type lidar_properties(const te_expr* context);
    friend te_type lidar_properties_indexed(const te_expr* context, te_type pointIndex);
    friend te_type lidar_properties_other(const te_expr* context);
    friend te_type lidar_properties_indexed_other(const te_expr* context, te_type pointIndex);
    friend te_type lidar_properties_dust(const te_expr* context);
    friend te_type lidar_properties_indexed_dust(const te_expr* context, te_type pointIndex);
    friend te_type lidar_properties_glue(const te_expr* context);
    friend te_type lidar_properties_indexed_glue(const te_expr* context, te_type pointIndex);
    friend te_type lidar_reflectivity(const te_expr* context);
    friend te_type lidar_reflectivity_indexed(const te_expr* context, te_type pointIndex);
    friend te_type lidar_distance(const te_expr* context);
    friend te_type lidar_distance_indexed(const te_expr* context, te_type pointIndex);
    friend te_type lidar_angle_horizontal(const te_expr* context);
    friend te_type lidar_angle_indexed_horizontal(const te_expr* context, te_type pointIndex);
    friend te_type lidar_angle_vertical(const te_expr* context);
    friend te_type lidar_angle_indexed_vertical(const te_expr* context, te_type pointIndex);
    friend te_type rig_coord_x(const te_expr* context);
    friend te_type rig_coord_indexed_x(const te_expr* context, te_type pointIndex);
    friend te_type rig_coord_y(const te_expr* context);
    friend te_type rig_coord_indexed_y(const te_expr* context, te_type pointIndex);
    friend te_type rig_coord_z(const te_expr* context);
    friend te_type rig_coord_indexed_z(const te_expr* context, te_type pointIndex);
    friend te_type ned_coord_x(const te_expr* context);
    friend te_type ned_coord_indexed_x(const te_expr* context, te_type pointIndex);
    friend te_type ned_coord_y(const te_expr* context);
    friend te_type ned_coord_indexed_y(const te_expr* context, te_type pointIndex);
    friend te_type ned_coord_z(const te_expr* context);
    friend te_type ned_coord_indexed_z(const te_expr* context, te_type pointIndex);
    friend te_type lidar_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin);
    friend te_type lidar_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex);
    friend te_type rig_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin);
    friend te_type rig_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex);
    friend te_type ned_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin);
    friend te_type ned_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex);
    friend te_type lidar_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ);
    friend te_type lidar_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex);
    friend te_type rig_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ);
    friend te_type rig_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex);
    friend te_type ned_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ);
    friend te_type ned_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex);
    friend te_type lidar_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance);
    friend te_type lidar_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex);
    friend te_type rig_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance);
    friend te_type rig_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex);
    friend te_type ned_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance);
    friend te_type ned_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex);
    friend te_type lidar_rplidar_quality(const te_expr* context);
    friend te_type lidar_rplidar_quality_indexed(const te_expr* context, te_type pointIndex);
};


inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_x() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->x();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->x();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_y() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->y();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->y();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_z() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getSourceVectorPtr()->z();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getSourceVectorPtr()->z();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_distance() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getDistance();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_distance_indexed(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getDistance();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_angle_horizontal() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getHorizontalAngle();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_angle_horizontal_indexed(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getHorizontalAngle();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_angle_vertical() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Lidar.getVerticalAngle();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_angle_vertical_indexed(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Lidar.getVerticalAngle();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_x() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().x();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().x();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_y() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().y();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().y();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_z() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_Rig.getTransformedVector().z();
}

inline te_type ExpressionFilter_Base::exprfunc_rig_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector().z();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_x() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().x();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_indexed_x(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().x();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_y() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().y();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_indexed_y(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().y();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_z() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().point_NED.getTransformedVector().z();
}

inline te_type ExpressionFilter_Base::exprfunc_ned_coord_indexed_z(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).point_NED.getTransformedVector().z();
}


inline te_type ExpressionFilter_Base::exprfunc_lidar_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getCurrentBufferItem().point_Lidar.getTransformedVector(), margin);
}


inline te_type ExpressionFilter_Base::exprfunc_lidar_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getIndexedBufferItem(pointIndex).point_Lidar.getTransformedVector(), margin);
}

inline te_type ExpressionFilter_Base::exprfunc_rig_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getCurrentBufferItem().point_Rig.getTransformedVector(), margin);
}

inline te_type ExpressionFilter_Base::exprfunc_rig_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector(), margin);
}

inline te_type ExpressionFilter_Base::exprfunc_ned_in_convex_hull(te_type hullIndex, te_type margin) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getCurrentBufferItem().point_NED.getTransformedVector(), margin);
}

inline te_type ExpressionFilter_Base::exprfunc_ned_in_convex_hull_indexed(te_type hullIndex, te_type margin, te_type pointIndex) const
{
    int intHullIndex = hullIndex;

    if ((intHullIndex >= (int)numOfConvexHullFilters) || (intHullIndex < 0))
    {
        return false;
    }

    return convexHullFilters[intHullIndex].filter.isInside(getIndexedBufferItem(pointIndex).point_NED.getTransformedVector(), margin);
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
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

inline te_type ExpressionFilter_Base::exprfunc_lidar_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
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

inline te_type ExpressionFilter_Base::exprfunc_rig_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
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

inline te_type ExpressionFilter_Base::exprfunc_rig_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
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

inline te_type ExpressionFilter_Base::exprfunc_ned_in_aabb(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ) const
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

inline te_type ExpressionFilter_Base::exprfunc_ned_in_aabb_indexed(te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex) const
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

inline te_type ExpressionFilter_Base::exprfunc_lidar_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Lidar.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Lidar.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type ExpressionFilter_Base::exprfunc_rig_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_Rig.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type ExpressionFilter_Base::exprfunc_rig_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_Rig.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type ExpressionFilter_Base::exprfunc_ned_in_sphere(te_type centerX, te_type centerY, te_type centerZ, te_type radius) const
{
    Eigen::Vector3d point = getCurrentBufferItem().point_NED.getTransformedVector();
    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);

    return ((point-center).squaredNorm() <= (radius * radius));
}

inline te_type ExpressionFilter_Base::exprfunc_ned_in_sphere_indexed(te_type centerX, te_type centerY, te_type centerZ, te_type radius, te_type pointIndex) const
{
    Eigen::Vector3d point = getIndexedBufferItem(pointIndex).point_NED.getTransformedVector();

    Eigen::Vector3d center = Eigen::Vector3d(centerX, centerY, centerZ);
    return ((point-center).squaredNorm() <= (radius * radius));
}

// Livox Mid-360:
inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties() const
{
    return getCurrentBufferItem().point_Mid360_Source.properties;
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.properties;
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_other() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_other();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_other_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_other();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_dust() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_dust();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_dust_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_dust();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_glue() const
{
    return getCurrentBufferItem().point_Mid360_Source.getProperties_glue();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_properties_glue_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.getProperties_glue();
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_reflectivity() const
{
    return getCurrentBufferItem().point_Mid360_Source.reflectivity;
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_mid360_reflectivity_indexed(te_type pointIndex) const
{
    return getIndexedBufferItem(pointIndex).point_Mid360_Source.reflectivity;
}


// RPLidar:
inline te_type ExpressionFilter_Base::exprfunc_lidar_rplidar_quality() const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getCurrentBufferItem().quality_RPLidar;
}

inline te_type ExpressionFilter_Base::exprfunc_lidar_rplidar_quality_indexed(te_type pointIndex) const
{
    Q_ASSERT(bufferIndex >= bufferLength);
    return getIndexedBufferItem(pointIndex).quality_RPLidar;
}


}; // namespace PointFilter
#endif // EXPRESSIONFILTER_BASE_H
