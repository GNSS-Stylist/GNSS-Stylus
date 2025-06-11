/*
    tinyexprcustomfunctions.cpp (part of GNSS-Stylus)
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

#include "tinyexprcustomfunctions.h"
#include "expressionfilter_base.h"
#include "qmath.h"

namespace PointFilter{

te_type rad_to_deg(te_type radians)
{
    return qRadiansToDegrees(radians);
}

te_type deg_to_rad(te_type degrees)
{
    return qDegreesToRadians(degrees);
}

/* Note:
 * context-fields here are casted from const to non-const. This breaks many rules in C++,
 * but I found it just too difficult to implement "mutable" objects inherited from te_expr in tinyexpr++
 * (and I don't want to change it's source code). Sorry...
*/

te_type lidar_coord_x(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_x());
}

te_type lidar_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_indexed_x(pointIndex));
}

te_type lidar_coord_y(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_y());
}

te_type lidar_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_indexed_y(pointIndex));
}

te_type lidar_coord_z(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_z());
}

te_type lidar_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_coord_indexed_z(pointIndex));
}

te_type lidar_properties(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties());
}

te_type lidar_properties_indexed(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_indexed(pointIndex));
}

te_type lidar_properties_other(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_other());
}

te_type lidar_properties_indexed_other(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_other_indexed(pointIndex));
}

te_type lidar_properties_dust(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_dust());
}

te_type lidar_properties_indexed_dust(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_dust_indexed(pointIndex));
}

te_type lidar_properties_glue(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_glue());
}

te_type lidar_properties_indexed_glue(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_properties_glue_indexed(pointIndex));
}

te_type lidar_reflectivity(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_reflectivity());
}

te_type lidar_reflectivity_indexed(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_mid360_reflectivity_indexed(pointIndex));
}

te_type lidar_distance(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_distance());
}

te_type lidar_distance_indexed(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_distance_indexed(pointIndex));
}

te_type lidar_angle_horizontal(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_angle_horizontal());
}

te_type lidar_angle_indexed_horizontal(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_angle_horizontal_indexed(pointIndex));
}

te_type lidar_angle_vertical(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_angle_vertical());
}

te_type lidar_angle_indexed_vertical(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_angle_vertical_indexed(pointIndex));
}

te_type rig_coord_x(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_x());
}

te_type rig_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_indexed_x(pointIndex));
}

te_type rig_coord_y(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_y());
}

te_type rig_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_indexed_y(pointIndex));
}

te_type rig_coord_z(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_z());
}

te_type rig_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_coord_indexed_z(pointIndex));
}

te_type ned_coord_x(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_x());
}

te_type ned_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_indexed_x(pointIndex));
}

te_type ned_coord_y(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_y());
}

te_type ned_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_indexed_y(pointIndex));
}

te_type ned_coord_z(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_z());
}

te_type ned_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_coord_indexed_z(pointIndex));
}

te_type xyz_coord_x(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_x());
}

te_type xyz_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_indexed_x(pointIndex));
}

te_type xyz_coord_y(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_y());
}

te_type xyz_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_indexed_y(pointIndex));
}

te_type xyz_coord_z(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_z());
}

te_type xyz_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_coord_indexed_z(pointIndex));
}


te_type lidar_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_convex_hull(hullIndex, margin));
}


te_type lidar_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_convex_hull_indexed(hullIndex, margin, pointIndex));
}

te_type rig_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_convex_hull(hullIndex, margin));
}

te_type rig_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_convex_hull_indexed(hullIndex, margin, pointIndex));
}

te_type ned_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_convex_hull(hullIndex, margin));
}

te_type ned_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_convex_hull_indexed(hullIndex, margin, pointIndex));
}

te_type xyz_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_convex_hull(hullIndex, margin));
}

te_type xyz_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_convex_hull_indexed(hullIndex, margin, pointIndex));
}

te_type lidar_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_aabb(minX, minY, minZ, maxX, maxY, maxZ));
}

te_type lidar_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_aabb_indexed(minX, minY, minZ, maxX, maxY, maxZ, pointIndex));
}

te_type rig_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_aabb(minX, minY, minZ, maxX, maxY, maxZ));
}

te_type rig_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_aabb_indexed(minX, minY, minZ, maxX, maxY, maxZ, pointIndex));
}

te_type ned_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_aabb(minX, minY, minZ, maxX, maxY, maxZ));
}

te_type ned_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_aabb_indexed(minX, minY, minZ, maxX, maxY, maxZ, pointIndex));
}

te_type xyz_in_aabb(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_aabb(minX, minY, minZ, maxX, maxY, maxZ));
}

te_type xyz_in_aabb_indexed(const te_expr* context, te_type minX, te_type minY, te_type minZ, te_type maxX, te_type maxY, te_type maxZ, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_aabb_indexed(minX, minY, minZ, maxX, maxY, maxZ, pointIndex));
}


te_type lidar_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_sphere(centerX, centerY, centerZ, distance));
}

te_type lidar_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_in_sphere_indexed(centerX, centerY, centerZ, distance, pointIndex));
}

te_type rig_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_sphere(centerX, centerY, centerZ, distance));
}

te_type rig_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_rig_in_sphere_indexed(centerX, centerY, centerZ, distance, pointIndex));
}

te_type ned_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_sphere(centerX, centerY, centerZ, distance));
}

te_type ned_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_ned_in_sphere_indexed(centerX, centerY, centerZ, distance, pointIndex));
}

te_type xyz_in_sphere(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_sphere(centerX, centerY, centerZ, distance));
}

te_type xyz_in_sphere_indexed(const te_expr* context, te_type centerX, te_type centerY, te_type centerZ, te_type distance, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_xyz_in_sphere_indexed(centerX, centerY, centerZ, distance, pointIndex));
}

// RPLidar:

te_type lidar_rplidar_quality(const te_expr* context)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_rplidar_quality());
}

te_type lidar_rplidar_quality_indexed(const te_expr* context, te_type pointIndex)
{
    ExpressionFilter_Base* c = (ExpressionFilter_Base*)(context);
    return static_cast<te_type>(c->exprfunc_lidar_rplidar_quality_indexed(pointIndex));}


} // namespace PointFilter
