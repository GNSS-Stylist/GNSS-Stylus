#include "tinyexprcustomfunctions.h"
#include "expressionfilter.h"
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

te_type lidar_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_x());
}

te_type lidar_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_x(pointIndex));
}

te_type lidar_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_y());
}

te_type lidar_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_y(pointIndex));
}

te_type lidar_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_z());
}

te_type lidar_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_z(pointIndex));
}

te_type lidar_properties(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties());
}

te_type lidar_properties_indexed(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_indexed(pointIndex));
}

te_type lidar_properties_other(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_other());
}

te_type lidar_properties_indexed_other(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_other_indexed(pointIndex));
}

te_type lidar_properties_dust(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_dust());
}

te_type lidar_properties_indexed_dust(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_dust_indexed(pointIndex));
}

te_type lidar_properties_glue(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_glue());
}

te_type lidar_properties_indexed_glue(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_glue_indexed(pointIndex));
}

te_type lidar_reflectivity(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_reflectivity());
}

te_type lidar_reflectivity_indexed(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_reflectivity_indexed(pointIndex));
}

te_type lidar_distance(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_distance());
}

te_type lidar_distance_indexed(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_distance_indexed(pointIndex));
}

te_type rig_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_x());
}

te_type rig_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_x(pointIndex));
}

te_type rig_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_y());
}

te_type rig_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_y(pointIndex));
}

te_type rig_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_z());
}

te_type rig_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_z(pointIndex));
}

te_type ned_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_x());
}

te_type ned_coord_indexed_x(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_x(pointIndex));
}

te_type ned_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_y());
}

te_type ned_coord_indexed_y(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_y(pointIndex));
}

te_type ned_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_z());
}

te_type ned_coord_indexed_z(const te_expr* context, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_z(pointIndex));
}


te_type lidar_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_in_convex_hull(hullIndex, margin));
}


te_type lidar_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_in_convex_hull_indexed(hullIndex, pointIndex, margin));
}

te_type rig_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_in_convex_hull(hullIndex, margin));
}

te_type rig_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_in_convex_hull_indexed(hullIndex, pointIndex, margin));
}

te_type ned_in_convex_hull(const te_expr* context, te_type hullIndex, te_type margin)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_in_convex_hull(hullIndex, margin));
}

te_type ned_in_convex_hull_indexed(const te_expr* context, te_type hullIndex, te_type margin, te_type pointIndex)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_in_convex_hull_indexed(hullIndex, pointIndex, margin));
}


} // namespace PointFilter
