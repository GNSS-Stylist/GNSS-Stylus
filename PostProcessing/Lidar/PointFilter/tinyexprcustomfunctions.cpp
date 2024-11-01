#include "tinyexprcustomfunctions.h"
#include "expressionfilter.h"
#include "qmath.h"

namespace PointFilter{

te_type rad_to_deg(te_type a)
{
    return qRadiansToDegrees(a);
}

te_type deg_to_rad(te_type a)
{
    return qDegreesToRadians(a);
}

te_type lidar_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_x());
}

te_type lidar_coord_indexed_x(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_x(a));
}

te_type lidar_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_y());
}

te_type lidar_coord_indexed_y(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_y(a));
}

te_type lidar_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_z());
}

te_type lidar_coord_indexed_z(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_indexed_z(a));
}

te_type lidar_properties(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties());
}

te_type lidar_properties_indexed(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_indexed(a));
}

te_type lidar_properties_other(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_other());
}

te_type lidar_properties_indexed_other(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_other_indexed(a));
}

te_type lidar_properties_dust(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_dust());
}

te_type lidar_properties_indexed_dust(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_dust_indexed(a));
}

te_type lidar_properties_glue(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_glue());
}

te_type lidar_properties_indexed_glue(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_properties_glue_indexed(a));
}

te_type lidar_reflectivity(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_reflectivity());
}

te_type lidar_reflectivity_indexed(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_reflectivity_indexed(a));
}

te_type lidar_distance(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_distance());
}

te_type lidar_distance_indexed(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_distance_indexed(a));
}

te_type rig_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_x());
}

te_type rig_coord_indexed_x(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_x(a));
}

te_type rig_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_y());
}

te_type rig_coord_indexed_y(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_y(a));
}

te_type rig_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_z());
}

te_type rig_coord_indexed_z(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->rig_coord_indexed_z(a));
}

te_type ned_coord_x(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_x());
}

te_type ned_coord_indexed_x(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_x(a));
}

te_type ned_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_y());
}

te_type ned_coord_indexed_y(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_y(a));
}

te_type ned_coord_z(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_z());
}

te_type ned_coord_indexed_z(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->ned_coord_indexed_z(a));
}


} // namespace PointFilter
