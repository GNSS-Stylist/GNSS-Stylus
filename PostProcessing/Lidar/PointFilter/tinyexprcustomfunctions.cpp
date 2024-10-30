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

te_type lidar_coord_y(const te_expr* context)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_y());
}


te_type lidar_coord_x_indexed(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const TinyExprCustomFuncHandler*>(context);
    return static_cast<te_type>(c->lidar_coord_x_indexed(a));
}

} // namespace PointFilter
