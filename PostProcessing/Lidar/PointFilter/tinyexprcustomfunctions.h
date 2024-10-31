#ifndef TINYEXPRCUSTOMFUNCTIONS_H
#define TINYEXPRCUSTOMFUNCTIONS_H

#include "tinyexpr-plusplus/tinyexpr.h"

namespace PointFilter{

te_type rad_to_deg(te_type a);
te_type deg_to_rad(te_type a);

te_type lidar_coord_x(const te_expr* context);
te_type lidar_coord_indexed_x(const te_expr* context, te_type a);
te_type lidar_coord_y(const te_expr* context);
te_type lidar_coord_indexed_y(const te_expr* context, te_type a);
te_type lidar_coord_z(const te_expr* context);
te_type lidar_coord_indexed_z(const te_expr* context, te_type a);

te_type ned_coord_x(const te_expr* context);
te_type ned_coord_indexed_x(const te_expr* context, te_type a);
te_type ned_coord_y(const te_expr* context);
te_type ned_coord_indexed_y(const te_expr* context, te_type a);
te_type ned_coord_z(const te_expr* context);
te_type ned_coord_indexed_z(const te_expr* context, te_type a);

te_type rig_coord_x(const te_expr* context);
te_type rig_coord_indexed_x(const te_expr* context, te_type a);
te_type rig_coord_y(const te_expr* context);
te_type rig_coord_indexed_y(const te_expr* context, te_type a);
te_type rig_coord_z(const te_expr* context);
te_type rig_coord_indexed_z(const te_expr* context, te_type a);

te_type lidar_distance(const te_expr* context);
te_type lidar_distance_indexed(const te_expr* context, te_type a);














/* te_type rad_to_deg(const te_expr* context, te_type a)
{
    auto* c = dynamic_cast<const ExpressionFilter*>(context);
    return static_cast<te_type>(c->m_data[static_cast<size_t>(a)]);
}
*/

} // namespace PointFilter

#endif // TINYEXPRCUSTOMFUNCTIONS_H
