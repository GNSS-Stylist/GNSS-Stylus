#ifndef TINYEXPRCUSTOMFUNCTIONS_H
#define TINYEXPRCUSTOMFUNCTIONS_H

#include "tinyexpr-plusplus/tinyexpr.h"

namespace PointFilter{

te_type rad_to_deg(te_type radians);
te_type deg_to_rad(te_type degrees);

te_type lidar_coord_x(const te_expr* context);
te_type lidar_coord_indexed_x(const te_expr* context, te_type pointIndex);
te_type lidar_coord_y(const te_expr* context);
te_type lidar_coord_indexed_y(const te_expr* context, te_type pointIndex);
te_type lidar_coord_z(const te_expr* context);
te_type lidar_coord_indexed_z(const te_expr* context, te_type pointIndex);

te_type lidar_properties(const te_expr* context);
te_type lidar_properties_indexed(const te_expr* context, te_type pointIndex);
te_type lidar_properties_other(const te_expr* context);
te_type lidar_properties_indexed_other(const te_expr* context, te_type pointIndex);
te_type lidar_properties_dust(const te_expr* context);
te_type lidar_properties_indexed_dust(const te_expr* context, te_type pointIndex);
te_type lidar_properties_glue(const te_expr* context);
te_type lidar_properties_indexed_glue(const te_expr* context, te_type pointIndex);
te_type lidar_reflectivity(const te_expr* context);
te_type lidar_reflectivity_indexed(const te_expr* context, te_type pointIndex);
te_type lidar_distance(const te_expr* context);
te_type lidar_distance_indexed(const te_expr* context, te_type pointIndex);

te_type ned_coord_x(const te_expr* context);
te_type ned_coord_indexed_x(const te_expr* context, te_type pointIndex);
te_type ned_coord_y(const te_expr* context);
te_type ned_coord_indexed_y(const te_expr* context, te_type pointIndex);
te_type ned_coord_z(const te_expr* context);
te_type ned_coord_indexed_z(const te_expr* context, te_type pointIndex);

te_type rig_coord_x(const te_expr* context);
te_type rig_coord_indexed_x(const te_expr* context, te_type pointIndex);
te_type rig_coord_y(const te_expr* context);
te_type rig_coord_indexed_y(const te_expr* context, te_type pointIndex);
te_type rig_coord_z(const te_expr* context);
te_type rig_coord_indexed_z(const te_expr* context, te_type pointIndex);





} // namespace PointFilter

#endif // TINYEXPRCUSTOMFUNCTIONS_H
