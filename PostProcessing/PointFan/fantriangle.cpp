/*
    fantriangle.cpp (part of GNSS-Stylus)
    Copyright (C) 2025-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "fantriangle.h"


FanTriangle::FanTriangle(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3)
{
    points_3D[0] = point1;
    points_3D[1] = point2;
    points_3D[2] = point3;

    points_2D[0] = Eigen::Vector2d(point1.x(), point1.y());
    points_2D[1] = Eigen::Vector2d(point2.x(), point2.y());
    points_2D[2] = Eigen::Vector2d(point3.x(), point3.y());

    edgeNormals_2D[0] = Eigen::Vector2d((points_2D[1].y() - points_2D[0].y()), -(points_2D[1].x() - points_2D[0].x())).normalized();
    edgeNormals_2D[1] = Eigen::Vector2d((points_2D[2].y() - points_2D[1].y()), -(points_2D[2].x() - points_2D[1].x())).normalized();
    edgeNormals_2D[2] = Eigen::Vector2d((points_2D[0].y() - points_2D[2].y()), -(points_2D[0].x() - points_2D[2].x())).normalized();

    normal_3D = ((point2 - point1).cross(point3 - point1)).normalized();

    planeEq_X = -normal_3D.x() / normal_3D.z();
    planeEq_Y = -normal_3D.y() / normal_3D.z();
}

double FanTriangle::getArea_2D(void)
{
    Eigen::Vector2d vec01 = points_2D[1] - points_2D[0];
    Eigen::Vector2d vec02 = points_2D[2] - points_2D[0];
    Eigen::Vector2d normalVec01 = Eigen::Vector2d(vec01.y(), -vec01.x()).normalized();

    return (vec01.norm()) * std::abs(vec02.dot(normalVec01)) * 0.5;
}
