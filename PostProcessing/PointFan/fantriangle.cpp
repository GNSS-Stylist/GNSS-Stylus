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


FanTriangle::FanTriangle(const Eigen::Vector3d& vertex0, const Eigen::Vector3d& vertex1, const Eigen::Vector3d& vertex2)
{
    vertices_3D[0] = vertex0;
    vertices_3D[1] = vertex1;
    vertices_3D[2] = vertex2;

    recalcSpeedupVariables();
}

void FanTriangle::recalcSpeedupVariables(void)
{
    vertices_2D[0] = Eigen::Vector2d(vertices_3D[0].x(), vertices_3D[0].y());
    vertices_2D[1] = Eigen::Vector2d(vertices_3D[1].x(), vertices_3D[1].y());
    vertices_2D[2] = Eigen::Vector2d(vertices_3D[2].x(), vertices_3D[2].y());

    edgeNormals_2D[0] = Eigen::Vector2d((vertices_2D[1].y() - vertices_2D[0].y()), -(vertices_2D[1].x() - vertices_2D[0].x())).normalized();
    edgeNormals_2D[1] = Eigen::Vector2d((vertices_2D[2].y() - vertices_2D[1].y()), -(vertices_2D[2].x() - vertices_2D[1].x())).normalized();
    edgeNormals_2D[2] = Eigen::Vector2d((vertices_2D[0].y() - vertices_2D[2].y()), -(vertices_2D[0].x() - vertices_2D[2].x())).normalized();

    normal_3D = ((vertices_3D[1] - vertices_3D[0]).cross(vertices_3D[2] - vertices_3D[0])).normalized();

    planeEq_X = -normal_3D.x() / normal_3D.z();
    planeEq_Y = -normal_3D.y() / normal_3D.z();
}

double FanTriangle::getArea_2D(void)
{
    Eigen::Vector2d vec01 = vertices_2D[1] - vertices_2D[0];
    Eigen::Vector2d vec02 = vertices_2D[2] - vertices_2D[0];
    Eigen::Vector2d normalVec01 = Eigen::Vector2d(vec01.y(), -vec01.x()).normalized();

    return (vec01.norm()) * std::abs(vec02.dot(normalVec01)) * 0.5;
}

FanTriangle operator*(const Eigen::Transform<double, 3, Eigen::Isometry>& transform , const FanTriangle& rhs)
{
    FanTriangle transformed(transform * rhs.vertices_3D[0], transform * rhs.vertices_3D[2], transform * rhs.vertices_3D[1]);
    return transformed;
}
