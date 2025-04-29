/*
    fantriangle.h (part of GNSS-Stylus)
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

#ifndef FANTRIANGLE_H
#define FANTRIANGLE_H

#include <QVector>
#include <Eigen/Geometry>

class FanTriangle
{
public:
    static constexpr double edgeLimit = 1e-6;

    FanTriangle(const Eigen::Vector3d& vertex0, const Eigen::Vector3d& vertex1, const Eigen::Vector3d& vertex2);
    inline bool testHit_2D(const Eigen::Vector2d& coord2D);
    inline Eigen::Vector3d getHitPoint(const Eigen::Vector2d& coord2D);
	Eigen::Vector3d getNormal(void) { return normal_3D; }
    double getArea_2D(void);
    const Eigen::Vector3d* getVertices(void) { return vertices_3D; };

    friend FanTriangle operator*(const Eigen::Transform<double, 3, Eigen::Affine>& transform , const FanTriangle& rhs);

private:
    Eigen::Vector3d vertices_3D[3];
    Eigen::Vector2d vertices_2D[3];

    Eigen::Vector2d edgeNormals_2D[3];
	Eigen::Vector3d normal_3D;
    double planeEq_X;  // "Slope" of z-coord along the x-axis
    double planeEq_Y;  // "Slope" of z-coord along the y-axis

    void recalcSpeedupVariables(void);
};

inline bool FanTriangle::testHit_2D(const Eigen::Vector2d& coord2D)
{
    if (normal_3D.z() < 0)
    {
        return (edgeNormals_2D[0].dot(coord2D - vertices_2D[0]) > -edgeLimit) &&
               (edgeNormals_2D[1].dot(coord2D - vertices_2D[1]) > -edgeLimit) &&
               (edgeNormals_2D[2].dot(coord2D - vertices_2D[2]) > -edgeLimit);
    }
    else
    {
        return (edgeNormals_2D[0].dot(coord2D - vertices_2D[0]) < edgeLimit) &&
               (edgeNormals_2D[1].dot(coord2D - vertices_2D[1]) < edgeLimit) &&
               (edgeNormals_2D[2].dot(coord2D - vertices_2D[2]) < edgeLimit);
    }
}

inline Eigen::Vector3d FanTriangle::getHitPoint(const Eigen::Vector2d& coord2D)
{
    return Eigen::Vector3d(coord2D.x(), coord2D.y(),
        vertices_3D[0].z() +
                               (coord2D.x() - vertices_3D[0].x()) * planeEq_X +
                               (coord2D.y() - vertices_3D[0].y()) * planeEq_Y);
}


#endif // FANTRIANGLE_H
