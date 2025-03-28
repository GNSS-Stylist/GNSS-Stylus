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

    FanTriangle(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3);
	inline bool testHit_2D(const Eigen::Vector2d& point2D);
	inline Eigen::Vector3d getHitPoint(const Eigen::Vector2d& point2D);
	Eigen::Vector3d getNormal(void) { return normal_3D; }
    double getArea_2D(void);

private:
    Eigen::Vector3d points_3D[3];
	Eigen::Vector2d points_2D[3];
	Eigen::Vector2d edgeNormals_2D[3];
	Eigen::Vector3d normal_3D;

    double planeEq_X;  // "Slope" of z-coord along the x-axis
    double planeEq_Y;  // "Slope" of z-coord along the y-axis
};

inline bool FanTriangle::testHit_2D(const Eigen::Vector2d& point2D)
{
    if (normal_3D.z() < 0)
    {
        return (edgeNormals_2D[0].dot(point2D - points_2D[0]) > -edgeLimit) &&
               (edgeNormals_2D[1].dot(point2D - points_2D[1]) > -edgeLimit) &&
               (edgeNormals_2D[2].dot(point2D - points_2D[2]) > -edgeLimit);
    }
    else
    {
        return (edgeNormals_2D[0].dot(point2D - points_2D[0]) < edgeLimit) &&
               (edgeNormals_2D[1].dot(point2D - points_2D[1]) < edgeLimit) &&
               (edgeNormals_2D[2].dot(point2D - points_2D[2]) < edgeLimit);
    }
}

inline Eigen::Vector3d FanTriangle::getHitPoint(const Eigen::Vector2d& point2D)
{
    return Eigen::Vector3d(point2D.x(), point2D.y(),
        points_3D[0].z() +
        (point2D.x() - points_3D[0].x()) * planeEq_X +
        (point2D.y() - points_3D[0].y()) * planeEq_Y);
}


#endif // FANTRIANGLE_H
