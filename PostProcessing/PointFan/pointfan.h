/*
    pointfan.h (part of GNSS-Stylus)
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

#ifndef POINTFAN_H
#define POINTFAN_H

#include <QVector>
#include <Eigen/Geometry>

class PointFan
{
public:

    enum WindingOrder
    {
        WO_CLOCKWISE = 0,
        WO_COUNTERCLOCKWISE,
    };

    enum CoordinateSpace
    {
        CS_XYZ = 0,
        CS_NED
    };

    PointFan();

    WindingOrder windingOrder = WO_CLOCKWISE;
    CoordinateSpace coordinateSpace = CS_XYZ;
    QVector<Eigen::Vector3d> pointCoords;

    unsigned int addPoints(const QVector<Eigen::Vector3d>& newPoints); // Returns the number of points added (only adds unique points)
    bool addPoint(const Eigen::Vector3d& newPoint); // Returns true if point was added (only adds unique points)
    void clearPoints(void);
    bool isPointUnique(const Eigen::Vector3d& newPoint);
    unsigned int getNumOfUniquePoints(void);
    bool exportFanToFile(const QString& filename, const double pointSpacing, const bool exportCorners, const bool exportFaces, const int countSanityLimit);
    bool isFanvalid(void);
    QVector<Eigen::Vector3d> getPoints(void);

    QVector<Eigen::Vector3d> points;

};



#endif // POINTFAN_H
