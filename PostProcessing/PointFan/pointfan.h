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
        // Due to coordinate system differences (left- vs right-handed when using xyz-coordinates)
        // It's impossible to say if they are clock- or counterclockwise. Therefore using these terms...
        WO_FORWARD = 0,
        WO_REVERSE,
    };

    enum CoordinateSpace
    {
        CS_NED = 0,
        CS_XYZ,
    };

    PointFan();

    WindingOrder windingOrder = WO_FORWARD;
    CoordinateSpace coordinateSpace = CS_XYZ;
    QVector<Eigen::Vector3d> vertexCoords;
    double pointSpacing = 0.1;

    unsigned int addVertices(const QVector<Eigen::Vector3d>& newVertices); // Returns the number of points added (only adds unique points)
    bool addVertex(const Eigen::Vector3d& newVertex); // Returns true if point was added (only adds unique points)
    void clearVertices(void);
    bool isVertexValid(const Eigen::Vector3d& newPoint);
    unsigned int getNumOfVertices(void);
    void exportFanToFile(const QString& filename, const bool exportCorners, const bool exportFaces, const bool invertedFaces, const int countSanityLimit);
    bool isFanValid(void);
    QVector<Eigen::Vector3d> getVertices(void);

    QVector<Eigen::Vector3d> vertices;

};



#endif // POINTFAN_H
