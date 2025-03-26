/*
    pointfan.cpp (part of GNSS-Stylus)
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

#include "pointfan.h"

PointFan::PointFan()
{

}

unsigned int PointFan::addPoints(const QVector<Eigen::Vector3d>& newPoints)
{
    unsigned int count = 0;

    for (int i = 0; i < newPoints.size(); i++)
    {
        if (!isPointUnique(newPoints[i]))
        {
            // Only add unique points
            continue;
        }
        else
        {
            points.push_back(newPoints[i]);
            count++;
        }
    }

    return count;
}

bool PointFan::addPoint(const Eigen::Vector3d& newPoint)
{
    if (!isPointUnique(newPoint))
    {
        // Only add unique points
        return false;
    }

    points.push_back(newPoint);
    return true;
}

void PointFan::clearPoints(void)
{
    points.clear();
}

bool PointFan::isPointUnique(const Eigen::Vector3d& newPoint)
{
    return points.isEmpty() || (points.constLast() != newPoint);
//    return (!points.contains(newPoint));
}

unsigned int PointFan::getNumOfUniquePoints(void)
{
    return points.size();
}

bool PointFan::isFanvalid(void)
{
    // Could add more checks here, but too lazy...

    return (getNumOfUniquePoints() >= 3);
}

QVector<Eigen::Vector3d> PointFan::getPoints(void)
{
    return points;
}

bool PointFan::exportFanToFile(const QString& filename, const double pointSpacing, const bool exportCorners, const bool exportFaces, const int countSanityLimit)
{
    if (!isFanvalid())
    {
        return false;
    }

    Eigen::Vector3d origin = points[0];
    Eigen::Vector3d xAxis = (points[1] - origin).normalized();
    Eigen::Vector3d zAxis = ((points[1] - origin).cross(points[2] - origin)).normalized();
    Eigen::Vector3d yAxis = xAxis.cross(zAxis).normalized();

    // Using coordinate space here, where axes are orientated according to the first triangle, where:
    // x-axis points from the origin to the first defined point
    // y-axis is perpendicular to x- and z-axes (z = normal)
    // z-axis points to the normal direction of the first triangle

    Eigen::Transform<double, 3, Eigen::Isometry> transform;
    transform(0, 0) = xAxis.x();
    transform(0, 1) = xAxis.y();
    transform(0, 2) = xAxis.z();
    transform(1, 0) = zAxis.x();
    transform(1, 1) = zAxis.y();
    transform(1, 2) = zAxis.z();
    transform(2, 0) = yAxis.x();
    transform(2, 1) = yAxis.y();
    transform(2, 2) = yAxis.z();
    transform.translation() = origin;

    // Doing all operations here in a space that has origin at the first point defined
    // and axes are defined as described above.
    // Therefore we need to convert all points back and forth using transform/inverse transform.

    Eigen::Transform<double, 3, Eigen::Isometry> inverseTransform = transform.inverse();

    QVector<Eigen::Vector3d> localPoints;

    localPoints.reserve(points.size());

    for (Eigen::Vector3d& point:points)
    {
        localPoints.push_back(inverseTransform * point);
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::min();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::min();

    for (Eigen::Vector3d& point:localPoints)
    {
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
        minZ = std::min(minZ, point.z());
        maxZ = std::max(maxZ, point.z());
    }



    return true;
}
