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
#include "fantriangle.h"
#include "pointfanfilewriter.h"

PointFan::PointFan()
{

}

unsigned int PointFan::addVertices(const QVector<Eigen::Vector3d>& newVertices)
{
    unsigned int count = 0;

    for (int i = 0; i < newVertices.size(); i++)
    {
        if (!isVertexValid(newVertices[i]))
        {
            // Only add vertices that are different from the last inserted and the first inserted (fan common point, "origin")
            continue;
        }
        else
        {
            vertices.push_back(newVertices[i]);
            count++;
        }
    }

    return count;
}

bool PointFan::addVertex(const Eigen::Vector3d& newVertex)
{
    if (!isVertexValid(newVertex))
    {
        // Only add vertices that are different from the last inserted and the first inserted (fan common point, "origin")
        return false;
    }

    vertices.push_back(newVertex);
    return true;
}

void PointFan::clearVertices(void)
{
    vertices.clear();
}

bool PointFan::isVertexValid(const Eigen::Vector3d& newVertex)
{
    // TODO: Could maybe add some more checks... Too lazy now...
    return vertices.isEmpty() || ((vertices.constLast() != newVertex) && (vertices[0] != newVertex));
}

unsigned int PointFan::getNumOfVertices(void)
{
    return vertices.size();
}

bool PointFan::isFanValid(void)
{
    // TODO: Could add more checks here (for identical points etc.), but too lazy...

    return (getNumOfVertices() >= 3);
}

QVector<Eigen::Vector3d> PointFan::getVertices(void)
{
    return vertices;
}

void PointFan::exportFanToFile(const QString& filename, const ExportParams& exportParams)
{
    if (!isFanValid())
    {
        throw QString("Fan is not valid.");
    }

    Eigen::Vector3d origin = vertices[0];
    Eigen::Vector3d xAxis = (vertices[1] - origin).normalized();
    Eigen::Vector3d zAxis = ((vertices[1] - origin).cross(vertices[2] - origin)).normalized();
    Eigen::Vector3d yAxis = xAxis.cross(zAxis).normalized();

    // Using coordinate space here, where axes are orientated according to the first triangle, where:
    // Origin = fan common vertex (points[0])
    // x-axis points from the origin to the first defined point
    // y-axis is perpendicular to x- and z-axes (z = normal)
    // z-axis points to the normal direction of the first triangle

    Eigen::Transform<double, 3, Eigen::Isometry> transform = Eigen::Transform<double, 3, Eigen::Isometry>::Identity();

    transform(0, 0) = xAxis.x();
    transform(1, 0) = xAxis.y();
    transform(2, 0) = xAxis.z();
    transform(0, 1) = yAxis.x();
    transform(1, 1) = yAxis.y();
    transform(2, 1) = yAxis.z();
    transform(0, 2) = zAxis.x();
    transform(1, 2) = zAxis.y();
    transform(2, 2) = zAxis.z();

    transform.translation() = origin;

    // Doing all operations here in a space that has origin at the first point defined (fan common vertex)
    // and axes are defined as described above.
    // Therefore we need to convert all points back and forth using transform/inverse transform.

    Eigen::Transform<double, 3, Eigen::Isometry> inverseTransform = transform.inverse();

    QVector<Eigen::Vector3d> localPoints;

    localPoints.reserve(vertices.size());

    for (Eigen::Vector3d& point:vertices)
    {
        localPoints.push_back(inverseTransform * point);
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();

    for (Eigen::Vector3d& localPoint:localPoints)
    {
        minX = std::min(minX, localPoint.x());
        maxX = std::max(maxX, localPoint.x());
        minY = std::min(minY, localPoint.y());
        maxY = std::max(maxY, localPoint.y());
        minZ = std::min(minZ, localPoint.z());
        maxZ = std::max(maxZ, localPoint.z());
    }

    QVector<FanTriangle> triangles;
    triangles.reserve(vertices.size() - 1);

    // Construct triangles

    if (windingOrder == WO_FORWARD)
    {
        // Winding order affects normal direction
        for (int i = 2; i < localPoints.size(); i++)
        {
            triangles.push_back(FanTriangle(Eigen::Vector3d(0, 0, 0), localPoints[i - 1], localPoints[i]));
        }
    }
    else
    {
        for (int i = 2; i < localPoints.size(); i++)
        {
            triangles.push_back(FanTriangle(Eigen::Vector3d(0, 0, 0), localPoints[i], localPoints[i - 1]));
        }
    }

    // Calculate combined 2D-area of all triangles

    double totalTriangle2DArea = 0.0;

    for (FanTriangle& triangle:triangles)
    {
        totalTriangle2DArea += triangle.getArea_2D();
    }

    double pointsPerSquare = pow((1.0 / pointSpacing), 2.0);
    double totalApproximatePointCount = totalTriangle2DArea * pointsPerSquare;

    if (totalApproximatePointCount > exportParams.countSanityLimit)
    {
        QString errorString = "Approximate point count (" + QString::number(totalApproximatePointCount) + ") exceeds sanity limit of " + QString::number(exportParams.countSanityLimit) + ". File not created.";
        throw QString(errorString);
    }

    int minXIndex = std::floor(minX / pointSpacing);
    int minYIndex = std::floor(minY / pointSpacing);
    int maxXIndex = std::ceil(maxX / pointSpacing);
    int maxYIndex = std::ceil(maxY / pointSpacing);

    PointFanFileWriter::Params fileWriterParams;

    fileWriterParams.writeCorners = exportParams.exportCorners;
    fileWriterParams.writeFaces = exportParams.exportFaces;
    fileWriterParams.invertedFaces = exportParams.invertedFaces;
    fileWriterParams.writeQuality = exportParams.writeQuality;
    fileWriterParams.qualityValue = exportParams.qualityValue;

    PointFanFileWriter writer(fileWriterParams);

    if (!writer.openFile(filename))
    {
        throw QString("Can't open file \"" + filename + "\".");
    }

    Eigen::Transform<double, 3, Eigen::Affine> combinedTransform = transform;

    if (this->coordinateSpace == CS_NED)
    {
        combinedTransform = exportParams.transform_NEDToXYZ * combinedTransform;
    }

    auto combinedLinearPart = combinedTransform.linear();

    // First point will be our "origin" and common point for the fan
    writer.writePoint(combinedTransform * Eigen::Vector3d::Zero(), combinedLinearPart * triangles[0].getNormal());

    // TODO: This could be optimized somewhat by reordering , limiting and/or "caching" some things.
    // But as they say, "premature optimization is the root of all evil"...

    for (int yIndex = minYIndex; yIndex <= maxYIndex; yIndex++)
    {
        double yCoord = yIndex * pointSpacing;
        for (int xIndex = minXIndex; xIndex <= maxXIndex; xIndex++)
        {
            if ((yIndex == 0) && (xIndex == 0))
            {
                // "Origin" is already written as the first point
                continue;
            }

            double xCoord = xIndex * pointSpacing;

            for (FanTriangle& triangle:triangles)
            {
                Eigen::Vector2d point2d(xCoord,yCoord);
                if (triangle.testHit_2D(point2d))
                {
                    Eigen::Vector3d hitPoint;
                    hitPoint = triangle.getHitPoint(point2d);
                    writer.writePoint(combinedTransform * hitPoint, combinedLinearPart * triangle.getNormal());
//                    writer.writePoint(combinedTransform * hitPoint, triangle.getNormal());
                    break; // Checking other triangles may lead to duplicate vertices (although the normals are different).
                }
            }
        }
    }

    for (auto& triangle:triangles)
    {
        writer.addTriangle(combinedTransform * triangle);
    }

//    writer.finalizeFile();
}
