/*
    convexhull.cpp (part of GNSS-Stylus)
    Copyright (C) 2024-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "convexhull.h"

#define CONVHULL_3D_ENABLE
#include "convhull_3d/convhull_3d.h"


ConvexHull::ConvexHull()
{
}

ConvexHull::~ConvexHull()
{
}

bool ConvexHull::generateMesh(void)
{
    if (points.size() < 4)
    {
        freeGeneratedMesh();
        return false;
    }

    ch_vertex* vertices = (ch_vertex *) malloc(points.size() * sizeof(ch_vertex));

    for (int i = 0; i < points.size(); i++)
    {
        vertices[i].x = points[i].x();
        vertices[i].y = points[i].y();
        vertices[i].z = points[i].z();
    }

    int* faceIndices = nullptr;
    int nFaces;

    convhull_3d_build(vertices, points.size(), &faceIndices, &nFaces);

    if (!faceIndices)
    {
        // "(*out_faces) is returned as NULL, if triangulation fails "

        free(vertices);
        freeGeneratedMesh();
        return false;
    }

    for (int i = 0; i < points.size(); i++)
    {
        mesh.vertices.push_back(points[i]);
    }

    for (int i = 0; i < nFaces; i++)
    {
        mesh.faceIndices.push_back(faceIndices[i * 3 + 0]);
        mesh.faceIndices.push_back(faceIndices[i * 3 + 1]);
        mesh.faceIndices.push_back(faceIndices[i * 3 + 2]);

        Eigen::Vector3d vecAB = points[faceIndices[i * 3 + 1]] - points[faceIndices[i * 3 + 0]];
        Eigen::Vector3d vecAC = points[faceIndices[i * 3 + 2]] - points[faceIndices[i * 3 + 0]];
        Eigen::Vector3d faceNormal = vecAB.cross(vecAC).normalized();

        mesh.normals.push_back(faceNormal);
    }

    mesh.isValid = true;

    meshGenerated = true;

    free(faceIndices);
    free(vertices);

    return true;
}

void ConvexHull::freeGeneratedMesh(void)
{
    mesh.faceIndices.clear();
    mesh.normals.clear();
    mesh.vertices.clear();
    mesh.isValid = false;
    meshGenerated = false;
}

unsigned int ConvexHull::addPoints(const QVector<Eigen::Vector3d>& newPoints)
{
    if (meshGenerated)
    {
        freeGeneratedMesh();
    }

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

bool ConvexHull::addPoint(const Eigen::Vector3d& newPoint)
{
    if (meshGenerated)
    {
        freeGeneratedMesh();
    }

    if (!isPointUnique(newPoint))
    {
        // Only add unique points
        return false;
    }

    points.push_back(newPoint);
    return true;
}

void ConvexHull::clearPoints(void)
{
    freeGeneratedMesh();
    points.clear();
}

bool ConvexHull::getFilter(ConvexHull::Filter& filter)
{
    filter.init();

    if (!meshGenerated)
    {
        if (!generateMesh())
        {
            return false;
        }
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::min();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::min();

    // find AABB (axis aligned bounding box)

    for (int i = 0; i < mesh.vertices.size(); i++)
    {
        minX = std::min(minX, (double)mesh.vertices[i].x());
        maxX = std::max(maxX, (double)mesh.vertices[i].x());

        minY = std::min(minY, (double)mesh.vertices[i].y());
        maxY = std::max(maxY, (double)mesh.vertices[i].y());

        minZ = std::min(minZ, (double)mesh.vertices[i].z());
        maxZ = std::max(maxZ, (double)mesh.vertices[i].z());
    }

    Eigen::AlignedBox3d newAABB(Eigen::Vector3d(minX, minY, minZ), Eigen::Vector3d(maxX, maxY, maxZ));
    filter.aabb = newAABB;

    for (int i = 0; i < mesh.faceIndices.size() / 3; i++)
    {
        Eigen::Vector3d pointA(mesh.vertices[mesh.faceIndices[i * 3]].x(), mesh.vertices[mesh.faceIndices[i * 3]].y(), mesh.vertices[mesh.faceIndices[i * 3]].z());
        Eigen::Vector3d pointB(mesh.vertices[mesh.faceIndices[i * 3 + 1]].x(), mesh.vertices[mesh.faceIndices[i * 3 + 1]].y(), mesh.vertices[mesh.faceIndices[i * 3 + 1]].z());
        Eigen::Vector3d pointC(mesh.vertices[mesh.faceIndices[i * 3 + 2]].x(), mesh.vertices[mesh.faceIndices[i * 3 + 2]].y(), mesh.vertices[mesh.faceIndices[i * 3 + 2]].z());

        Filter::FaceDef newFace;

        newFace.origin = (1.0 / 3.0) * (pointA + pointB + pointC);
        newFace.normal = mesh.normals[i];

        filter.faceDefs.push_back(newFace);
    }

    return true;
}

bool ConvexHull::isPointUnique(const Eigen::Vector3d& newPoint)
{
/*    for (const Eigen::Vector3d& point : qAsConst(points))
    {
        if ((point - newPoint).norm() < 0.001)
        {
            // Arbitrary minimum distance since to test if issues with 3d-quickhull could be prevented (not)
            // (in GNSS-Stylus-case this is 1 mm, so should be far enough).
            return false;
        }
  }
*/
    return (!points.contains(newPoint));
}

unsigned int ConvexHull::getNumOfUniquePoints(void)
{
    return points.size();
}

void ConvexHull::exportHullToObjFile(const QString& filename)
{
    if (!meshGenerated)
    {
        if (!generateMesh())
        {
            return;
        }
    }

    // This is quite "back and forth"-style handling
    // (first creating QVectors base on "raw" data and here converting them back to "raw").
    // But this is not very often used, so who cares?

    ch_vertex* vertices = (ch_vertex*) malloc(mesh.vertices.size() * sizeof(ch_vertex));
    int* faceIndices = (int*) malloc(mesh.faceIndices.size() * sizeof(int));

    char* filenamePtr = filename.toLocal8Bit().data();

    for (int i = 0; i < mesh.vertices.size(); i++)
    {
        vertices[i].x = mesh.vertices[i].x();
        vertices[i].y = mesh.vertices[i].y();
        vertices[i].z = mesh.vertices[i].z();
    }

    for (int i = 0; i < mesh.faceIndices.size(); i++)
    {
        faceIndices[i] = mesh.faceIndices[i];
    }

    convhull_3d_export_obj(vertices, mesh.vertices.size(), faceIndices, mesh.faceIndices.size() / 3, true, filenamePtr);

    free(vertices);
    free(faceIndices);
}





