#include "convexhull.h"

#define QUICKHULL_IMPLEMENTATION
#include "3d-quickhull/quickhull.h"

ConvexHull::ConvexHull()
{
    meshGenerated = false;
}

ConvexHull::~ConvexHull()
{
    freeGeneratedMesh();
}

void ConvexHull::freeGeneratedMesh(void)
{
    if (meshGenerated)
    {
        qh_free_mesh(mesh);
        meshGenerated = false;
    }
}

bool ConvexHull::generateMesh(void)
{
    if (!meshGenerated)
    {
        if (points.size() < 4)
        {
            return false;
        }
    }

    qh_vertex_t* vertices = new qh_vertex_t[points.size()];

    for (int i = 0; i < points.size(); i++)
    {
        vertices[i].x = points[i].x();
        vertices[i].y = points[i].y();
        vertices[i].z = points[i].z();
    }

    mesh = qh_quickhull3d(vertices, points.size());

    delete[] vertices;

    meshGenerated = true;
    return true;
}

unsigned int ConvexHull::addPoints(const QVector<Eigen::Vector3d>& newPoints)
{
    unsigned int count = 0;

    for (int i = 0; i < newPoints.size(); i++)
    {
        if (isPointUnique(newPoints[i]))
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

    freeGeneratedMesh();
    return count;
}

bool ConvexHull::addPoint(const Eigen::Vector3d& newPoint)
{
    if (isPointUnique(newPoint))
    {
        // Only add unique points
        return false;
    }

    points.push_back(newPoint);
    freeGeneratedMesh();
    return true;
}

void ConvexHull::clearPoints(void)
{
    points.clear();
    freeGeneratedMesh();
}

bool ConvexHull::getFilter(ConvexHull::Filter& filter)
{
    filter.init();

    if (!generateMesh())
    {
        return false;
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::min();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::min();
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::min();

    // find AABB (axis aligned bounding box)

    for (unsigned int i = 0; i < mesh.nvertices; i++)
    {
        minX = std::min(minX, (double)mesh.vertices[i].x);
        maxX = std::max(maxX, (double)mesh.vertices[i].x);

        minY = std::min(minY, (double)mesh.vertices[i].y);
        maxY = std::max(maxY, (double)mesh.vertices[i].y);

        minZ = std::min(minZ, (double)mesh.vertices[i].z);
        maxZ = std::max(maxZ, (double)mesh.vertices[i].z);
    }

    Eigen::AlignedBox3d newAABB(Eigen::Vector3d(minX, minY, minZ), Eigen::Vector3d(maxX, maxY, maxZ));
    filter.aabb = newAABB;

    for (unsigned int i = 0; i < mesh.nvertices / 3; i++)
    {
        Eigen::Vector3d pointA(mesh.vertices[i * 3].x, mesh.vertices[i * 3].y, mesh.vertices[i * 3].z);
        Eigen::Vector3d pointB(mesh.vertices[i * 3 + 1].x, mesh.vertices[i * 3 + 1].y, mesh.vertices[i * 3 + 1].z);
        Eigen::Vector3d pointC(mesh.vertices[i * 3 + 2].x, mesh.vertices[i * 3 + 2].y, mesh.vertices[i * 3 + 2].z);

        Eigen::Vector3d vecAB = pointB - pointA;
        Eigen::Vector3d vecAC = pointC - pointA;
        Eigen::Vector3d faceNormal = vecAB.cross(vecAC).normalized();

        Filter::FaceDef newFace;

        newFace.origin = (1.0 / 3.0) * (pointA + pointB + pointC);
        newFace.normal = faceNormal;

        filter.faceDefs.push_back(newFace);
    }

    return true;
}

bool ConvexHull::isPointUnique(const Eigen::Vector3d& newPoint)
{
    return points.contains(newPoint);
}

unsigned int ConvexHull::getNumOfUniquePoints(void)
{
    return points.size();
}

void ConvexHull::exportHullToObjFile(const QString& filename)
{
    generateMesh();
    qh_mesh_export(&mesh, filename.toLocal8Bit());
}





