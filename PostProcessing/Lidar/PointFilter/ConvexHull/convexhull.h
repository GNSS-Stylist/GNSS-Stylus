#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "QVector"
#include "Eigen/Geometry"
#include "3d-quickhull/quickhull.h"

class ConvexHull
{
public:
    class Filter
    {
    private:
        class FaceDef
        {
        public:
            Eigen::Vector3d origin;
            Eigen::Vector3d normal;
        };

        Eigen::AlignedBox3d aabb;

    public:
        bool isInside(const Eigen::Vector3d& pointCoords, const double& margin = 0)
        {
            if (((pointCoords.x()) < (aabb.min().x() - margin)) ||
                ((pointCoords.x()) > (aabb.max().x() + margin)) ||
                ((pointCoords.y()) < (aabb.min().y() - margin)) ||
                ((pointCoords.y()) > (aabb.max().y() + margin)) ||
                ((pointCoords.z()) < (aabb.min().z() - margin)) ||
                ((pointCoords.z()) > (aabb.max().z() + margin)) ||
                (faceDefs.size() < 4))
            {
                return false;
            }

            for (const FaceDef& face : qAsConst(faceDefs))
            {
                if ((pointCoords - face.origin).dot(face.normal) > margin)
                {
                    return false;
                }
            }

            return true;
        }

        bool isValid(void) { return faceDefs.size() >= 4; };

    private:
        QVector<FaceDef> faceDefs;
        void init(void) { faceDefs.clear(); };
        friend class ConvexHull;
    };

public:
    ConvexHull();
    ~ConvexHull();
    unsigned int addPoints(const QVector<Eigen::Vector3d>& newPoints); // Returns the number of points added (only adds unique points)
    bool addPoint(const Eigen::Vector3d& newPoint); // Returns true if point was added (only adds unique points)
    void clearPoints(void);
    bool getFilter(ConvexHull::Filter& filter);
    bool isPointUnique(const Eigen::Vector3d& newPoint);
    unsigned int getNumOfUniquePoints(void);
    void exportHullToObjFile(const QString& filename);

private:
    QVector<Eigen::Vector3d> points;
    qh_mesh mesh;
    bool meshGenerated;
    void freeGeneratedMesh(void);
    bool generateMesh(void);
};

#endif // CONVEXHULL_H
