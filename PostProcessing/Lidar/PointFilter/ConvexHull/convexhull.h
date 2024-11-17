/*
    convexhull.h (part of GNSS-Stylus)
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

#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "QVector"
#include "Eigen/Geometry"
#include "convhull_3d/convhull_3d.h"

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
            double adjustedMargin = std::max(margin, 0.0) * 2;

            if (((pointCoords.x()) < (aabb.min().x() - adjustedMargin)) ||
                ((pointCoords.x()) > (aabb.max().x() + adjustedMargin)) ||
                ((pointCoords.y()) < (aabb.min().y() - adjustedMargin)) ||
                ((pointCoords.y()) > (aabb.max().y() + adjustedMargin)) ||
                ((pointCoords.z()) < (aabb.min().z() - adjustedMargin)) ||
                ((pointCoords.z()) > (aabb.max().z() + adjustedMargin)) ||
                (faceDefs.size() < 4))
            {
                // Positive margin can push the real limits of the hull also further than handled here,
                // especially when using narrow wedges. Let's say that this "bug" is a feature....

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

    class Mesh
    {
    public:
//        Mesh() { isValid = false; };

        bool isValid = false;
        QVector<Eigen::Vector3d> vertices;
        QVector<unsigned int> faceIndices;
        QVector<Eigen::Vector3d> normals;
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
    Mesh mesh;
    bool meshGenerated;
    void freeGeneratedMesh(void);
    bool generateMesh(void);
};

#endif // CONVEXHULL_H
