/*
    convexhullgenerator.h (part of GNSS-Stylus)
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

#ifndef CONVEXHULLGENERATOR_H
#define CONVEXHULLGENERATOR_H

#include <QStringList>
#include <QMap>
#include <Eigen/Geometry>
#include "convexhull.h"

class ConvexHullGenerator
{
public:
    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    static QMap<QString, ConvexHull> generateMap(const QString& plainText);

private:
    static void addHullPointsFromBlock(const QString& blockString, const int blockStartCharIndex, ConvexHull &hull);
    static Eigen::Vector3d extractCoordinatesFromBlock(const QString& blockString, const int blockStartCharIndex);
    static double evaluateBlockContents(const QString& blockString, const int blockStartCharIndex);
};


#endif // CONVEXHULLGENERATOR_H
