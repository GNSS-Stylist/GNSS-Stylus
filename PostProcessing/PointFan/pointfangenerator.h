/*
    pointfangenerator.h (part of GNSS-Stylus)
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

#ifndef POINTFANGENERATOR_H
#define POINTFANGENERATOR_H

#include <QStringList>
#include <QMap>
#include <Eigen/Geometry>

#include "pointfan.h"

/**
 * This class is based on ConvexHullGenerator.
 * Lots of code here is similar, but I still didn't want to reorganize these classes etc.
 * to get rid of duplicate code.
 */


class PointFanGenerator
{
public:
    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    static QMap<QString, PointFan> generateMap(const QString& plainText);

private:
    static void addFanPointsFromBlock(const QString& blockString, const int blockStartCharIndex, PointFan &fan);
    static Eigen::Vector3d extractCoordinatesFromBlock(const QString& blockString, const int blockStartCharIndex);
    static double evaluateBlockContents(const QString& blockString, const int blockStartCharIndex);
};


#endif // CONVEXHULLGENERATOR_H
