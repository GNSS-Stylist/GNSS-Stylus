/*
    tst_fantriangle.h (part of GNSS-Stylus)
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

#ifndef TST_FANTRIANGE_H
#define TST_FANTRIANGE_H

#include <QCoreApplication>
#include <QRandomGenerator>
#include "Eigen/Geometry"

class TestFanTriangle : public QObject
{
    Q_OBJECT

public:
    TestFanTriangle();
    ~TestFanTriangle();

private:
    QRandomGenerator randomGenerator;
    bool compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2);

private slots:
    void initTestCase();
    void cleanupTestCase();

    void simplePlanarTriangle();
    void simplePlanarTriangle_ReverseWindingOrder();
    void randomTriangles();

};

#endif // TST_CONVEXHULLGENERATOR_H
