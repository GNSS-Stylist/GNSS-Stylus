/*
    tst_expressionfilter.h (part of GNSS-Stylus)
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

#ifndef TST_EXPRESSIONFILTER_H
#define TST_EXPRESSIONFILTER_H

#include <QtTest>
#include <QCoreApplication>
#include <QRandomGenerator>
#include "Eigen/Geometry"
#include "../PostProcessing/Lidar/PointFilter/expressionfilter.h"

class TestExpressionFilter : public QObject
{
    Q_OBJECT

public:
    TestExpressionFilter();
    ~TestExpressionFilter();

private:
    static const unsigned int defaultTestRounds = 1000;
    static const unsigned int filterBufferLength = PointFilter::ExpressionFilter::bufferLength;

    QRandomGenerator randomGenerator;
    Eigen::Vector3d getRandomVec(double lowLimit = -10, double highLimit = 10);
    Eigen::Transform<double, 3, Eigen::Affine> getRandomTransform(double translateLowLimit = -10.0, double translateHighLimit = 10.0);
    bool compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2);

    PointFilter::ExpressionFilter::OutItem getRandomOutItem(void);
    LivoxMid360::PointCloudData::Point getRandomLidarSourcePoint(const quint8 propertyMask = 0x3f, const double pointCoordLowLimit = -40.0, const double pointCoordHighLimit = 40.0);

private slots:
    void initTestCase();
    void cleanupTestCase();
    void expressionValidity_ValidExpressions();
    void expressionValidity_InvalidExpressions();
    void noData();
    void defaultExpressions();
    void pureFunctions();
    void lidarCoords();
    void lidarCoords_Indexed();
    void lidarDistance();
    void lidarDistance_Indexed();
    void lidarProperties();
    void lidarProperties_Indexed();
    void lidarReflectivity();
    void lidarReflectivity_Indexed();

    void rigCoords_DefaultTransform();
    void rigCoords_Indexed_DefaultTransform();
    void nedCoords_DefaultTransform();
    void nedCoords_Indexed_DefaultTransform();
    void rigAndNEDCoords_RandomTransforms();
    void rigAndNEDCoords_Indexed_RandomTransforms();

    void convexHullIndexes();
    void invalidConvexHullIndexes();
    void convexHulls_SingleCubeOnOrigin_DefaultTransforms();
    void convexHulls_OverwriteHulls();
    void convexHulls_TwoStaticCubes_DefaultTransforms();
    void convexHulls_MultipleRandomCubes_RandomTransforms();
    void convexHulls_MultipleRandomCubes_RandomTransforms_Indexed();

    void aabb_MultipleRandomCubes_RandomTransforms();
    void aabb_MultipleRandomCubes_RandomTransforms_Indexed();

};

#endif // TST_EXPRESSIONFILTER_H
