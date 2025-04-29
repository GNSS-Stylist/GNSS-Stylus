/*
    tst_pointfanenerator.h (part of GNSS-Stylus)
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

#ifndef TST_POINTFANGENERATOR_H
#define TST_POINTFANGENERATOR_H

#include <QCoreApplication>
#include <QRandomGenerator>
#include "Eigen/Geometry"

class TestPointFanGenerator : public QObject
{
    Q_OBJECT

public:
    TestPointFanGenerator();
    ~TestPointFanGenerator();

private:
    QRandomGenerator randomGenerator;
    Eigen::Vector3d getRandomVec(double lowLimit = -10, double highLimit = 10);
    Eigen::Transform<double, 3, Eigen::Affine> getRandomTransform(double translateLowLimit = -10.0, double translateHighLimit = 10.0);

private slots:
    void initTestCase();
    void cleanupTestCase();

    void valid_Input();
    void error_DuplicateFans();
    void error_InvalidName();
    void error_ParamsMissing();
    void error_InvalidCoordinateSpace();
    void error_InvalidWindingOrder();
    void error_InvalidPointSpacing();
    void error_PointDefinitionsMissing();
    void error_CharsAtWrongPlaces();
    void error_NotEnoughPoints();
    void error_TextBlockParser();
    void error_DuplicatePoints();
    void error_notEnoughDimensions();
    void error_UnicodeInExpressions();

//    void exportFanToFile(); // Generates files to be inspected so disabled by default
//    void error_InvalidExpression(); // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.

};

#endif // TST_CONVEXHULLGENERATOR_H
