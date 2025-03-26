/*
    tst_convexhullgenerator.h (part of GNSS-Stylus)
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

#ifndef TST_CONVEXHULLGENERATOR_H
#define TST_CONVEXHULLGENERATOR_H

#include <QtTest>
#include <QCoreApplication>
#include <QRandomGenerator>

class TestConvexHullGenerator : public QObject
{
    Q_OBJECT

public:
    TestConvexHullGenerator();
    ~TestConvexHullGenerator();

private slots:
    void initTestCase();
    void cleanupTestCase();

    void valid_Input();
    void error_DuplicateHulls();
    void error_InvalidName();
    void error_PointDefinitionsMissing();
    void error_CharsAtWrongPlaces();
    void error_NotEnoughPoints();
    void error_TextBlockParser();
    void error_DuplicatePoints();
    void error_notEnoughDimensions();
    void error_UnicodeInExpressions();

    // void error_InvalidExpression(); // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.
    // void error_InvalidHull(); // Can't get convhull_3d to return invalid hull. Colinear or coplanar points didn't do the trick.
    // void error_InfiniteEvaluationResult();// Cant get TinyExpr++ to return infinite values (division by zero returns "normal" error)

};

#endif // TST_CONVEXHULLGENERATOR_H
