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

class TestPointFanGenerator : public QObject
{
    Q_OBJECT

public:
    TestPointFanGenerator();
    ~TestPointFanGenerator();

private slots:
    void initTestCase();
    void cleanupTestCase();

    void valid_Input();
    void error_DuplicateFans();
    void error_InvalidName();
    void error_PointDefinitionsMissing();
    void error_CharsAtWrongPlaces();
    void error_NotEnoughPoints();
    void error_TextBlockParser();
    void error_DuplicatePoints();
    void error_notEnoughDimensions();
    void error_UnicodeInExpressions();

    void exportFanToFile();

//    void error_InvalidExpression(); // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.

};

#endif // TST_CONVEXHULLGENERATOR_H
