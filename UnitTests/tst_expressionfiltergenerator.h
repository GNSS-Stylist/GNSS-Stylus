/*
    tst_expressionfiltergenerator.h (part of GNSS-Stylus)
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

#ifndef TST_EXPRESSIONFILTERGENERATOR_H
#define TST_EXPRESSIONFILTERGENERATOR_H

#include <QtTest>
#include <QCoreApplication>
#include <QRandomGenerator>

class TestExpressionFilterGenerator : public QObject
{
    Q_OBJECT

public:
    TestExpressionFilterGenerator();
    ~TestExpressionFilterGenerator();

private slots:
    void initTestCase();
    void cleanupTestCase();

    void validDefinitions_NoConvexHulls();
    void validDefinitions_ConvexHulls();
    void error_DuplicateDevices();
    void error_UnknownDeviceType();
    void error_InvalidMid360IP();
    void error_ExpressionBlocks();
};

#endif // TST_EXPRESSIONFILTERGENERATOR_H
