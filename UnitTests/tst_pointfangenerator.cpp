/*
    tst_pointfangenerator.cpp (part of GNSS-Stylus)
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

/**
 * This file is heavily based on tst_convexhullgenerator.cpp
 */

#include <qtestcase.h>

#include "tst_pointfangenerator.h"
#include "../PostProcessing/PointFan/pointfangenerator.h"
//#include "PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h"

TestPointFanGenerator::TestPointFanGenerator()
{
}

TestPointFanGenerator::~TestPointFanGenerator()
{
}

void TestPointFanGenerator::initTestCase()
{
}

void TestPointFanGenerator::cleanupTestCase()
{
}

void TestPointFanGenerator::valid_Input()
{
    QString plainText = QString::fromUtf8(
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {cos(0)} {0} {0} }\n"
        "	{ {cos(0)} {0} {cos(0)} }\n"
        "	{ {cos(0)} {1} {0} }\n"
        "	{ {cos(0)} {1} {cos(0)} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
          "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "THIRD\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {100} {200} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {200} {200} {200} }\n"
        " // Chinese hello world in comment (utf8 here): \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n"
        "}");

    try
    {
        QMap<QString, PointFan> fanMap = PointFanGenerator::generateMap(plainText);

        QCOMPARE(fanMap.count(), 3);

        auto fan = fanMap.begin();

        // These are reordered according to keys
        QCOMPARE(fan.key(), "THIRD");
        QVERIFY(fan.value().isFanvalid());
        QCOMPARE(fan.value().getNumOfUniquePoints(), (unsigned int)8);
        auto points = fan.value().getPoints();
        QCOMPARE(points.size(), 8);
        QCOMPARE(points[0], Eigen::Vector3d(100, 100, 100));
        QCOMPARE(points[1], Eigen::Vector3d(100, 100, 200));
        QCOMPARE(points[2], Eigen::Vector3d(100, 200, 100));
        QCOMPARE(points[3], Eigen::Vector3d(100, 200, 200));
        QCOMPARE(points[4], Eigen::Vector3d(200, 100, 100));
        QCOMPARE(points[5], Eigen::Vector3d(200, 100, 200));
        QCOMPARE(points[6], Eigen::Vector3d(200, 200, 100));
        QCOMPARE(points[7], Eigen::Vector3d(200, 200, 200));

        fan++;
        QCOMPARE(fan.key(), "firstfan");
        QVERIFY(fan.value().isFanvalid());
        QCOMPARE(fan.value().getNumOfUniquePoints(), (unsigned int)8);
        points = fan.value().getPoints();
        QCOMPARE(points.size(), 8);
        QCOMPARE(points[0], Eigen::Vector3d(0, 0, 0));
        QCOMPARE(points[1], Eigen::Vector3d(0, 0, 1));
        QCOMPARE(points[2], Eigen::Vector3d(0, 1, 0));
        QCOMPARE(points[3], Eigen::Vector3d(0, 1, 1));
        QCOMPARE(points[4], Eigen::Vector3d(cos(0), 0, 0));
        QCOMPARE(points[5], Eigen::Vector3d(cos(0), 0, cos(0)));
        QCOMPARE(points[6], Eigen::Vector3d(cos(0), 1, 0));
        QCOMPARE(points[7], Eigen::Vector3d(cos(0), 1, cos(0)));

        fan++;
        QCOMPARE(fan.key(), "secondFan");
        QVERIFY(fan.value().isFanvalid());
        QCOMPARE(fan.value().getNumOfUniquePoints(), (unsigned int)8);
        points = fan.value().getPoints();
        QCOMPARE(points.size(), 8);
        QCOMPARE(points[0], Eigen::Vector3d(0, 0, 0));
        QCOMPARE(points[1], Eigen::Vector3d(0, 0, -1));
        QCOMPARE(points[2], Eigen::Vector3d(0, -1, 0));
        QCOMPARE(points[3], Eigen::Vector3d(0, -1, -1));
        QCOMPARE(points[4], Eigen::Vector3d(-1, 0, 0));
        QCOMPARE(points[5], Eigen::Vector3d(-1, 0, -1));
        QCOMPARE(points[6], Eigen::Vector3d(-1, -1, 0));
        QCOMPARE(points[7], Eigen::Vector3d(-1, -1, -1));
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QFAIL("Should not throw this expection.");
    }

    plainText = "/* Just comment */";

    try
    {
        QMap<QString, PointFan> fanMap = PointFanGenerator::generateMap(plainText);

        QCOMPARE(fanMap.count(), 0);
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QFAIL("Should not throw this expection.");
    }
}

void TestPointFanGenerator::error_DuplicateFans()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "FirstFan\n"   // <- duplicate
        "/* This is Duplicate fan */"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {100} {200} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {200} {200} {200} }\n"
        "}";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Duplicate fan: \"FirstFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("FirstFan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* This is Duplicate fan */"));
    }
}

void TestPointFanGenerator::error_InvalidName()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "FirstFän\n"   // <- Invalid name
        "/* Invalid name */"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {100} {200} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {200} {200} {200} }\n"
        "}";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only alphanumeric \"ASCII\" (a...z, 0...9) or \"_\" allowed in fan name.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("ö"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("ö") + 1);
    }

    plainText = QString::fromUtf8(
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n"   // <- Invalid name ("hello world" in chinese)
        "/* Invalid name */"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {100} {200} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {200} {200} {200} }\n"
        "}");

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only alphanumeric \"ASCII\" (a...z, 0...9) or \"_\" allowed in fan name.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf(QString::fromUtf8("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd")));
        QCOMPARE(issue.endChar, plainText.lastIndexOf(QString::fromUtf8("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd")) + 1);
    }

}

void TestPointFanGenerator::error_PointDefinitionsMissing()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "PointlessFan\n"
        "/* fan without point block */";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Point definitions missing for fan \"PointlessFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("PointlessFan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* fan without point block */"));
    }
}

void TestPointFanGenerator::error_CharsAtWrongPlaces()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "Text in wrong place"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "}\n";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed between fan name and opening curly brace for coordinate definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("Text in wrong place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("Text in wrong place") + 1);
    }

    plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} {42} }\n" // <- extra dimension
        "// Extra dimension \n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "}\n";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Opening curly brace after point definitions (only 3 spatial dimensions in use in the known universe).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{42}"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("{42}") + 1);
    }

    plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} chars at worng place}\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "}\n";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed in point definition block after the coordinate blocks.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at worng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at worng place") + 1);
    }

    plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} chars at wrrng place {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} // EOL comment";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed in point definition block outside the coordinate blocks.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at wrrng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at wrrng place") + 1);
    }

    plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{chars at wrng place\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} // EOL comment";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed before first coordinate definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at wrng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at wrng place") + 1);
    }

}

void TestPointFanGenerator::error_NotEnoughPoints()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "TooShortFan/*comment*/\n"
        "/* fan with only 2 points */"
        "{\n"
        "{ {0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "}//EOL comment\n";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "At least 3 unique points needed to define a point fan.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{\n{ {0} {0} {-1} }\n"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("}") + 1);
    }
}

void TestPointFanGenerator::error_TextBlockParser()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "TooShortFan/*comment*/\n"
        "/* fan with only 3 points */"
        "{// Fan start\n"
        "{ {0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"; // <- Unterminated block
//        "}//EOL comment\n";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Unterminated block (matching \"}\"-character missing).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{// Fan start"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("{// Fan start") + 1);
    }
}

void TestPointFanGenerator::error_DuplicatePoints()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {2} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n" // <- Duplicate
        "	{ {0} {-1} {0} }\n" // <- Duplicate
        "/* last point was duplicate */"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "ThirdFan\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {100} {200} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {200} {200} {200} }\n"
        "}";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Duplicate point. Only unique points allowed when defining a point fan.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{ {0} {-1} {0} }\n"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* last point was duplicate */"));
    }
}

void TestPointFanGenerator::error_notEnoughDimensions()
{
    QString plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "ThirdFan\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {200} }// 2 dimensions \n" // <- only 2 values (dimensions)
        "	{ {205} {240} {100} }\n"
        "	{ {203} {203} {203} }\n"
        "}";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Not enough coordinate blocks (3 for xyz) in point definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf(" {200} {200} }"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("}// 2 dimensions"));
    }
}

void TestPointFanGenerator::error_UnicodeInExpressions()
{
    QString plainText = QString::fromUtf8(
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0 /* Unicode in comment (chinese hello world): \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd */} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "ThirdFan\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {200} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {205} {240} {100} }\n"
        "	{ {203} {203} {/* Unicode in expression: */\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd} }\n"
        "}");

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }
}

void TestPointFanGenerator::exportFanToFile()
{
    QString plainText = QString::fromUtf8(
        "CubeFace_Front\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {1} {0} } /* No end of line here */"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {0} {0} }\n"
        "}"
    );

    QMap<QString, PointFan> fanMap = PointFanGenerator::generateMap(plainText);

    QCOMPARE(fanMap.count(), 1);

    auto fan = fanMap.begin();

    fan.value().exportFanToFile(fan.key(), 0.1, false, false, 10e6);

}

#if 0
void TestPointFanGenerator::error_InvalidExpression()
{
    // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.
    // Therefore disabled for now.

    QString plainText = QString::fromUtf8(
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0 /* Unicode in comment (chinese hello world): \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd */} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "ThirdFan\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {inv} {100 // EOL-style comment inside expression\n} {100} }\n" // <- Invalid expression ("inv")
        "	{ {200} {200} {100} }\n"
        "	{ {205} {240} {100} }\n"
        "	{ {203} {203} {7676} }\n"
        "}");

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        // TinyExpr++ doesn't always return any error messages (like invalid identifier(?) in this case)
        // (Things like division by zero seem to return a string)
        // Also it returns only one error position (no range).

        QCOMPARE(issue.text, "Error evaluating expression. TinyExpr error: (empty).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("inv") + 2);
        QCOMPARE(issue.endChar, plainText.lastIndexOf("inv") + 2);
    }

    plainText =
        "firstfan\n"
        "{\n"
        "// Comment for the first fan\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondFan/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next fan title\n"
        "ThirdFan\n"
        "{\n"
        "	{ {100} {100} {100} }\n"
        "	{ {100}/*comment between coordinates*/ {100} {200} }\n"
        "	{ {100} {200} // EOL-style comment between coordinates\n{100} }\n"
        "	{ {100} {200} {200 /*Comment inside expression*/} }\n"
        "	{ {323} {100 // EOL-style comment inside expression\n} {100} }\n"
        "	{ {200} {200} {100} }\n"
        "	{ {205} {240} {100} }\n"
        "	{ {203} {203 / 0}/*<- Division by zero */ {7676} }\n"
        "}";

    try
    {
        PointFanGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (PointFanGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Error evaluating expression. TinyExpr error: Division by zero.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("203 / 0"));
        QCOMPARE(issue.endChar, issue.beginChar);
    }
}
#endif
