/*
    tst_convexhullgenerator.cpp (part of GNSS-Stylus)
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

#include "tst_convexhullgenerator.h"
#include "../PostProcessing/Lidar/PointFilter/ConvexHull/convexhullgenerator.h"
#include "PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h"

TestConvexHullGenerator::TestConvexHullGenerator()
{
}

TestConvexHullGenerator::~TestConvexHullGenerator()
{
}

void TestConvexHullGenerator::initTestCase()
{
}

void TestConvexHullGenerator::cleanupTestCase()
{
}

void TestConvexHullGenerator::valid_Input()
{
    QString plainText = QString::fromUtf8(
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {cos(0)} {0} {0} }\n"
        "	{ {cos(0)} {0} {cos(0)} }\n"
        "	{ {cos(0)} {1} {0} }\n"
        "	{ {cos(0)} {1} {cos(0)} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
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
        QMap<QString, ConvexHull> hullMap = ConvexHullGenerator::generateMap(plainText);

        QCOMPARE(hullMap.count(), 3);

        ConvexHull::Filter hullFilter;

        auto hull = hullMap.begin();

        // As there's currently no way to extract points from hulls,
        // test them by throwing some points through filters.
        QCOMPARE(hull.key(), "firsthull");
        QVERIFY(hull.value().getFilter(hullFilter));
        QVERIFY(hullFilter.isInside(Eigen::Vector3d(0.5, 0.5, 0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, 0.5, -0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, -0.5, 0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(-0.5, 0.5, 0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, 0.5, 1.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, 1.5, 0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(1.5, 0.5, 0.5)));

        hull++;
        QCOMPARE(hull.key(), "secondhull");
        QVERIFY(hull.value().getFilter(hullFilter));
        QVERIFY(hullFilter.isInside(Eigen::Vector3d(-0.5, -0.5, -0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, 0.5, -0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(0.5, -0.5, 0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(-0.5, -0.5, -1.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(-0.5, -0.5, -1.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(-0.5, -1.5, -0.5)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(-1.5, -0.5, -0.5)));

        hull++;
        QCOMPARE(hull.key(), "third");
        QVERIFY(hull.value().getFilter(hullFilter));
        QVERIFY(hullFilter.isInside(Eigen::Vector3d(150, 150, 150)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(150, 150, 99)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(150, 99, 150)));
        QVERIFY(!hullFilter.isInside(Eigen::Vector3d(99, 150, 150)));

    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QFAIL("Should not throw this expection.");
    }

    plainText = "/* Just comment */";

    try
    {
        QMap<QString, ConvexHull> hullMap = ConvexHullGenerator::generateMap(plainText);

        QCOMPARE(hullMap.count(), 0);
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QFAIL("Should not throw this expection.");
    }
}

void TestConvexHullGenerator::error_DuplicateHulls()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "FirstHull\n"   // <- duplicate
        "/* This is Duplicate hull */"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Duplicate hull: \"FirstHull\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("FirstHull"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* This is Duplicate hull */"));
    }
}

void TestConvexHullGenerator::error_InvalidName()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "FirstHöll\n"   // <- Invalid name
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only alphanumeric \"ASCII\" (a...z, 0...9) allowed in convex hull name.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("ö"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("ö") + 1);
    }

    plainText = QString::fromUtf8(
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only alphanumeric \"ASCII\" (a...z, 0...9) allowed in convex hull name.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf(QString::fromUtf8("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd")));
        QCOMPARE(issue.endChar, plainText.lastIndexOf(QString::fromUtf8("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd")) + 1);
    }

}

void TestConvexHullGenerator::error_PointDefinitionsMissing()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "PointlessHull\n"
        "/* hull without point block */";

    try
    {
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Point definitions missing for hull \"PointlessHull\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("PointlessHull"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* hull without point block */"));
    }
}

void TestConvexHullGenerator::error_CharsAtWrongPlaces()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed between hull name and opening curly brace for coordinate definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("Text in wrong place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("Text in wrong place") + 1);
    }

    plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Opening curly brace after point definitions (only 3 spatial dimensions in use in the known universe).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{42}"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("{42}") + 1);
    }

    plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed in point definition block after the coordinate blocks.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at worng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at worng place") + 1);
    }

    plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed in point definition block outside the coordinate blocks.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at wrrng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at wrrng place") + 1);
    }

    plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only comments and whitespaces allowed before first coordinate definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("chars at wrng place"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("chars at wrng place") + 1);
    }

}

void TestConvexHullGenerator::error_NotEnoughPoints()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "TooShortHull/*comment*/\n"
        "/* hull with only 3 points */"
        "{\n"
        "{ {0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"
        "}//EOL comment\n";

    try
    {
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "At least 4 unique points needed to define a convex hull.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{\n{ {0} {0} {-1} }\n"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("}") + 1);
    }
}

void TestConvexHullGenerator::error_TextBlockParser()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "TooShortHull/*comment*/\n"
        "/* hull with only 3 points */"
        "{// Hull start\n"
        "{ {0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n"
        "	{ {0} {-1} {-1} }\n"; // <- Unterminated block
//        "}//EOL comment\n";

    try
    {
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Unterminated block (matching \"}\"-character missing).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{// Hull start"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("{// Hull start") + 1);
    }
}

void TestConvexHullGenerator::error_DuplicatePoints()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
        "{\n"
        "	{/*Comment before coordinates*/ {0} {0} {0} }\n"
        "	{ // EOL-style comment before coordinates\n"
        "{0} {0} {-1} }\n"
        "	{ {0} {-1} {0} }\n" // <- Duplicate
        "	{ {0} {-1} {-1} }\n"
        "	{ {-1} {0} {0} }\n"
        "	{ {-1} {0} {-1} }\n"
        "	{ {-1} {-1} {0} }\n"
        "	{ {0} {-1} {0} }\n" // <- Duplicate
        "/* last point was duplicate */"
        "	{ {-1} {-1} {-1} }\n"
        "/* Comment after points */"
        "} //EOL-style comment before next hull title\n"
        "ThirdHull\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Duplicate point. Only unique points allowed when defining a convex hull.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("{ {0} {-1} {0} }\n"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n/* last point was duplicate */"));
    }
}

void TestConvexHullGenerator::error_notEnoughDimensions()
{
    QString plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "ThirdHull\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Not enough coordinate blocks (3 for xyz) in point definition block.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf(" {200} {200} }"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("}// 2 dimensions"));
    }
}

void TestConvexHullGenerator::error_UnicodeInExpressions()
{
    QString plainText = QString::fromUtf8(
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0 /* Unicode in comment (chinese hello world): \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd */} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "ThirdHull\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }
}

#if 0
void TestConvexHullGenerator::error_InvalidExpression()
{
    // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.
    // Therefore disabled for now.

    QString plainText = QString::fromUtf8(
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0 /* Unicode in comment (chinese hello world): \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd */} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "ThirdHull\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        // TinyExpr++ doesn't always return any error messages (like invalid identifier(?) in this case)
        // (Things like division by zero seem to return a string)
        // Also it returns only one error position (no range).

        QCOMPARE(issue.text, "Error evaluating expression. TinyExpr error: (empty).");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("inv") + 2);
        QCOMPARE(issue.endChar, plainText.lastIndexOf("inv") + 2);
    }

    plainText =
        "firsthull\n"
        "{\n"
        "// Comment for the first hull\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {0} {1} } /* No end of line here */"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "}"
        "secondHull/*comment*/\n"
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
        "} //EOL-style comment before next hull title\n"
        "ThirdHull\n"
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
        ConvexHullGenerator::generateMap(plainText);

        QFAIL("Should throw an exception.");
    }
    catch (ConvexHullGenerator::Issue& issue)
    {
        QCOMPARE(issue.text, "Error evaluating expression. TinyExpr error: Division by zero.");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("203 / 0"));
        QCOMPARE(issue.endChar, issue.beginChar);
    }
}
#endif
