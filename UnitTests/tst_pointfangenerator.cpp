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
    randomGenerator.seed(42);
}

void TestPointFanGenerator::cleanupTestCase()
{
}

Eigen::Vector3d TestPointFanGenerator::getRandomVec(double lowLimit, double highLimit)
{
    return Eigen::Vector3d(randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit
                           );
}

Eigen::Transform<double, 3, Eigen::Affine> TestPointFanGenerator::getRandomTransform(double translateLowLimit, double translateHighLimit)
{
    // Doesn't return very evenly distributed transforms, but should suffice in this context.

    Eigen::AngleAxisd orientation(randomGenerator.generateDouble() * (2 * M_PI), getRandomVec(-1.0, 1.0).normalized());
    Eigen::Transform<double, 3, Eigen::Affine> ret;
    //    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, getRandomVec(-10.0, 10.0));
    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, Eigen::Vector3d(1,1,1));

    return ret;
}

void TestPointFanGenerator::valid_Input()
{
    QString plainText = QString::fromUtf8(
        "firstfan ned forward 0.5\n"
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
        "secondFan  xyz \t reverse 1.05/*comment*/\n"
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
        "THIRD ned reverse 0.05\n"
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
        QVERIFY(fan.value().isFanValid());
        QCOMPARE(fan.value().getNumOfVertices(), (unsigned int)8);
        auto vertices = fan.value().getVertices();
        QCOMPARE(vertices.size(), 8);
        QCOMPARE(vertices[0], Eigen::Vector3d(100, 100, 100));
        QCOMPARE(vertices[1], Eigen::Vector3d(100, 100, 200));
        QCOMPARE(vertices[2], Eigen::Vector3d(100, 200, 100));
        QCOMPARE(vertices[3], Eigen::Vector3d(100, 200, 200));
        QCOMPARE(vertices[4], Eigen::Vector3d(200, 100, 100));
        QCOMPARE(vertices[5], Eigen::Vector3d(200, 100, 200));
        QCOMPARE(vertices[6], Eigen::Vector3d(200, 200, 100));
        QCOMPARE(vertices[7], Eigen::Vector3d(200, 200, 200));

        fan++;
        QCOMPARE(fan.key(), "firstfan");
        QVERIFY(fan.value().isFanValid());
        QCOMPARE(fan.value().getNumOfVertices(), (unsigned int)8);
        vertices = fan.value().getVertices();
        QCOMPARE(vertices.size(), 8);
        QCOMPARE(vertices[0], Eigen::Vector3d(0, 0, 0));
        QCOMPARE(vertices[1], Eigen::Vector3d(0, 0, 1));
        QCOMPARE(vertices[2], Eigen::Vector3d(0, 1, 0));
        QCOMPARE(vertices[3], Eigen::Vector3d(0, 1, 1));
        QCOMPARE(vertices[4], Eigen::Vector3d(cos(0), 0, 0));
        QCOMPARE(vertices[5], Eigen::Vector3d(cos(0), 0, cos(0)));
        QCOMPARE(vertices[6], Eigen::Vector3d(cos(0), 1, 0));
        QCOMPARE(vertices[7], Eigen::Vector3d(cos(0), 1, cos(0)));

        fan++;
        QCOMPARE(fan.key(), "secondFan");
        QVERIFY(fan.value().isFanValid());
        QCOMPARE(fan.value().getNumOfVertices(), (unsigned int)8);
        vertices = fan.value().getVertices();
        QCOMPARE(vertices.size(), 8);
        QCOMPARE(vertices[0], Eigen::Vector3d(0, 0, 0));
        QCOMPARE(vertices[1], Eigen::Vector3d(0, 0, -1));
        QCOMPARE(vertices[2], Eigen::Vector3d(0, -1, 0));
        QCOMPARE(vertices[3], Eigen::Vector3d(0, -1, -1));
        QCOMPARE(vertices[4], Eigen::Vector3d(-1, 0, 0));
        QCOMPARE(vertices[5], Eigen::Vector3d(-1, 0, -1));
        QCOMPARE(vertices[6], Eigen::Vector3d(-1, -1, 0));
        QCOMPARE(vertices[7], Eigen::Vector3d(-1, -1, -1));
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned reverse 0.5/*comment*/\n"
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.endChar, plainText.lastIndexOf("\n ned reverse 0.05/* This is Duplicate fan */"));
    }
}

void TestPointFanGenerator::error_InvalidName()
{
    QString plainText =
        "firstfan\n ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "FirstFän\n"   // <- Invalid name (no need for params as the fan name is validated first)
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
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("ä"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("ä") + 1);
    }

    plainText = QString::fromUtf8(
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd ned forward 0.05\n"   // <- Invalid name ("hello world" in chinese)
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

void TestPointFanGenerator::error_ParamsMissing()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan/*params missing*/\n"    // <- Parameters missing
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Parameters (coordinate system, winding order, point spacing) missing for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("secondFan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("/*params missing*/\n"));
    }

    plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan ned/*params missing*/\n"    // <- Parameters missing
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Some parameters (winding order, point spacing) missing for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("secondFan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf(" ned/*params missing*/\n"));
    }

    plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward/*params missing*/\n"    // <- Parameters missing
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Parameter for point spacing missing for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("secondFan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf(" ned forward/*params missing*/\n"));
    }
}

void TestPointFanGenerator::error_InvalidCoordinateSpace()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan worngspace/*worng(!) coordspace*/\n"    // <- Invalid coordinate space
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Invalid coordinate system definition (\"worngspace\") for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("worngspace"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("/*worng(!) coordspace*/"));
    }
}

void TestPointFanGenerator::error_InvalidWindingOrder()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan NED vastapaivaan/*invalid winding order*/\n"    // <- Invalid winding order
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Invalid winding order definition (\"vastapaivaan\") for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("vastapaivaan"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("/*invalid winding order*/"));
    }
}

void TestPointFanGenerator::error_InvalidPointSpacing()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan NED forward 0.000/*invalid spacing*/\n"    // <- Invalid spacing (out of range)
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Point spacing must be > 0, fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("0.000"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("/*invalid spacing*/"));
    }

    plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan NED forward spacingvalue/*invalid spacing*/\n"    // <- Invalid spacing (can not convert)
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
        "FirstFan\n ned reverse 0.05"   // <- duplicate
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
        QCOMPARE(issue.text, "Can not convert parameter for point spacing (\"spacingvalue\") to float for fan \"secondFan\".");
        QCOMPARE(issue.beginChar, plainText.lastIndexOf("spacingvalue"));
        QCOMPARE(issue.endChar, plainText.lastIndexOf("/*invalid spacing*/"));
    }
}

void TestPointFanGenerator::error_PointDefinitionsMissing()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "PointlessFan ned forward 0.05\n"
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
        QCOMPARE(issue.endChar, plainText.lastIndexOf(" ned forward 0.05\n/* fan without point block */"));
    }
}

void TestPointFanGenerator::error_CharsAtWrongPlaces()
{
    QString plainText =
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "TooShortFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "TooShortFan ned forward 0.05/*comment*/\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "ThirdFan ned forward 0.05\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "ThirdFan ned forward 0.05\n"
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
        "firstfan ned forward 0.05\n"
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
        "secondFan ned forward 0.05/*comment*/\n"
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
        "ThirdFan ned forward 0.05\n"
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

#if 0
// This isn't a "traditional" unit test, but generates some files instead whose validity can be inspected.
// They are also generated into ram disk by default, so you may want to change the path.
void TestPointFanGenerator::exportFanToFile()
{
    // Top, bottom, north, south, east, west here refer to NED
    // (north = x, east = y, down = z) - coordinates

    QString plainText = QString::fromUtf8(
        "CubeFace_Top ned forward 0.05\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {0} {1} {0} }\n"
        "}"
        "CubeFace_Top_Inverted ned reverse 0.05\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {0} {1} {0} }\n"
        "}"
        "CubeFace_Bottom ned reverse 0.05\n"
        "{\n"
        "	{ {0} {0} {1} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {1} {1} {1} }\n"
        "	{ {0} {1} {1} }\n"
        "}"
        "CubeFace_South ned forward 0.05\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {0} {1} {0} }\n"
        "	{ {0} {1} {1} }\n"
        "	{ {0} {0} {1} }\n"
        "}"
        "CubeFace_North ned reverse 0.05\n"
        "{\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "	{ {1} {0} {1} }\n"
        "}"
        "CubeFace_East ned forward 0.05\n"
        "{\n"
        "	{ {0} {1} {0} }\n"
        "	{ {1} {1} {0} }\n"
        "	{ {1} {1} {1} }\n"
        "	{ {0} {1} {1} }\n"
        "}"
        "CubeFace_West ned reverse 0.05\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {1} {0} {0} }\n"
        "	{ {1} {0} {1} }\n"
        "	{ {0} {0} {1} }\n"
        "}"

        "CubeFace_Rotated_Top ned forward 0.03\n"
        "{\n"
        "	{ {sin(2)}          {-cos(2)}           {-sin(pi/4)} }\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {-sin(pi/4)} }\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {-sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {-sin(pi/4)} }\n"
        "}"

        "CubeFace_Rotated_Bottom ned reverse 0.03\n"
        "{\n"
        "	{ {sin(2)}          {-cos(2)}           {sin(pi/4)} }\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {sin(pi/4)} }\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {sin(pi/4)} }\n"
        "}"

        "CubeFace_Rotated_Side1 ned reverse 0.03\n"
        "{\n"
        "	{ {sin(2)}          {-cos(2)}           {-sin(pi/4)} }\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {-sin(pi/4)} }\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {sin(pi/4)} }\n"
        "	{ {sin(2)}          {-cos(2)}           {sin(pi/4)} }\n"
        "}"

        "CubeFace_Rotated_Side2 ned reverse 0.03\n"
        "{\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {-sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {-sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {sin(pi/4)} }\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {sin(pi/4)} }\n"
        "}"

        "CubeFace_Rotated_Side3 ned reverse 0.03\n"
        "{\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {-sin(pi/4)} }\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {-sin(pi/4)} }\n"
        "	{ {sin(2+2*pi/2)}   {-cos(2+2*pi/2)}    {sin(pi/4)} }\n"
        "	{ {sin(2+pi/2)}     {-cos(2+pi/2)}      {sin(pi/4)} }\n"
        "}"

        "CubeFace_Rotated_Side4 ned forward 0.03\n"
        "{\n"
        "	{ {sin(2)}          {-cos(2)}           {-sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {-sin(pi/4)} }\n"
        "	{ {sin(2+3*pi/2)}   {-cos(2+3*pi/2)}    {sin(pi/4)} }\n"
        "	{ {sin(2)}          {-cos(2)}           {sin(pi/4)} }\n"
        "}"

    );

    QMap<QString, PointFan> fanMap = PointFanGenerator::generateMap(plainText);

//    QCOMPARE(fanMap.count(), 7);

    auto fan = fanMap.begin();

    while (fan != fanMap.end())
    {
        fan.value().exportFanToFile("/tmp/ramdisk/" + fan.key() + ".ply", false, true, false, 10e6, Eigen::Transform<double, 3, Eigen::Affine>::Identity());
        fan++;
    }

    fan = fanMap.begin();

    while (fan != fanMap.end())
    {
        fan.value().exportFanToFile("/tmp/ramdisk/" + fan.key() + "_RandomTransform.ply", false, true, false, 10e6, getRandomTransform());
        fan++;
    }


    plainText = QString::fromUtf8(
        "InsaneAmountOfPoints ned forward 0.01\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {10} {0} {0} }\n"
        "	{ {10} {10} {0} }\n"
        "}"
    );

    fanMap = PointFanGenerator::generateMap(plainText);

    fan = fanMap.begin();

    try
    {
        fan.value().exportFanToFile("/tmp/ramdisk/" + fan.key() + ".ply", false, true, false, 400e3, Eigen::Transform<double, 3, Eigen::Affine>::Identity());
        QFAIL("Should throw an exception");
    }
    catch (QString& errorThrown)
    {
        QCOMPARE(errorThrown, "Approximate point count (500000) exceeds sanity limit of 400000. File not created.");
    }


    plainText = QString::fromUtf8(
        "fan ned forward 0.1\n"
        "{\n"
        "	{ {0} {0} {0} }\n"
        "	{ {10} {0} {0} }\n"
        "	{ {10} {10} {0} }\n"
        "}"
        );

    fanMap = PointFanGenerator::generateMap(plainText);

    fan = fanMap.begin();

    try
    {
        fan.value().exportFanToFile("/tmp/ramdisk/this/directory/does_not_exist/" + fan.key() + ".ply", false, true, false, 400e3, Eigen::Transform<double, 3, Eigen::Affine>::Identity());
        QFAIL("Should throw an exception");
    }
    catch (QString& errorThrown)
    {
        QCOMPARE(errorThrown, "Can't open file \"/tmp/ramdisk/this/directory/does_not_exist/fan.ply\".");
    }

    PointFan emptyFan;

    try
    {
        emptyFan.exportFanToFile("/tmp/ramdisk/emptyfan.ply", false, true, false, 1e6, Eigen::Transform<double, 3, Eigen::Affine>::Identity());
        QFAIL("Should throw an exception");
    }
    catch (QString& errorThrown)
    {
        QCOMPARE(errorThrown, "Fan is not valid.");
    }
}
#endif

#if 0
void TestPointFanGenerator::error_InvalidExpression()
{
    // This passes, but TinyExpr leaks memory on division by zero, according to valgrind memory analyzer.
    // Therefore disabled for now.

    QString plainText = QString::fromUtf8(
        "firstfan ned forward 0.03\n"
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
        "secondFan ned forward 0.03/*comment*/\n"
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
        "ThirdFan ned forward 0.03\n"
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
        "firstfan ned forward 0.03\n"
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
        "secondFan ned forward 0.03/*comment*/\n"
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
        "ThirdFan ned forward 0.03\n"
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
