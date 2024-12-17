/*
    tst_expressionfilter.cpp (part of GNSS-Stylus)
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

#include "tst_expressionfilter.h"
#include <iostream>

TestExpressionFilter::TestExpressionFilter()
{

}

TestExpressionFilter::~TestExpressionFilter()
{

}

void TestExpressionFilter::initTestCase()
{
    randomGenerator.seed(42);
}

void TestExpressionFilter::cleanupTestCase()
{

}

Eigen::Vector3d TestExpressionFilter::getRandomVec(double lowLimit, double highLimit)
{
    return Eigen::Vector3d(randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit
                           );
}

PointFilter::ExpressionFilter_Mid360::OutItem TestExpressionFilter::getRandomOutItem(void)
{
    PointFilter::ExpressionFilter_Mid360::OutItem item;
    item.valid = randomGenerator.generate() & 1;
    item.filterResult = randomGenerator.bounded(2e9) - 1e9;
    item.uptime_ms = randomGenerator.generate();
    item.coords = getRandomVec();
    item.quality = randomGenerator.bounded(2e9) - 1e9;

    return item;
}

LivoxMid360::PointCloudData::Point TestExpressionFilter::getRandomLidarSourcePoint(const quint8 propertyMask, const double pointCoordLowLimit, const double pointCoordHighLimit)
{
    LivoxMid360::PointCloudData::Point point;

    point.x = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;
    point.y = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;
    point.z = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;

    point.reflectivity = randomGenerator.generate();    // TODO: Check range!
    point.properties = randomGenerator.generate() & propertyMask;

    return point;
}

RPLidarThread::DistanceItem TestExpressionFilter::getRandomRPLidarDistanceItem(const float minDistance, const float maxDistance, const float minQuality, const float maxQuality)
{
    RPLidarThread::DistanceItem distItem;

    distItem.angle = randomGenerator.generateDouble() * M_PI * 2.0;
    distItem.distance = minDistance + randomGenerator.generateDouble() * (maxDistance - minDistance);
    distItem.quality = minQuality + randomGenerator.generateDouble() * (maxQuality - minQuality);

    return distItem;
}

Eigen::Transform<double, 3, Eigen::Affine> TestExpressionFilter::getRandomTransform(double translateLowLimit, double translateHighLimit)
{
    // Doesn't return very evenly distributed transforms, but should suffice in this context.

    Eigen::AngleAxisd orientation(randomGenerator.generateDouble() * (2 * M_PI), getRandomVec(-1.0, 1.0).normalized());
    Eigen::Transform<double, 3, Eigen::Affine> ret;
//    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, getRandomVec(-10.0, 10.0));
    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, Eigen::Vector3d(1,1,1));

    return ret;
}

bool TestExpressionFilter::compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2)
{
    // Compares if two vectors are close enough (< 1 millionth error)
    // "Expanded" if/else to allow breakpoints

    if ((vec1 - vec2).norm() < std::max(vec1.norm() * 1e-6, 1e-100))    // Very small minimum value to prevent failing with _very_ short vecs
    {
        return true;
    }
    else
    {
        return false;
    }
}

void TestExpressionFilter::expressionValidity_ValidExpressions()
{
    PointFilter::ExpressionFilter_Mid360 filter;

    try
    {
        filter.setExpression_Filter("1");
        filter.setExpression_Quality("1");
    }
    catch (...)
    {
        QFAIL("Should not throw expeption");
    }

    // Test hello world in chinese (世界您好) (<- Does this survive gitHub etc. btw?) is 4 characters long.
    try
    {
        filter.setExpression_Filter(QString::fromUtf8("1//Chinese hello world: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n//Hello again: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        filter.setExpression_Quality(QString::fromUtf8("1//Chinese hello world: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n//Hello again: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (...)
    {
        QFAIL("Should not throw expeption");
    }
}

void TestExpressionFilter::expressionValidity_ValidExpressions_RPLidar()
{
    PointFilter::ExpressionFilter_RPLidar filter;

    try
    {
        filter.setExpression_Filter("1");
        filter.setExpression_Quality("1");
    }
    catch (...)
    {
        QFAIL("Should not throw expeption");
    }

    // Test hello world in chinese (世界您好) (<- Does this survive gitHub etc. btw?) is 4 characters long.
    try
    {
        filter.setExpression_Filter(QString::fromUtf8("1//Chinese hello world: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n//Hello again: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        filter.setExpression_Quality(QString::fromUtf8("1//Chinese hello world: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd\n//Hello again: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (...)
    {
        QFAIL("Should not throw expeption");
    }
}

void TestExpressionFilter::expressionValidity_InvalidExpressions()
{
    PointFilter::ExpressionFilter_Mid360 filter;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    try
    {
        filter.setExpression_Filter("invalid//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 1);
    }

    try
    {
        filter.setExpression_Quality("invalid//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
    }

    try
    {
        filter.setExpression_Filter("sin(3.14//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment"));
    }

    try
    {
        filter.setExpression_Quality("cos(45665))//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf(")//Comment"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf(")//Comment"));
    }

    try
    {
        filter.setExpression_Filter("lidar.rplidar.quality()//Comment\n");    // Should only be available in ExpressionFilter_RPLidar
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 3);
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 3);
    }

    try
    {
        filter.setExpression_Quality("lidar.rplidar.quality()//Comment\n");   // Should only be available in ExpressionFilter_RPLidar
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 3);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 3);
    }

    // Test hello world in chinese (世界您好) (<- Does this survive gitHub etc. btw?) is 4 characters long.
    try
    {
        filter.setExpression_Filter(QString::fromUtf8("1\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }

    try
    {
        filter.setExpression_Quality(QString::fromUtf8("1\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QVERIFY(std::isnan(out.filterResult));
        QCOMPARE(out.quality, 0);   // Although the expression is invalid, quality is set to 0 if filter expression is invalid
    }

    try
    {
        filter.setExpression_Filter("1");
    }
    catch (...)
    {
        QFAIL("Should not throw any exception");
    }

    try
    {
        filter.setExpression_Quality("invalid//Comment");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QVERIFY(std::isnan(out.quality));
    }

    try
    {
        filter.setExpression_Filter("#");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("#"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("#"));
    }

    try
    {
        filter.setExpression_Quality("#");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("#"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("#"));
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QVERIFY(std::isnan(out.filterResult));
        QCOMPARE(out.quality, 0);   // Although the expression is invalid, quality is set to 0 if filter expression is invalid
    }
}

void TestExpressionFilter::expressionValidity_InvalidExpressions_RPLidar()
{
    PointFilter::ExpressionFilter_RPLidar filter;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    try
    {
        filter.setExpression_Filter("invalid//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 1);
    }

    try
    {
        filter.setExpression_Quality("invalid//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
    }

    try
    {
        filter.setExpression_Filter("sin(3.14//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment"));
    }

    try
    {
        filter.setExpression_Quality("cos(45665))//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf(")//Comment"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf(")//Comment"));
    }

    try
    {
        filter.setExpression_Filter("lidar.mid360.properties()//Comment\n");    // Should only be available in ExpressionFilter_Mid360
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 3);
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("//Comment") - 3);
    }

    try
    {
        filter.setExpression_Quality("lidar.mid360.reflectivity()//Comment\n");   // Should only be available in ExpressionFilter_Mid360
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 3);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 3);
    }

    // Test hello world in chinese (世界您好) (<- Does this survive gitHub etc. btw?) is 4 characters long.
    try
    {
        filter.setExpression_Filter(QString::fromUtf8("1\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }

    try
    {
        filter.setExpression_Quality(QString::fromUtf8("1\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("\xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd") + 1);
    }

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QVERIFY(std::isnan(out.filterResult));
        QCOMPARE(out.quality, 0);   // Although the expression is invalid, quality is set to 0 if filter expression is invalid
    }

    try
    {
        filter.setExpression_Filter("1");
    }
    catch (...)
    {
        QFAIL("Should not throw any exception");
    }

    try
    {
        filter.setExpression_Quality("invalid//Comment");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("//Comment") - 1);
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QVERIFY(std::isnan(out.quality));
    }

    try
    {
        filter.setExpression_Filter("#");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Filter().lastIndexOf("#"));
        QCOMPARE(issue.endChar, filter.getExpression_Filter().lastIndexOf("#"));
    }

    try
    {
        filter.setExpression_Quality("#");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue& issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter.getExpression_Quality().lastIndexOf("#"));
        QCOMPARE(issue.endChar, filter.getExpression_Quality().lastIndexOf("#"));
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QVERIFY(std::isnan(out.filterResult));
        QCOMPARE(out.quality, 0);   // Although the expression is invalid, quality is set to 0 if filter expression is invalid
    }
}

void TestExpressionFilter::noData()
{
    PointFilter::ExpressionFilter_Mid360 filter;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    for (int i = 0; i < 100; i++)
    {
        out = getRandomOutItem();
        QCOMPARE(filter.getFilteredPoint(out), false);
        QCOMPARE(out.valid, false);
    }
}

void TestExpressionFilter::noData_RPLidar()
{
    PointFilter::ExpressionFilter_RPLidar filter;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    for (int i = 0; i < 100; i++)
    {
        out = getRandomOutItem();
        QCOMPARE(filter.getFilteredPoint(out), false);
        QCOMPARE(out.valid, false);
    }
}

void TestExpressionFilter::defaultExpressions()
{
    PointFilter::ExpressionFilter_Mid360 filter;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    // Prefill buffer
    for (unsigned int i = 0; i < filterBufferLength - 1; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), i);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (unsigned int i = 0; i < filterBufferLength - 1; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), i);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), i + filterBufferLength - 1);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QCOMPARE(out.uptime_ms, i + (filterBufferLength / 2) - 1);
        QCOMPARE(out.quality, 1);
    }
}

void TestExpressionFilter::defaultExpressions_RPLidar()
{
    PointFilter::ExpressionFilter_RPLidar filter;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    // Prefill buffer
    for (unsigned int i = 0; i < filterBufferLength - 1; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), i);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (unsigned int i = 0; i < filterBufferLength - 1; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), i);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), i + filterBufferLength - 1);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QCOMPARE(out.uptime_ms, i + (filterBufferLength / 2) - 1);
        QCOMPARE(out.quality, 1);
    }
}

void TestExpressionFilter::pureFunctions()
{
    PointFilter::ExpressionFilter_Mid360 filter;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    struct
    {
        QString str;
        double expected;
    } expressions[] =
    {
        { "rad_to_deg(3.1415926535897932)", 180 },
        { "rad_to_deg(3.1415926535897932*2)", 180*2 },
        { "rad_to_deg(-3.1415926535897932)", -180 },
        { "rad_to_deg(-3.1415926535897932*2)", -180*2 },

        { "deg_to_rad(180)", M_PI },
        { "deg_to_rad(180 * 2)", 2 * M_PI },
        { "deg_to_rad(-180)", -M_PI },
        { "deg_to_rad(-180 * 2)", -2 * M_PI },
    };

    for (auto expression : expressions)
    {
#if 0
        QString errorMessage;
        int errorlocation;

        if (!filter.setExpression_Filter(expression.str, &errorMessage, &errorlocation))
        {
            // Debug-trap for failing compile

            int foo = 0xabba;
        }
#endif

        filter.setExpression_Filter(expression.str);
        filter.setExpression_Quality("1");

        for (int i = 0; i < 100; i++)
        {
            out = getRandomOutItem();
            filter.addPoint(getRandomLidarSourcePoint(), index++);
            QCOMPARE(filter.getFilteredPoint(out), true);
            QCOMPARE(out.valid, true);
            QCOMPARE(out.filterResult, expression.expected);
            QCOMPARE(out.quality, 0);   // filterResult != 0 -> Quality = 0
        }

        // Test also quality expression

        filter.setExpression_Filter("1"); // Quality will only be calculated if filter returns true (1)
        filter.setExpression_Quality(expression.str);

        for (int i = 0; i < 100; i++)
        {
            out = getRandomOutItem();
            filter.addPoint(getRandomLidarSourcePoint(), index++);
            QCOMPARE(filter.getFilteredPoint(out), true);
            QCOMPARE(out.valid, true);
            QCOMPARE(out.filterResult, 1);
            QCOMPARE(out.quality, expression.expected);
        }
    }
}

void TestExpressionFilter::pureFunctions_RPLidar()
{
    PointFilter::ExpressionFilter_RPLidar filter;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    // Test prefill again after buffer init
    filter.initBuffer();
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomRPLidarDistanceItem(), index);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    struct
    {
        QString str;
        double expected;
    } expressions[] =
        {
         { "rad_to_deg(3.1415926535897932)", 180 },
         { "rad_to_deg(3.1415926535897932*2)", 180*2 },
         { "rad_to_deg(-3.1415926535897932)", -180 },
         { "rad_to_deg(-3.1415926535897932*2)", -180*2 },

         { "deg_to_rad(180)", M_PI },
         { "deg_to_rad(180 * 2)", 2 * M_PI },
         { "deg_to_rad(-180)", -M_PI },
         { "deg_to_rad(-180 * 2)", -2 * M_PI },
         };

    for (auto expression : expressions)
    {
#if 0
        QString errorMessage;
        int errorlocation;

        if (!filter.setExpression_Filter(expression.str, &errorMessage, &errorlocation))
        {
            // Debug-trap for failing compile

            int foo = 0xabba;
        }
#endif

        filter.setExpression_Filter(expression.str);
        filter.setExpression_Quality("1");

        for (int i = 0; i < 100; i++)
        {
            out = getRandomOutItem();
            filter.addPoint(getRandomRPLidarDistanceItem(), index++);
            QCOMPARE(filter.getFilteredPoint(out), true);
            QCOMPARE(out.valid, true);
            QCOMPARE(out.filterResult, expression.expected);
            QCOMPARE(out.quality, 0);   // filterResult != 0 -> Quality = 0
        }

        // Test also quality expression

        filter.setExpression_Filter("1"); // Quality will only be calculated if filter returns true (1)
        filter.setExpression_Quality(expression.str);

        for (int i = 0; i < 100; i++)
        {
            out = getRandomOutItem();
            filter.addPoint(getRandomRPLidarDistanceItem(), index++);
            QCOMPARE(filter.getFilteredPoint(out), true);
            QCOMPARE(out.valid, true);
            QCOMPARE(out.filterResult, 1);
            QCOMPARE(out.quality, expression.expected);
        }
    }
}

void TestExpressionFilter::lidarCoords()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX.setExpression_Filter("lidar.coord.x");
    filter_CoordY.setExpression_Filter("lidar.coord.y");
    filter_CoordZ.setExpression_Filter("lidar.coord.z");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_CoordX.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);

        QCOMPARE(filter_CoordY.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);

        QCOMPARE(filter_CoordZ.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);
    }
}

void TestExpressionFilter::lidarCoords_RPLidar()
{
    RPLidarThread::DistanceItem sourceItems[defaultTestRounds];

    PointFilter::ExpressionFilter_RPLidar filter_CoordX;
    PointFilter::ExpressionFilter_RPLidar filter_CoordY;
    PointFilter::ExpressionFilter_RPLidar filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    filter_CoordX.setExpression_Filter("lidar.coord.x");
    filter_CoordY.setExpression_Filter("lidar.coord.y");
    filter_CoordZ.setExpression_Filter("lidar.coord.z");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourceItems[i] = getRandomRPLidarDistanceItem();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX.addPoint(sourceItems[index], index);
        filter_CoordY.addPoint(sourceItems[index], index);
        filter_CoordZ.addPoint(sourceItems[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX.addPoint(sourceItems[index], index);
        filter_CoordY.addPoint(sourceItems[index], index);
        filter_CoordZ.addPoint(sourceItems[index], index);

        RPLidarThread::DistanceItem source = sourceItems[index - (filterBufferLength / 2)];

        QCOMPARE(filter_CoordX.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sin(source.angle) * source.distance);

        QCOMPARE(filter_CoordY.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, cos(source.angle) * source.distance);

        QCOMPARE(filter_CoordZ.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, 0);
    }
}

void TestExpressionFilter::lidarCoords_Indexed()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX_Index0.setExpression_Filter("lidar.coord_indexed.x(0)");
    filter_CoordY_Index0.setExpression_Filter("lidar.coord_indexed.y(0)");
    filter_CoordZ_Index0.setExpression_Filter("lidar.coord_indexed.z(0)");

    filter_CoordX_IndexMinus1.setExpression_Filter("lidar.coord_indexed.x(-1)");
    filter_CoordX_IndexPlus2.setExpression_Filter("lidar.coord_indexed.x(2)");

    filter_CoordY_IndexMinus2.setExpression_Filter("lidar.coord_indexed.y(-2)");
    filter_CoordY_IndexPlus1.setExpression_Filter("lidar.coord_indexed.y(1)");

    filter_CoordZ_IndexMinus7.setExpression_Filter("lidar.coord_indexed.z(-7)");
    filter_CoordZ_IndexPlus7.setExpression_Filter("lidar.coord_indexed.z(7)");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_CoordX_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);

        QCOMPARE(filter_CoordY_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);

        QCOMPARE(filter_CoordZ_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);

        QCOMPARE(filter_CoordX_IndexMinus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 1].x);

        QCOMPARE(filter_CoordX_IndexPlus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 2].x);

        QCOMPARE(filter_CoordY_IndexMinus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 2].y);

        QCOMPARE(filter_CoordY_IndexPlus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 1].y);

        QCOMPARE(filter_CoordZ_IndexMinus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 7].z);

        QCOMPARE(filter_CoordZ_IndexPlus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 7].z);
    }
}

void TestExpressionFilter::lidarDistance()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Distance;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_Distance.setExpression_Filter("lidar.distance");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Distance.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Distance.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Distance.getFilteredPoint(out), true);
        Eigen::Vector3d sourceVector = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, sourceVector.norm());
    }
}

void TestExpressionFilter::lidarDistance_RPLidar()
{
    RPLidarThread::DistanceItem sourceItems[defaultTestRounds];

    PointFilter::ExpressionFilter_RPLidar filter_Distance;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_RPLidar::OutItem out;

    filter_Distance.setExpression_Filter("lidar.distance");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourceItems[i] = getRandomRPLidarDistanceItem();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Distance.addPoint(sourceItems[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Distance.addPoint(sourceItems[index], index);

        QCOMPARE(filter_Distance.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourceItems[index - (filterBufferLength / 2)].distance);
    }
}


void TestExpressionFilter::lidarDistance_Indexed()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Distance_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_Distance_IndexMinus7;
    PointFilter::ExpressionFilter_Mid360 filter_Distance_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_Distance_Index0.setExpression_Filter("lidar.distance_indexed(0)");
    filter_Distance_IndexMinus7.setExpression_Filter("lidar.distance_indexed(-7)");
    filter_Distance_IndexPlus7.setExpression_Filter("lidar.distance_indexed(7)");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Distance_Index0.addPoint(sourcePoints[index], index);
        filter_Distance_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_Distance_IndexPlus7.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Distance_Index0.addPoint(sourcePoints[index], index);
        filter_Distance_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_Distance_IndexPlus7.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Distance_Index0.getFilteredPoint(out), true);
        int offsettedIndex = index;
        Eigen::Vector3d sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, sourceVector.norm());

        QCOMPARE(filter_Distance_IndexMinus7.getFilteredPoint(out), true);
        offsettedIndex = index - 7;
        sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, sourceVector.norm());

        QCOMPARE(filter_Distance_IndexPlus7.getFilteredPoint(out), true);
        offsettedIndex = index + 7;
        sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, sourceVector.norm());
    }
}

void TestExpressionFilter::lidarAngles()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Angle_Horizontal;
    PointFilter::ExpressionFilter_Mid360 filter_Angle_Vertical;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Horizontal;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Vertical;

    filter_Angle_Horizontal.setExpression_Filter("lidar.angle.horizontal");
    filter_Angle_Vertical.setExpression_Filter("lidar.angle.vertical");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Angle_Horizontal.addPoint(sourcePoints[index], index);
        filter_Angle_Vertical.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Angle_Horizontal.addPoint(sourcePoints[index], index);
        filter_Angle_Vertical.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Angle_Horizontal.getFilteredPoint(out_Horizontal), true);
        QCOMPARE(filter_Angle_Vertical.getFilteredPoint(out_Vertical), true);
        Eigen::Vector3d sourceVector = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);
        QCOMPARE(out_Horizontal.filterResult, atan2(sourceVector.x(), sourceVector.y()));
        QCOMPARE(out_Vertical.filterResult, atan2(sourceVector.z(), sqrt(sourceVector.x() * sourceVector.x() + sourceVector.y() * sourceVector.y())));
    }
}

void TestExpressionFilter::lidarAngles_RPLidar()
{
    RPLidarThread::DistanceItem sourceItems[defaultTestRounds];

    PointFilter::ExpressionFilter_RPLidar filter_Angle_Horizontal;
    PointFilter::ExpressionFilter_RPLidar filter_Angle_Vertical;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_Horizontal;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_Vertical;

    filter_Angle_Horizontal.setExpression_Filter("lidar.angle.horizontal");
    filter_Angle_Vertical.setExpression_Filter("lidar.angle.vertical");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourceItems[i] = getRandomRPLidarDistanceItem();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Angle_Horizontal.addPoint(sourceItems[index], index);
        filter_Angle_Vertical.addPoint(sourceItems[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Angle_Horizontal.addPoint(sourceItems[index], index);
        filter_Angle_Vertical.addPoint(sourceItems[index], index);

        QCOMPARE(filter_Angle_Horizontal.getFilteredPoint(out_Horizontal), true);
        QCOMPARE(filter_Angle_Vertical.getFilteredPoint(out_Vertical), true);
        QCOMPARE(out_Horizontal.filterResult, sourceItems[index - (filterBufferLength / 2)].angle);
        QCOMPARE(out_Vertical.filterResult, 0);
    }
}


void TestExpressionFilter::lidarAngles_Indexed()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Angle_Horizontal_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_Angle_Horizontal_IndexMinus7;
    PointFilter::ExpressionFilter_Mid360 filter_Angle_Vertical_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_Angle_Horizontal_Index0.setExpression_Filter("lidar.angle_indexed.horizontal(0)");
    filter_Angle_Horizontal_IndexMinus7.setExpression_Filter("lidar.angle_indexed.horizonTal(-7)");
    filter_Angle_Vertical_IndexPlus7.setExpression_Filter("lidar.angle_InDexed.vertiCal(7)");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Angle_Horizontal_Index0.addPoint(sourcePoints[index], index);
        filter_Angle_Horizontal_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_Angle_Vertical_IndexPlus7.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Angle_Horizontal_Index0.addPoint(sourcePoints[index], index);
        filter_Angle_Horizontal_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_Angle_Vertical_IndexPlus7.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Angle_Horizontal_Index0.getFilteredPoint(out), true);
        int offsettedIndex = index;
        Eigen::Vector3d sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, atan2(sourceVector.x(), sourceVector.y()));

        QCOMPARE(filter_Angle_Horizontal_IndexMinus7.getFilteredPoint(out), true);
        offsettedIndex = index - 7;
        sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, atan2(sourceVector.x(), sourceVector.y()));

        QCOMPARE(filter_Angle_Vertical_IndexPlus7.getFilteredPoint(out), true);
        offsettedIndex = index + 7;
        sourceVector = Eigen::Vector3d(sourcePoints[offsettedIndex - (filterBufferLength / 2)].x, sourcePoints[offsettedIndex - (filterBufferLength / 2)].y, sourcePoints[offsettedIndex - (filterBufferLength / 2)].z);
        QCOMPARE(out.filterResult, atan2(sourceVector.z(), sqrt(sourceVector.x() * sourceVector.x() + sourceVector.y() * sourceVector.y())));
    }
}


void TestExpressionFilter::lidarProperties()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Properties;
    PointFilter::ExpressionFilter_Mid360 filter_Properties_other;
    PointFilter::ExpressionFilter_Mid360 filter_Properties_dust;
    PointFilter::ExpressionFilter_Mid360 filter_Properties_glue;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Properties;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_other;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_dust;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_glue;

    filter_Properties.setExpression_Filter("lidar.mid360.properties");
    filter_Properties_other.setExpression_Filter("lidar.mid360.properties.other");
    filter_Properties_dust.setExpression_Filter("lidar.mid360.properties.dust");
    filter_Properties_glue.setExpression_Filter("lidar.mid360.properties.glue");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0xFF);
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Properties.addPoint(sourcePoints[index], index);
        filter_Properties_other.addPoint(sourcePoints[index], index);
        filter_Properties_dust.addPoint(sourcePoints[index], index);
        filter_Properties_glue.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Properties.addPoint(sourcePoints[index], index);
        filter_Properties_other.addPoint(sourcePoints[index], index);
        filter_Properties_dust.addPoint(sourcePoints[index], index);
        filter_Properties_glue.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Properties.getFilteredPoint(out_Properties), true);
        QCOMPARE(filter_Properties_other.getFilteredPoint(out_Properties_other), true);
        QCOMPARE(filter_Properties_dust.getFilteredPoint(out_Properties_dust), true);
        QCOMPARE(filter_Properties_glue.getFilteredPoint(out_Properties_glue), true);

        QCOMPARE(out_Properties.filterResult, sourcePoints[index - (filterBufferLength / 2)].properties);
        QCOMPARE(out_Properties_other.filterResult, ((sourcePoints[index - (filterBufferLength / 2)].properties) >> 4) & 0x03);
        QCOMPARE(out_Properties_dust.filterResult, ((sourcePoints[index - (filterBufferLength / 2)].properties) >> 2) & 0x03);
        QCOMPARE(out_Properties_glue.filterResult, ((sourcePoints[index - (filterBufferLength / 2)].properties) >> 0) & 0x03);
    }
}

void TestExpressionFilter::lidarProperties_Indexed()
{
    for (int offset = -7; offset <= 7; offset += 1)
    {
        LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

        PointFilter::ExpressionFilter_Mid360 filter_Properties;
        PointFilter::ExpressionFilter_Mid360 filter_Properties_other;
        PointFilter::ExpressionFilter_Mid360 filter_Properties_dust;
        PointFilter::ExpressionFilter_Mid360 filter_Properties_glue;

        unsigned int index = 0;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Properties;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_other;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_dust;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Properties_glue;

        filter_Properties.setExpression_Filter(QString("lidar.mid360.properties_indexed(") + QString::number(offset) + ")");
        filter_Properties_other.setExpression_Filter(QString("lidar.mid360.properties_indexed.other(") + QString::number(offset) + ")");
        filter_Properties_dust.setExpression_Filter(QString("lidar.mid360.properties_indexed.dust(") + QString::number(offset) + ")");
        filter_Properties_glue.setExpression_Filter(QString("lidar.mid360.properties_indexed.glue(") + QString::number(offset) + ")");

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourcePoints[i] = getRandomLidarSourcePoint(0xFF);
        }

        // Prefill buffer
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Properties.addPoint(sourcePoints[index], index);
            filter_Properties_other.addPoint(sourcePoints[index], index);
            filter_Properties_dust.addPoint(sourcePoints[index], index);
            filter_Properties_glue.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Properties.addPoint(sourcePoints[index], index);
            filter_Properties_other.addPoint(sourcePoints[index], index);
            filter_Properties_dust.addPoint(sourcePoints[index], index);
            filter_Properties_glue.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Properties.getFilteredPoint(out_Properties), true);
            QCOMPARE(filter_Properties_other.getFilteredPoint(out_Properties_other), true);
            QCOMPARE(filter_Properties_dust.getFilteredPoint(out_Properties_dust), true);
            QCOMPARE(filter_Properties_glue.getFilteredPoint(out_Properties_glue), true);

            int offsettedIndex = index - (filterBufferLength / 2) + offset;

            QCOMPARE(out_Properties.filterResult, sourcePoints[offsettedIndex].properties);
            QCOMPARE(out_Properties_other.filterResult, ((sourcePoints[offsettedIndex].properties) >> 4) & 0x03);
            QCOMPARE(out_Properties_dust.filterResult, ((sourcePoints[offsettedIndex].properties) >> 2) & 0x03);
            QCOMPARE(out_Properties_glue.filterResult, ((sourcePoints[offsettedIndex].properties) >> 0) & 0x03);
        }
    }
}

void TestExpressionFilter::lidarReflectivity()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Reflectivity;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Reflectivity;

    filter_Reflectivity.setExpression_Filter("lidar.mid360.reflectivity");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Reflectivity.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Reflectivity.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Reflectivity.getFilteredPoint(out_Reflectivity), true);

        QCOMPARE(out_Reflectivity.filterResult, sourcePoints[index - (filterBufferLength / 2)].reflectivity);
    }
}

void TestExpressionFilter::lidarReflectivity_Indexed()
{
    for (int offset = -7; offset <= 7; offset += 1)
    {
        LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

        PointFilter::ExpressionFilter_Mid360 filter_Reflectivity;

        unsigned int index = 0;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Reflectivity;

        filter_Reflectivity.setExpression_Filter(QString("lidar.mid360.reflectivity_indexed(") + QString::number(offset) + ")");

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourcePoints[i] = getRandomLidarSourcePoint();
        }

        // Prefill buffer
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Reflectivity.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Reflectivity.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Reflectivity.getFilteredPoint(out_Reflectivity), true);

            int offsettedIndex = index - (filterBufferLength / 2) + offset;

            QCOMPARE(out_Reflectivity.filterResult, sourcePoints[offsettedIndex].reflectivity);
        }
    }
}

void TestExpressionFilter::lidarQuality_RPLidar()
{
    RPLidarThread::DistanceItem sourceItems[defaultTestRounds];

    PointFilter::ExpressionFilter_RPLidar filter_Quality;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_Quality;

    filter_Quality.setExpression_Filter("lidar.rplidar.quality");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourceItems[i] = getRandomRPLidarDistanceItem();
    }

    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_Quality.addPoint(sourceItems[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_Quality.addPoint(sourceItems[index], index);

        QCOMPARE(filter_Quality.getFilteredPoint(out_Quality), true);

        QCOMPARE(out_Quality.filterResult, sourceItems[index - (filterBufferLength / 2)].quality);
    }
}

void TestExpressionFilter::lidarQuality_Indexed_RPLidar()
{
    for (int offset = -7; offset <= 7; offset += 1)
    {
        RPLidarThread::DistanceItem sourceItems[defaultTestRounds];

        PointFilter::ExpressionFilter_RPLidar filter_Quality;

        unsigned int index = 0;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_Quality;

        filter_Quality.setExpression_Filter(QString("lidar.rplidar.quality_indexed(") + QString::number(offset) + ")");

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourceItems[i] = getRandomRPLidarDistanceItem();
        }

        // Prefill buffer
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Quality.addPoint(sourceItems[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Quality.addPoint(sourceItems[index], index);

            QCOMPARE(filter_Quality.getFilteredPoint(out_Quality), true);

            int offsettedIndex = index - (filterBufferLength / 2) + offset;

            QCOMPARE(out_Quality.filterResult, sourceItems[offsettedIndex].quality);
        }
    }
}


void TestExpressionFilter::rigCoords_DefaultTransform()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX.setExpression_Filter("rig.coord.x");
    filter_CoordY.setExpression_Filter("rig.coord.y");
    filter_CoordZ.setExpression_Filter("rig.coord.z");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_CoordX.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);

        QCOMPARE(filter_CoordY.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);

        QCOMPARE(filter_CoordZ.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);
    }
}

void TestExpressionFilter::rigCoords_Indexed_DefaultTransform()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX_Index0.setExpression_Filter("rig.coord_indexed.x(0)");
    filter_CoordY_Index0.setExpression_Filter("rig.coord_indexed.y(0)");
    filter_CoordZ_Index0.setExpression_Filter("rig.coord_indexed.z(0)");

    filter_CoordX_IndexMinus1.setExpression_Filter("rig.coord_indexed.x(-1)");
    filter_CoordX_IndexPlus2.setExpression_Filter("rig.coord_indexed.x(2)");

    filter_CoordY_IndexMinus2.setExpression_Filter("rig.coord_indexed.y(-2)");
    filter_CoordY_IndexPlus1.setExpression_Filter("rig.coord_indexed.y(1)");

    filter_CoordZ_IndexMinus7.setExpression_Filter("rig.coord_indexed.z(-7)");
    filter_CoordZ_IndexPlus7.setExpression_Filter("rig.coord_indexed.z(7)");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_CoordX_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);

        QCOMPARE(filter_CoordY_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);

        QCOMPARE(filter_CoordZ_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);

        QCOMPARE(filter_CoordX_IndexMinus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 1].x);

        QCOMPARE(filter_CoordX_IndexPlus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 2].x);

        QCOMPARE(filter_CoordY_IndexMinus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 2].y);

        QCOMPARE(filter_CoordY_IndexPlus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 1].y);

        QCOMPARE(filter_CoordZ_IndexMinus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 7].z);

        QCOMPARE(filter_CoordZ_IndexPlus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 7].z);
    }
}

void TestExpressionFilter::nedCoords_DefaultTransform()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX.setExpression_Filter("ned.coord.x");
    filter_CoordY.setExpression_Filter("ned.coord.y");
    filter_CoordZ.setExpression_Filter("ned.coord.z");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX.addPoint(sourcePoints[index], index);
        filter_CoordY.addPoint(sourcePoints[index], index);
        filter_CoordZ.addPoint(sourcePoints[index], index);

        Eigen::Vector3d expectedNEDOutVector = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);

        QCOMPARE(filter_CoordX.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordY.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordZ.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));
    }
}

void TestExpressionFilter::nedCoords_Indexed_DefaultTransform()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_CoordX_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_Index0;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter_Mid360 filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter_Mid360 filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter_Mid360 filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter_Mid360::OutItem out;

    filter_CoordX_Index0.setExpression_Filter("ned.coord_indexed.x(0)");
    filter_CoordY_Index0.setExpression_Filter("ned.coord_indexed.y(0)");
    filter_CoordZ_Index0.setExpression_Filter("ned.coord_indexed.z(0)");

    filter_CoordX_IndexMinus1.setExpression_Filter("ned.coord_indexed.x(-1)");
    filter_CoordX_IndexPlus2.setExpression_Filter("ned.coord_indexed.x(2)");

    filter_CoordY_IndexMinus2.setExpression_Filter("ned.coord_indexed.y(-2)");
    filter_CoordY_IndexPlus1.setExpression_Filter("ned.coord_indexed.y(1)");

    filter_CoordZ_IndexMinus7.setExpression_Filter("ned.coord_indexed.z(-7)");
    filter_CoordZ_IndexPlus7.setExpression_Filter("ned.coord_indexed.z(7)");

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        filter_CoordX_Index0.addPoint(sourcePoints[index], index);
        filter_CoordY_Index0.addPoint(sourcePoints[index], index);
        filter_CoordZ_Index0.addPoint(sourcePoints[index], index);

        filter_CoordX_IndexMinus1.addPoint(sourcePoints[index], index);
        filter_CoordX_IndexPlus2.addPoint(sourcePoints[index], index);

        filter_CoordY_IndexMinus2.addPoint(sourcePoints[index], index);
        filter_CoordY_IndexPlus1.addPoint(sourcePoints[index], index);

        filter_CoordZ_IndexMinus7.addPoint(sourcePoints[index], index);
        filter_CoordZ_IndexPlus7.addPoint(sourcePoints[index], index);

        Eigen::Vector3d expectedNEDOutVector = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);

        QCOMPARE(filter_CoordX_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].x);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordY_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].y);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordZ_Index0.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2)].z);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordX_IndexMinus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 1].x);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordX_IndexPlus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 2].x);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordY_IndexMinus2.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 2].y);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordY_IndexPlus1.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 1].y);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordZ_IndexMinus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) - 7].z);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));

        QCOMPARE(filter_CoordZ_IndexPlus7.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filterBufferLength / 2) + 7].z);
        QVERIFY(compareVectors(out.coords, expectedNEDOutVector));
    }
}

void TestExpressionFilter::rigAndNEDCoords_RandomTransforms()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];
    Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
    Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordX;
    PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordY;
    PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordZ;

    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_X;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_Y;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_Z;

    PointFilter::ExpressionFilter_Mid360 filter_NED_CoordX;
    PointFilter::ExpressionFilter_Mid360 filter_NED_CoordY;
    PointFilter::ExpressionFilter_Mid360 filter_NED_CoordZ;

    PointFilter::ExpressionFilter_Mid360::OutItem out_NED_X;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED_Y;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED_Z;

    filter_Rig_CoordX.setExpression_Filter("rig.coord.x");
    filter_Rig_CoordY.setExpression_Filter("rig.coord.y");
    filter_Rig_CoordZ.setExpression_Filter("rig.coord.z");

    filter_NED_CoordX.setExpression_Filter("ned.coord.x");
    filter_NED_CoordY.setExpression_Filter("ned.coord.y");
    filter_NED_CoordZ.setExpression_Filter("ned.coord.z");

    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();
//    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
//    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    unsigned int transformChanges_LidarToRig = 0;
    unsigned int transformChanges_RigToNED = 0;

    unsigned int index = 0;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
        transforms_LidarToRig[i] = transform_LidarToRig;
        transforms_RigToNED[i] = transform_RigToNED;

        if ((randomGenerator.generate() % 20) == 0)
        {
            transform_LidarToRig = getRandomTransform();
            transformChanges_LidarToRig++;
        }
        if ((randomGenerator.generate() % 20) == 0)
        {
            transform_RigToNED = getRandomTransform();
            transformChanges_RigToNED++;
        }
    }

    Q_ASSERT(transformChanges_LidarToRig > 3);
    Q_ASSERT(transformChanges_RigToNED > 3);
    Q_ASSERT(transformChanges_LidarToRig < defaultTestRounds - 10);
    Q_ASSERT(transformChanges_RigToNED < defaultTestRounds - 10);

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        if ((index == 0) || (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index]))))
        {
            filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

            filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
        }

        if ((index == 0) || (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index]))))
        {
            filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

            filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
        }

        filter_Rig_CoordX.addPoint(sourcePoints[index], index);
        filter_Rig_CoordY.addPoint(sourcePoints[index], index);
        filter_Rig_CoordZ.addPoint(sourcePoints[index], index);

        filter_NED_CoordX.addPoint(sourcePoints[index], index);
        filter_NED_CoordY.addPoint(sourcePoints[index], index);
        filter_NED_CoordZ.addPoint(sourcePoints[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        if (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index])))
        {
            filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

            filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
        }

        if (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index])))
        {
            filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

            filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
        }

        filter_Rig_CoordX.addPoint(sourcePoints[index], index);
        filter_Rig_CoordY.addPoint(sourcePoints[index], index);
        filter_Rig_CoordZ.addPoint(sourcePoints[index], index);

        filter_NED_CoordX.addPoint(sourcePoints[index], index);
        filter_NED_CoordY.addPoint(sourcePoints[index], index);
        filter_NED_CoordZ.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_Rig_CoordX.getFilteredPoint(out_Rig_X), true);
        QCOMPARE(filter_Rig_CoordY.getFilteredPoint(out_Rig_Y), true);
        QCOMPARE(filter_Rig_CoordZ.getFilteredPoint(out_Rig_Z), true);

        QCOMPARE(filter_NED_CoordX.getFilteredPoint(out_NED_X), true);
        QCOMPARE(filter_NED_CoordY.getFilteredPoint(out_NED_Y), true);
        QCOMPARE(filter_NED_CoordZ.getFilteredPoint(out_NED_Z), true);

        Eigen::Vector3d sourceVector(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);
        Eigen::Vector3d rigVector(out_Rig_X.filterResult, out_Rig_Y.filterResult, out_Rig_Z.filterResult);
        Eigen::Vector3d nedVector(out_NED_X.filterResult, out_NED_Y.filterResult, out_NED_Z.filterResult);

        QVERIFY(compareVectors(transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector, rigVector));
        QVERIFY(compareVectors(transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector), nedVector));

        Eigen::Vector3d expectedNEDOutVector = transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector);

        QVERIFY(compareVectors(out_NED_X.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_NED_Y.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_NED_Z.coords, expectedNEDOutVector));

        QVERIFY(compareVectors(out_Rig_X.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_Rig_Y.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_Rig_Z.coords, expectedNEDOutVector));
    }
}

void TestExpressionFilter::rigAndNEDCoords_RandomTransforms_RPLidar()
{
    RPLidarThread::DistanceItem sourceItems[defaultTestRounds];
    Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
    Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

    PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordX;
    PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordY;
    PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordZ;

    PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_X;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Y;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Z;

    PointFilter::ExpressionFilter_RPLidar filter_NED_CoordX;
    PointFilter::ExpressionFilter_RPLidar filter_NED_CoordY;
    PointFilter::ExpressionFilter_RPLidar filter_NED_CoordZ;

    PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_X;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Y;
    PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Z;

    filter_Rig_CoordX.setExpression_Filter("rig.coord.x");
    filter_Rig_CoordY.setExpression_Filter("rig.coord.y");
    filter_Rig_CoordZ.setExpression_Filter("rig.coord.z");

    filter_NED_CoordX.setExpression_Filter("ned.coord.x");
    filter_NED_CoordY.setExpression_Filter("ned.coord.y");
    filter_NED_CoordZ.setExpression_Filter("ned.coord.z");

    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();
    //    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    //    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    unsigned int transformChanges_LidarToRig = 0;
    unsigned int transformChanges_RigToNED = 0;

    unsigned int index = 0;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourceItems[i] = getRandomRPLidarDistanceItem();
        transforms_LidarToRig[i] = transform_LidarToRig;
        transforms_RigToNED[i] = transform_RigToNED;

        if ((randomGenerator.generate() % 20) == 0)
        {
            transform_LidarToRig = getRandomTransform();
            transformChanges_LidarToRig++;
        }
        if ((randomGenerator.generate() % 20) == 0)
        {
            transform_RigToNED = getRandomTransform();
            transformChanges_RigToNED++;
        }
    }

    Q_ASSERT(transformChanges_LidarToRig > 3);
    Q_ASSERT(transformChanges_RigToNED > 3);
    Q_ASSERT(transformChanges_LidarToRig < defaultTestRounds - 10);
    Q_ASSERT(transformChanges_RigToNED < defaultTestRounds - 10);

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        if ((index == 0) || (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index]))))
        {
            filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

            filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
        }

        if ((index == 0) || (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index]))))
        {
            filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

            filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
        }

        filter_Rig_CoordX.addPoint(sourceItems[index], index);
        filter_Rig_CoordY.addPoint(sourceItems[index], index);
        filter_Rig_CoordZ.addPoint(sourceItems[index], index);

        filter_NED_CoordX.addPoint(sourceItems[index], index);
        filter_NED_CoordY.addPoint(sourceItems[index], index);
        filter_NED_CoordZ.addPoint(sourceItems[index], index);
    }

    for (; index < defaultTestRounds; index++)
    {
        if (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index])))
        {
            filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

            filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
            filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
        }

        if (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index])))
        {
            filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

            filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
            filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
        }

        filter_Rig_CoordX.addPoint(sourceItems[index], index);
        filter_Rig_CoordY.addPoint(sourceItems[index], index);
        filter_Rig_CoordZ.addPoint(sourceItems[index], index);

        filter_NED_CoordX.addPoint(sourceItems[index], index);
        filter_NED_CoordY.addPoint(sourceItems[index], index);
        filter_NED_CoordZ.addPoint(sourceItems[index], index);

        QCOMPARE(filter_Rig_CoordX.getFilteredPoint(out_Rig_X), true);
        QCOMPARE(filter_Rig_CoordY.getFilteredPoint(out_Rig_Y), true);
        QCOMPARE(filter_Rig_CoordZ.getFilteredPoint(out_Rig_Z), true);

        QCOMPARE(filter_NED_CoordX.getFilteredPoint(out_NED_X), true);
        QCOMPARE(filter_NED_CoordY.getFilteredPoint(out_NED_Y), true);
        QCOMPARE(filter_NED_CoordZ.getFilteredPoint(out_NED_Z), true);

//        Eigen::Vector3d sourceVector(sourceItems[index - (filterBufferLength / 2)].x, sourceItems[index - (filterBufferLength / 2)].y, sourceItems[index - (filterBufferLength / 2)].z);
        RPLidarThread::DistanceItem item = sourceItems[index - (filterBufferLength / 2)];
        Eigen::Vector3d sourceVector(sin(item.angle) * item.distance, cos(item.angle) * item.distance, 0);
        Eigen::Vector3d rigVector(out_Rig_X.filterResult, out_Rig_Y.filterResult, out_Rig_Z.filterResult);
        Eigen::Vector3d nedVector(out_NED_X.filterResult, out_NED_Y.filterResult, out_NED_Z.filterResult);

        QVERIFY(compareVectors(transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector, rigVector));
        QVERIFY(compareVectors(transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector), nedVector));

        Eigen::Vector3d expectedNEDOutVector = transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * sourceVector);

        QVERIFY(compareVectors(out_NED_X.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_NED_Y.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_NED_Z.coords, expectedNEDOutVector));

        QVERIFY(compareVectors(out_Rig_X.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_Rig_Y.coords, expectedNEDOutVector));
        QVERIFY(compareVectors(out_Rig_Z.coords, expectedNEDOutVector));
    }
}

void TestExpressionFilter::rigAndNEDCoords_Indexed_RandomTransforms()
{
    for (int rigOffset = -((filterBufferLength / 2) - 1); rigOffset < int(filterBufferLength / 2); rigOffset++)
    {
        int nedOffset = -rigOffset;

        LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

        PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordX;
        PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordY;
        PointFilter::ExpressionFilter_Mid360 filter_Rig_CoordZ;

        PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_X;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_Y;
        PointFilter::ExpressionFilter_Mid360::OutItem out_Rig_Z;

        PointFilter::ExpressionFilter_Mid360 filter_NED_CoordX;
        PointFilter::ExpressionFilter_Mid360 filter_NED_CoordY;
        PointFilter::ExpressionFilter_Mid360 filter_NED_CoordZ;

        PointFilter::ExpressionFilter_Mid360::OutItem out_NED_X;
        PointFilter::ExpressionFilter_Mid360::OutItem out_NED_Y;
        PointFilter::ExpressionFilter_Mid360::OutItem out_NED_Z;

        filter_Rig_CoordX.setExpression_Filter(QString("rig.coord_indexed.x(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordY.setExpression_Filter(QString("rig.coord_indexed.y(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordZ.setExpression_Filter(QString("rig.coord_indexed.z(" + QString::number(rigOffset) + ")"));

        filter_NED_CoordX.setExpression_Filter(QString("ned.coord_indexed.x(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordY.setExpression_Filter(QString("ned.coord_indexed.y(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordZ.setExpression_Filter(QString("ned.coord_indexed.z(" + QString::number(nedOffset) + ")"));

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        unsigned int transformChanges_LidarToRig = 0;
        unsigned int transformChanges_RigToNED = 0;

        unsigned int index = 0;

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourcePoints[i] = getRandomLidarSourcePoint();
            transforms_LidarToRig[i] = transform_LidarToRig;
            transforms_RigToNED[i] = transform_RigToNED;

            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_LidarToRig = getRandomTransform();
                transformChanges_LidarToRig++;
            }
            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_RigToNED = getRandomTransform();
                transformChanges_RigToNED++;
            }
        }

        Q_ASSERT(transformChanges_LidarToRig > 3);
        Q_ASSERT(transformChanges_RigToNED > 3);
        Q_ASSERT(transformChanges_LidarToRig < defaultTestRounds - 10);
        Q_ASSERT(transformChanges_RigToNED < defaultTestRounds - 10);

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            if ((index == 0) || (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index]))))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if ((index == 0) || (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index]))))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourcePoints[index], index);
            filter_Rig_CoordY.addPoint(sourcePoints[index], index);
            filter_Rig_CoordZ.addPoint(sourcePoints[index], index);

            filter_NED_CoordX.addPoint(sourcePoints[index], index);
            filter_NED_CoordY.addPoint(sourcePoints[index], index);
            filter_NED_CoordZ.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            if (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index])))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index])))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourcePoints[index], index);
            filter_Rig_CoordY.addPoint(sourcePoints[index], index);
            filter_Rig_CoordZ.addPoint(sourcePoints[index], index);

            filter_NED_CoordX.addPoint(sourcePoints[index], index);
            filter_NED_CoordY.addPoint(sourcePoints[index], index);
            filter_NED_CoordZ.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Rig_CoordX.getFilteredPoint(out_Rig_X), true);
            QCOMPARE(filter_Rig_CoordY.getFilteredPoint(out_Rig_Y), true);
            QCOMPARE(filter_Rig_CoordZ.getFilteredPoint(out_Rig_Z), true);

            QCOMPARE(filter_NED_CoordX.getFilteredPoint(out_NED_X), true);
            QCOMPARE(filter_NED_CoordY.getFilteredPoint(out_NED_Y), true);
            QCOMPARE(filter_NED_CoordZ.getFilteredPoint(out_NED_Z), true);

            int rigOffsettedIndex = index - (filterBufferLength / 2) + rigOffset;
            int nedOffsettedIndex = index - (filterBufferLength / 2) + nedOffset;

            Eigen::Vector3d rigSourceVector(sourcePoints[rigOffsettedIndex].x, sourcePoints[rigOffsettedIndex].y, sourcePoints[rigOffsettedIndex].z);
            Eigen::Vector3d nedSourceVector(sourcePoints[nedOffsettedIndex].x, sourcePoints[nedOffsettedIndex].y, sourcePoints[nedOffsettedIndex].z);

            Eigen::Vector3d rigVector(out_Rig_X.filterResult, out_Rig_Y.filterResult, out_Rig_Z.filterResult);
            Eigen::Vector3d nedVector(out_NED_X.filterResult, out_NED_Y.filterResult, out_NED_Z.filterResult);

            QVERIFY(compareVectors(rigVector, transforms_LidarToRig[rigOffsettedIndex] * rigSourceVector));
            QVERIFY(compareVectors(nedVector, transforms_RigToNED[nedOffsettedIndex] * (transforms_LidarToRig[nedOffsettedIndex] * nedSourceVector)));

            Eigen::Vector3d expectedNEDOutVector = transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] *
                Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z));

            QVERIFY(compareVectors(out_NED_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Z.coords, expectedNEDOutVector));

            QVERIFY(compareVectors(out_Rig_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Z.coords, expectedNEDOutVector));
        }
    }
}

void TestExpressionFilter::rigAndNEDCoords_Indexed_RandomTransforms_RPLidar()
{
    for (int rigOffset = -((filterBufferLength / 2) - 1); rigOffset < int(filterBufferLength / 2); rigOffset++)
    {
        int nedOffset = -rigOffset;

        RPLidarThread::DistanceItem sourceItems[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordX;
        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordY;
        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordZ;

        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_X;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Y;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Z;

        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordX;
        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordY;
        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordZ;

        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_X;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Y;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Z;

        filter_Rig_CoordX.setExpression_Filter(QString("rig.coord_indexed.x(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordY.setExpression_Filter(QString("rig.coord_indexed.y(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordZ.setExpression_Filter(QString("rig.coord_indexed.z(" + QString::number(rigOffset) + ")"));

        filter_NED_CoordX.setExpression_Filter(QString("ned.coord_indexed.x(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordY.setExpression_Filter(QString("ned.coord_indexed.y(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordZ.setExpression_Filter(QString("ned.coord_indexed.z(" + QString::number(nedOffset) + ")"));

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        unsigned int transformChanges_LidarToRig = 0;
        unsigned int transformChanges_RigToNED = 0;

        unsigned int index = 0;

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourceItems[i] = getRandomRPLidarDistanceItem();
            transforms_LidarToRig[i] = transform_LidarToRig;
            transforms_RigToNED[i] = transform_RigToNED;

            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_LidarToRig = getRandomTransform();
                transformChanges_LidarToRig++;
            }
            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_RigToNED = getRandomTransform();
                transformChanges_RigToNED++;
            }
        }

        Q_ASSERT(transformChanges_LidarToRig > 3);
        Q_ASSERT(transformChanges_RigToNED > 3);
        Q_ASSERT(transformChanges_LidarToRig < defaultTestRounds - 10);
        Q_ASSERT(transformChanges_RigToNED < defaultTestRounds - 10);

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            if ((index == 0) || (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index]))))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if ((index == 0) || (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index]))))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourceItems[index], index);
            filter_Rig_CoordY.addPoint(sourceItems[index], index);
            filter_Rig_CoordZ.addPoint(sourceItems[index], index);

            filter_NED_CoordX.addPoint(sourceItems[index], index);
            filter_NED_CoordY.addPoint(sourceItems[index], index);
            filter_NED_CoordZ.addPoint(sourceItems[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            if (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index])))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index])))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourceItems[index], index);
            filter_Rig_CoordY.addPoint(sourceItems[index], index);
            filter_Rig_CoordZ.addPoint(sourceItems[index], index);

            filter_NED_CoordX.addPoint(sourceItems[index], index);
            filter_NED_CoordY.addPoint(sourceItems[index], index);
            filter_NED_CoordZ.addPoint(sourceItems[index], index);

            QCOMPARE(filter_Rig_CoordX.getFilteredPoint(out_Rig_X), true);
            QCOMPARE(filter_Rig_CoordY.getFilteredPoint(out_Rig_Y), true);
            QCOMPARE(filter_Rig_CoordZ.getFilteredPoint(out_Rig_Z), true);

            QCOMPARE(filter_NED_CoordX.getFilteredPoint(out_NED_X), true);
            QCOMPARE(filter_NED_CoordY.getFilteredPoint(out_NED_Y), true);
            QCOMPARE(filter_NED_CoordZ.getFilteredPoint(out_NED_Z), true);

            int rigOffsettedIndex = index - (filterBufferLength / 2) + rigOffset;
            int nedOffsettedIndex = index - (filterBufferLength / 2) + nedOffset;

            RPLidarThread::DistanceItem rigSourceItem = sourceItems[rigOffsettedIndex];
            RPLidarThread::DistanceItem nedSourceItem = sourceItems[nedOffsettedIndex];

            Eigen::Vector3d rigSourceVector(sin(rigSourceItem.angle) * rigSourceItem.distance, cos(rigSourceItem.angle) * rigSourceItem.distance, 0.0);
            Eigen::Vector3d nedSourceVector(sin(nedSourceItem.angle) * nedSourceItem.distance, cos(nedSourceItem.angle) * nedSourceItem.distance, 0.0);

            Eigen::Vector3d rigVector(out_Rig_X.filterResult, out_Rig_Y.filterResult, out_Rig_Z.filterResult);
            Eigen::Vector3d nedVector(out_NED_X.filterResult, out_NED_Y.filterResult, out_NED_Z.filterResult);

            QVERIFY(compareVectors(rigVector, transforms_LidarToRig[rigOffsettedIndex] * rigSourceVector));
            QVERIFY(compareVectors(nedVector, transforms_RigToNED[nedOffsettedIndex] * (transforms_LidarToRig[nedOffsettedIndex] * nedSourceVector)));

            RPLidarThread::DistanceItem lidarItem = sourceItems[index - (filterBufferLength / 2)];
            Eigen::Vector3d lidarVector(sin(lidarItem.angle) * lidarItem.distance, cos(lidarItem.angle) * lidarItem.distance, 0);

            Eigen::Vector3d expectedNEDOutVector = transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * lidarVector);

            QVERIFY(compareVectors(out_NED_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Z.coords, expectedNEDOutVector));

            QVERIFY(compareVectors(out_Rig_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Z.coords, expectedNEDOutVector));
        }
    }
}

void TestExpressionFilter::convexHullIndexes()
{
    QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;

    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter firstFilter { .Name = "first", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter secondFilter { .Name = "second", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter thirdFilter { .Name = "tHiRd", .filter = ConvexHull::Filter() };

    convexHullFilters.push_back(firstFilter);
    convexHullFilters.push_back(secondFilter);
    convexHullFilters.push_back(thirdFilter);

    PointFilter::ExpressionFilter_Mid360 filter_First;
    PointFilter::ExpressionFilter_Mid360 filter_Second;
    PointFilter::ExpressionFilter_Mid360 filter_Third;
    PointFilter::ExpressionFilter_Mid360 filter_InvalidHullIndexIdent;

    QVERIFY(filter_First.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_Second.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_Third.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_InvalidHullIndexIdent.setConvexHullFilters(convexHullFilters));

    // Calling this again should not change anything
    QVERIFY(filter_Third.setConvexHullFilters(convexHullFilters));

    PointFilter::ExpressionFilter_Mid360::OutItem out_First;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Second;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Third;

    filter_First.setExpression_Filter("chull_first");
    filter_Second.setExpression_Filter("chull_sEcOnD");
    filter_Third.setExpression_Filter("chull_third");

    try
    {
        filter_InvalidHullIndexIdent.setExpression_Filter("chull_InValid//Comment\n");
    }
    catch (PointFilter::ExpressionFilter_Base::Issue &issue)
    {
        QCOMPARE(issue.text, "TinyExpr error: (empty)");
        QCOMPARE(issue.beginChar, filter_InvalidHullIndexIdent.getExpression_Filter().lastIndexOf("//Comment") - 1);
        QCOMPARE(issue.endChar, filter_InvalidHullIndexIdent.getExpression_Filter().lastIndexOf("//Comment") - 1);
    }

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        out_First = getRandomOutItem();
        out_Second = getRandomOutItem();
        out_Third = getRandomOutItem();
        filter_First.addPoint(getRandomLidarSourcePoint(), index);
        filter_Second.addPoint(getRandomLidarSourcePoint(), index);
        filter_Third.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter_First.getFilteredPoint(out_First), false);
        QCOMPARE(filter_Second.getFilteredPoint(out_Second), false);
        QCOMPARE(filter_Third.getFilteredPoint(out_Third), false);
    }

    for (index = 0; index < defaultTestRounds; index++)
    {
        out_First = getRandomOutItem();
        out_Second = getRandomOutItem();
        out_Third = getRandomOutItem();
        filter_First.addPoint(getRandomLidarSourcePoint(), index);
        filter_Second.addPoint(getRandomLidarSourcePoint(), index);
        filter_Third.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter_First.getFilteredPoint(out_First), true);
        QCOMPARE(filter_Second.getFilteredPoint(out_Second), true);
        QCOMPARE(filter_Third.getFilteredPoint(out_Third), true);
        QCOMPARE(out_First.valid, true);
        QCOMPARE(out_First.filterResult, 0);
        QCOMPARE(out_Second.valid, true);
        QCOMPARE(out_Second.filterResult, 1);
        QCOMPARE(out_Third.valid, true);
        QCOMPARE(out_Third.filterResult, 2);
    }
}

static ConvexHull getConvexHullBox(const Eigen::Vector3d& corner1, const Eigen::Vector3d& corner2, const Eigen::Transform<double, 3, Eigen::Affine>& transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity())
{
    ConvexHull hull;

    hull.addPoint(transform * corner1);
    hull.addPoint(transform * Eigen::Vector3d(corner1.x(), corner1.y(), corner2.z()));
    hull.addPoint(transform * Eigen::Vector3d(corner1.x(), corner2.y(), corner1.z()));
    hull.addPoint(transform * Eigen::Vector3d(corner1.x(), corner2.y(), corner2.z()));
    hull.addPoint(transform * Eigen::Vector3d(corner2.x(), corner1.y(), corner1.z()));
    hull.addPoint(transform * Eigen::Vector3d(corner2.x(), corner1.y(), corner2.z()));
    hull.addPoint(transform * Eigen::Vector3d(corner2.x(), corner2.y(), corner1.z()));
    hull.addPoint(transform * corner2);

    return hull;
}

static ConvexHull getConvexHullBox(const Eigen::AlignedBox3d& box, const Eigen::Transform<double, 3, Eigen::Affine>& transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity())
{
    return getConvexHullBox(box.min(), box.max(), transform);
}


void TestExpressionFilter::invalidConvexHullIndexes()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -1.3, 1.3);
    }

    ConvexHull hull = getConvexHullBox(Eigen::Vector3d(-100, -100, -100), Eigen::Vector3d(100, 100, 100));

    ConvexHull::Filter cHullFilter;

    QVERIFY(hull.getFilter(cHullFilter));

    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter hugeOriginBoxCHullFilter { .Name = "hugeoriginbox", .filter = cHullFilter };

    PointFilter::ExpressionFilter_Mid360 exprFilter_Lidar;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Rig;
    PointFilter::ExpressionFilter_Mid360 exprFilter_NED;

    QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(hugeOriginBoxCHullFilter);
    QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));

    // Use invalid hull indexes so "in_convex_hull"-functions should always return false (0)
    // "hugeoriginbox" would be index 0 so that's skipped here
    exprFilter_Lidar.setExpression_Filter("lidar.in_convex_hull(-1, 0)");
    exprFilter_Rig.setExpression_Filter("rig.in_convex_hull(1, 0)");
    exprFilter_NED.setExpression_Filter("NED.In_Convex_Hull(2, 0)");

    unsigned int index;

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);
    }

    PointFilter::ExpressionFilter_Mid360::OutItem out;

    for (; index < defaultTestRounds; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);

        // Invalid hull indexes should always return false
        QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
        QVERIFY(!out.filterResult);
        QVERIFY(exprFilter_Rig.getFilteredPoint(out));
        QVERIFY(!out.filterResult);
        QVERIFY(exprFilter_NED.getFilteredPoint(out));
        QVERIFY(!out.filterResult);
    }
}

void TestExpressionFilter::convexHulls_SingleCubeOnOrigin_DefaultTransforms()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -1.3, 1.3);
    }

    ConvexHull hull = getConvexHullBox(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));

    ConvexHull::Filter cHullFilter;

    QVERIFY(hull.getFilter(cHullFilter));

    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter originBoxCHullFilter { .Name = "originbox", .filter = cHullFilter };

    PointFilter::ExpressionFilter_Mid360 exprFilter_Lidar;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Rig;
    PointFilter::ExpressionFilter_Mid360 exprFilter_NED;

    QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(originBoxCHullFilter);
    QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));

    exprFilter_Lidar.setExpression_Filter("lidar.in_convex_hull(chull_originbox, 0)");
    exprFilter_Rig.setExpression_Filter("rig.in_convex_hull(chull_originbox, 0)");
    exprFilter_NED.setExpression_Filter("NED.In_Convex_Hull(CHULL_OriginBox, 0)");

    unsigned int index;

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);
    }

    const double margin = 0.001;

    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    (void) indeterminates;

    PointFilter::ExpressionFilter_Mid360::OutItem out;

    for (; index < defaultTestRounds; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);

        Eigen::Vector3d point = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);

        if ((std::abs(point.x() - 1.0) < margin) ||
            (std::abs(point.y() - 1.0) < margin) ||
            (std::abs(point.z() - 1.0) < margin))
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            indeterminates++;
            continue;
        }
        else if ((std::abs(point.x()) < 1.0) &&
            (std::abs(point.y()) < 1.0) &&
            (std::abs(point.z()) < 1.0))
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(out.filterResult);
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            QVERIFY(out.filterResult);
            insides++;
        }
        else
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            outsides++;
        }
    }

    Q_ASSERT(insides > 10);
    Q_ASSERT(outsides > 10);
}

void TestExpressionFilter::convexHulls_OverwriteHulls()
{
    // This is heavily based on convexHulls_SingleCubeOnOrigin_DefaultTransforms,
    // there's just multiple setConvexHullFilters calls

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -1.3, 1.3);
    }

    PointFilter::ExpressionFilter_Mid360 exprFilter_Lidar;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Rig;
    PointFilter::ExpressionFilter_Mid360 exprFilter_NED;

    for (int i = 0; i < 10; i++)
    {
        ConvexHull hull1 = getConvexHullBox(Eigen::Vector3d(-10, -10, -10), Eigen::Vector3d(10, 10, 10));
        ConvexHull hull2 = getConvexHullBox(Eigen::Vector3d(-0.1, -0.1, -0.1), Eigen::Vector3d(0.1, 0.1, 0.1));

        ConvexHull::Filter cHullFilter1;
        ConvexHull::Filter cHullFilter2;

        QVERIFY(hull1.getFilter(cHullFilter1));
        QVERIFY(hull1.getFilter(cHullFilter2));

        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter originBoxCHullFilter1 { .Name = "originbox", .filter = cHullFilter1 };
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter originBoxCHullFilter2 { .Name = "anothername", .filter = cHullFilter2 };

        QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;
        convexHullFilters.push_back(originBoxCHullFilter1);
        convexHullFilters.push_back(originBoxCHullFilter2);
        QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
        QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
        QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));
    }

    // Rest is cloned from convexHulls_SingleCubeOnOrigin_DefaultTransforms

    ConvexHull hull = getConvexHullBox(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(1, 1, 1));

    ConvexHull::Filter cHullFilter;

    QVERIFY(hull.getFilter(cHullFilter));

    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter originBoxCHullFilter { .Name = "originbox", .filter = cHullFilter };


    QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(originBoxCHullFilter);
    QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));

    exprFilter_Lidar.setExpression_Filter("lidar.in_convex_hull(chull_originbox, 0)");
    exprFilter_Rig.setExpression_Filter("rig.in_convex_hull(chull_originbox, 0)");
    exprFilter_NED.setExpression_Filter("NED.In_Convex_Hull(CHULL_OriginBox, 0)");

    unsigned int index;

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);
    }

    const double margin = 0.001;

    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    (void) indeterminates;

    PointFilter::ExpressionFilter_Mid360::OutItem out;

    for (; index < defaultTestRounds; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);

        Eigen::Vector3d point = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);

        if ((std::abs(point.x() - 1.0) < margin) ||
            (std::abs(point.y() - 1.0) < margin) ||
            (std::abs(point.z() - 1.0) < margin))
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            indeterminates++;
            continue;
        }
        else if ((std::abs(point.x()) < 1.0) &&
                 (std::abs(point.y()) < 1.0) &&
                 (std::abs(point.z()) < 1.0))
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(out.filterResult);
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            QVERIFY(out.filterResult);
            insides++;
        }
        else
        {
            QVERIFY(exprFilter_Lidar.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            QVERIFY(exprFilter_Rig.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            QVERIFY(exprFilter_NED.getFilteredPoint(out));
            QVERIFY(!out.filterResult);
            outsides++;
        }
    }

    Q_ASSERT(insides > 10);
    Q_ASSERT(outsides > 10);
}


void TestExpressionFilter::convexHulls_TwoStaticCubes_DefaultTransforms()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -1, 1);
    }

    ConvexHull hull1 = getConvexHullBox(Eigen::Vector3d(-1, -1, -1), Eigen::Vector3d(0, 0, 0));
    ConvexHull hull2 = getConvexHullBox(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));

    ConvexHull::Filter cHullFilter1;
    ConvexHull::Filter cHullFilter2;

    QVERIFY(hull1.getFilter(cHullFilter1));
    QVERIFY(hull2.getFilter(cHullFilter2));

    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter boxCHullFilter1 { .Name = "box1", .filter = cHullFilter1 };
    PointFilter::ExpressionFilter_Mid360::ConvexHullFilter boxCHullFilter2 { .Name = "box2", .filter = cHullFilter2 };

    PointFilter::ExpressionFilter_Mid360 exprFilter_Lidar1;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Rig1;
    PointFilter::ExpressionFilter_Mid360 exprFilter_NED1;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Lidar2;
    PointFilter::ExpressionFilter_Mid360 exprFilter_Rig2;
    PointFilter::ExpressionFilter_Mid360 exprFilter_NED2;

    QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(boxCHullFilter1);
    convexHullFilters.push_back(boxCHullFilter2);
    QVERIFY(exprFilter_Lidar1.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig1.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED1.setConvexHullFilters(convexHullFilters));

    QVERIFY(exprFilter_Lidar2.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig2.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED2.setConvexHullFilters(convexHullFilters));

    exprFilter_Lidar1.setExpression_Filter("lidar.in_convex_hull(chull_box1, 0)");
    exprFilter_Rig1.setExpression_Filter("rig.in_convex_hull(chull_box1, 0)");
    exprFilter_NED1.setExpression_Filter("NED.In_Convex_Hull(CHULL_Box1, 0)");

    exprFilter_Lidar2.setExpression_Filter("lidar.in_convex_hull(chull_box2, 0)");
    exprFilter_Rig2.setExpression_Filter("rig.in_convex_hull(chull_box2, 0)");
    exprFilter_NED2.setExpression_Filter("NED.In_Convex_Hull(CHULL_Box2, 0)");

    unsigned int index;

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        exprFilter_Lidar1.addPoint(sourcePoints[index], index);
        exprFilter_Rig1.addPoint(sourcePoints[index], index);
        exprFilter_NED1.addPoint(sourcePoints[index], index);

        exprFilter_Lidar2.addPoint(sourcePoints[index], index);
        exprFilter_Rig2.addPoint(sourcePoints[index], index);
        exprFilter_NED2.addPoint(sourcePoints[index], index);
    }

    const double edgeMargin = 0.001;

    int insides1 = 0;
    int outsides1 = 0;
    int indeterminates1 = 0;
    (void) indeterminates1;

    int insides2 = 0;
    int outsides2 = 0;
    int indeterminates2 = 0;
    (void) indeterminates2;

    PointFilter::ExpressionFilter_Mid360::OutItem out;

    for (; index < defaultTestRounds; index++)
    {
        exprFilter_Lidar1.addPoint(sourcePoints[index], index);
        exprFilter_Rig1.addPoint(sourcePoints[index], index);
        exprFilter_NED1.addPoint(sourcePoints[index], index);

        exprFilter_Lidar2.addPoint(sourcePoints[index], index);
        exprFilter_Rig2.addPoint(sourcePoints[index], index);
        exprFilter_NED2.addPoint(sourcePoints[index], index);

        Eigen::Vector3d point = Eigen::Vector3d(sourcePoints[index - (filterBufferLength / 2)].x, sourcePoints[index - (filterBufferLength / 2)].y, sourcePoints[index - (filterBufferLength / 2)].z);

        if ((point.x() > -1 + edgeMargin) &&
            (point.x() < 0 - edgeMargin) &&
            (point.y() > -1 + edgeMargin) &&
            (point.y() < 0 - edgeMargin) &&
            (point.z() > -1 + edgeMargin) &&
            (point.z() < 0 - edgeMargin))
        {
            QVERIFY(exprFilter_Lidar1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_Rig1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_NED1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            insides1++;
        }
        else if
            ((point.x() < -1 - edgeMargin) ||
            (point.x() > 0 + edgeMargin) ||
            (point.y() < -1 - edgeMargin) ||
            (point.y() > 0 + edgeMargin) ||
            (point.z() < -1 - edgeMargin) ||
            (point.z() > 0 + edgeMargin))
        {
            QVERIFY(exprFilter_Lidar1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            QVERIFY(exprFilter_Rig1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            QVERIFY(exprFilter_NED1.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            outsides1++;
        }
        else
        {
            QVERIFY(exprFilter_Lidar1.getFilteredPoint(out));
            QVERIFY(exprFilter_Rig1.getFilteredPoint(out));
            QVERIFY(exprFilter_NED1.getFilteredPoint(out));
            indeterminates1++;
        }

        if ((point.x() > 0 + edgeMargin) &&
            (point.x() < 1 - edgeMargin) &&
            (point.y() > 0 + edgeMargin) &&
            (point.y() < 1 - edgeMargin) &&
            (point.z() > 0 + edgeMargin) &&
            (point.z() < 1 - edgeMargin))
        {
            QVERIFY(exprFilter_Lidar2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_Rig2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            QVERIFY(exprFilter_NED2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 1.0);
            insides2++;
        }
        else if
            ((point.x() < 0 - edgeMargin) ||
             (point.x() > 1 + edgeMargin) ||
             (point.y() < 0 - edgeMargin) ||
             (point.y() > 1 + edgeMargin) ||
             (point.z() < 0 - edgeMargin) ||
             (point.z() > 1 + edgeMargin))
        {
            QVERIFY(exprFilter_Lidar2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            QVERIFY(exprFilter_Rig2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            QVERIFY(exprFilter_NED2.getFilteredPoint(out));
            QCOMPARE(out.filterResult, 0.0);
            outsides2++;
        }
        else
        {
            QVERIFY(exprFilter_Lidar2.getFilteredPoint(out));
            QVERIFY(exprFilter_Rig2.getFilteredPoint(out));
            QVERIFY(exprFilter_NED2.getFilteredPoint(out));
            indeterminates2++;
        }

    }

    Q_ASSERT(insides1 > 10);
    Q_ASSERT(outsides1 > 10);
    Q_ASSERT(insides2 > 10);
    Q_ASSERT(outsides2 > 10);
}


void TestExpressionFilter::convexHulls_MultipleRandomCubes_RandomTransforms()
{
    const double edgeMargin = 0.01;

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3] = { 0 };
    int outsides[3] = { 0 };
    int indeterminates[3] = { 0 };

    for (int cubeSet = 0; cubeSet < 10; cubeSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

//        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
//        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);

        double hullMargins[3];
        Eigen::AlignedBox3d hullBoxes[3];

        for (int i = 0; i < 3; i++)
        {
            hullMargins[i] = (randomGenerator.generateDouble() - 0.5) * 1.0;

            double x1, x2, y1, y2, z1, z2;

            do
            {
                // Randomize until the box is wide/tall/deep enough
                x1 = (randomGenerator.generateDouble() - 0.5) * 20;
                x2 = (randomGenerator.generateDouble() - 0.5) * 20;
                y1 = (randomGenerator.generateDouble() - 0.5) * 20;
                y2 = (randomGenerator.generateDouble() - 0.5) * 20;
                z1 = (randomGenerator.generateDouble() - 0.5) * 20;
                z2 = (randomGenerator.generateDouble() - 0.5) * 20;
            } while ((fabs(x2 - x1) < 0.5) || (fabs(y2 - y1) < 0.5) || (fabs(z2 - z1) < 0.5));

            hullBoxes[i] = Eigen::AlignedBox3d(Eigen::Vector3d(std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)), Eigen::Vector3d(std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)));

/*            QString fileNamePrefix = "cubes/R" + QString::number(cubeSet) + "B" + QString::number(i);
            ConvexHull hull_Lidar = getConvexHullBox(hullBoxes[i]);
            hull_Lidar.exportHullToObjFile(fileNamePrefix + "_Lidar.obj");
            ConvexHull hull_Rig = getConvexHullBox(hullBoxes[i], transform_LidarToRig.inverse());
            hull_Rig.exportHullToObjFile(fileNamePrefix + "_Rig.obj");
            ConvexHull hull_NED = getConvexHullBox(hullBoxes[i], (transform_RigToNED * transform_LidarToRig).inverse());
            hull_NED.exportHullToObjFile(fileNamePrefix + "_NED.obj");
*/
        }

        ConvexHull::Filter chFilter_Lidar_First;
        QVERIFY(getConvexHullBox(hullBoxes[0]).getFilter(chFilter_Lidar_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_First { .Name = "first", .filter = chFilter_Lidar_First };

        ConvexHull::Filter chFilter_Lidar_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1]).getFilter(chFilter_Lidar_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_Second { .Name = "second", .filter = chFilter_Lidar_Second };


        ConvexHull::Filter chFilter_Rig_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_LidarToRig).getFilter(chFilter_Rig_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_First { .Name = "first_rig", .filter = chFilter_Rig_First };

        ConvexHull::Filter chFilter_Rig_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_LidarToRig).getFilter(chFilter_Rig_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_Second { .Name = "second_rig", .filter = chFilter_Rig_Second };


        ConvexHull::Filter chFilter_NED_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_First { .Name = "first_ned", .filter = chFilter_NED_First };

        ConvexHull::Filter chFilter_NED_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Second { .Name = "second_ned", .filter = chFilter_NED_Second };

        ConvexHull::Filter chFilter_NED_Third;
        QVERIFY(getConvexHullBox(hullBoxes[2], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Third));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Third { .Name = "third_ned", .filter = chFilter_NED_Third };

        QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;

        convexHullFilters.push_back(chullFilter_Expr_Lidar_First);
        convexHullFilters.push_back(chullFilter_Expr_Lidar_Second);
        convexHullFilters.push_back(chullFilter_Expr_Rig_First);
        convexHullFilters.push_back(chullFilter_Expr_Rig_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_First);
        convexHullFilters.push_back(chullFilter_Expr_NED_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_Third);

        QVERIFY(filter_Lidar.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_Rig.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_NED.setConvexHullFilters(convexHullFilters));

        filter_Lidar.setExpression_Filter(QString("lidar.in_convex_hull(chull_first, ") + QString::number(hullMargins[0], 'g', 14) + ")");

        filter_Rig.setExpression_Filter(QString("rig.in_convex_hull(chull_first_rig, ") + QString::number(hullMargins[0], 'g', 14) +
            ") || rig.in_convex_hull(chull_second_rig, " + QString::number(hullMargins[1], 'g', 14) + ")");

        filter_NED.setExpression_Filter(QString("ned.in_convex_hull(chull_first_ned, ") + QString::number(hullMargins[0], 'g', 14) +
            ") || ned.in_convex_hull(chull_second_ned, " + QString::number(hullMargins[1], 'g', 14) +
            ") || ned.in_convex_hull(chull_third_ned, " + QString::number(hullMargins[2], 'g', 14) + ")");

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            int filteredItemIndex = index - (filterBufferLength / 2);
            LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[filteredItemIndex];

#if 0 // Debug-code
            if ((index == 128) && (cubeSet == 27))
            {
                Eigen::Vector3d bugger = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);
                Eigen::Vector3d bugger_rig = transform_LidarToRig.inverse() * bugger;
                Eigen::Vector3d bugger_ned = (transform_RigToNED * transform_LidarToRig).inverse() * bugger;

                QString bugger_xyz = QString::number(bugger.x()) + " " + QString::number(bugger.y()) + " " + QString::number(bugger.z());
                QString bugger_Rig_xyz = QString::number(bugger_rig.x()) + " " + QString::number(bugger_rig.y()) + " " + QString::number(bugger_rig.z());
                QString bugger_NED_xyz = QString::number(bugger_ned.x()) + " " + QString::number(bugger_ned.y()) + " " + QString::number(bugger_ned.z());
            }
#endif
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);

            bool shouldBeInside[3] = { 0 };
            bool shouldBeOutside[3] = { 0 }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3] = { 0 };

            for (int i = 0; i < 3; i++)
            {
                if ((sourcePoint.x > hullBoxes[i].min().x() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.x < hullBoxes[i].max().x() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.y > hullBoxes[i].min().y() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.y < hullBoxes[i].max().y() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.z > hullBoxes[i].min().z() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.z < hullBoxes[i].max().z() + hullMargins[i] - edgeMargin))
                {
                    shouldBeInside[i] = true;
                    insides[i]++;
                }
                else if (
                    (sourcePoint.x < hullBoxes[i].min().x() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.x > hullBoxes[i].max().x() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.y < hullBoxes[i].min().y() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.y > hullBoxes[i].max().y() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.z < hullBoxes[i].min().z() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.z > hullBoxes[i].max().z() + hullMargins[i] + edgeMargin))
                {
                    shouldBeOutside[i] = true;
                    outsides[i]++;
                }
                else
                {
                    indeterminate[i] = true;
                    indeterminates[i]++;
                }
            }

#if 0 // Debugging-code
            if (
                ((!indeterminate[0]) &&
                    (out_Lidar.filterResult != shouldBeInside[0])) ||
                ((!indeterminate[0] && !indeterminate[1]) &&
                    (out_Rig.filterResult != (shouldBeInside[0] || shouldBeInside[1]))) ||
                ((!indeterminate[0] && !indeterminate[1] && !indeterminate[2]) &&
                    (out_NED.filterResult != (shouldBeInside[0] || shouldBeInside[1] || shouldBeInside[2])))
                )
            {
                Eigen::Vector3d bugger = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);
                Eigen::Vector3d bugger_rig = transform_LidarToRig.inverse() * bugger;
                Eigen::Vector3d bugger_ned = (transform_RigToNED * transform_LidarToRig).inverse() * bugger;

                QString bugger_xyz = QString::number(bugger.x()) + " " + QString::number(bugger.y()) + " " + QString::number(bugger.z());
                QString bugger_Rig_xyz = QString::number(bugger_rig.x()) + " " + QString::number(bugger_rig.y()) + " " + QString::number(bugger_rig.z());
                QString bugger_NED_xyz = QString::number(bugger_ned.x()) + " " + QString::number(bugger_ned.y()) + " " + QString::number(bugger_ned.z());

                bool in_lidar_1 = convexHullFilters[0].filter.isInside(bugger);
                bool in_lidar_2 = convexHullFilters[1].filter.isInside(bugger);
                bool in_rig_1 = convexHullFilters[2].filter.isInside(bugger_rig);
                bool in_rig_2 = convexHullFilters[3].filter.isInside(bugger_rig);
                bool in_ned_1 = convexHullFilters[4].filter.isInside(bugger_ned);
                bool in_ned_2 = convexHullFilters[5].filter.isInside(bugger_ned);
                bool in_ned_3 = convexHullFilters[6].filter.isInside(bugger_ned);

                ConvexHull::Filter chFilter_Rig_First2;
                QVERIFY(getConvexHullBox(hullBoxes[0], transform_LidarToRig).getFilter(chFilter_Rig_First2));
                //                    PointFilter::ExpressionFilter::ConvexHullFilter chullFilter_Expr_Rig_First { .Name = "first_rig", .filter = chFilter_Rig_First };
                bool in_rig_1_2 = chFilter_Rig_First2.isInside(bugger_rig);

                for (int i = 0; i < 3; i++)
                {
                    QString fileNamePrefix = "cuboids/R" + QString::number(cubeSet) + "B" + QString::number(i);
                    ConvexHull hull_Lidar = getConvexHullBox(hullBoxes[i]);
                    hull_Lidar.exportHullToObjFile(fileNamePrefix + "_Lidar");
                    ConvexHull hull_Rig = getConvexHullBox(hullBoxes[i], transform_LidarToRig.inverse());
                    hull_Rig.exportHullToObjFile(fileNamePrefix + "_Rig");
                    ConvexHull hull_NED = getConvexHullBox(hullBoxes[i], (transform_RigToNED * transform_LidarToRig).inverse());
                    hull_NED.exportHullToObjFile(fileNamePrefix + "_NED");
                }
            }
#endif

            if (!indeterminate[0])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0]);
            }

            if (!indeterminate[0] && !indeterminate[1])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[0] || shouldBeInside[1]);
            }

            if (!indeterminate[0] && !indeterminate[1] && !indeterminate[2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[0] || shouldBeInside[1] || shouldBeInside[2]);
            }
        }
    }

//    int foo = 0; // Debug-trap
}


void TestExpressionFilter::convexHulls_MultipleRandomCubes_RandomTransforms_Indexed()
{
    // This is almost identical to "non-indexed"-version above, only indexing added
    // and removed some debugging code that would not work with indexing
    // and added invalid hull indexes (out-of-range "sample indexes" just return undefined values)

    const double edgeMargin = 0.01;

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;
    PointFilter::ExpressionFilter_Mid360::OutItem out_InvalidHullIndexes;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3] = { 0 };
    int outsides[3] = { 0 };
    int indeterminates[3] = { 0 };

    for (int cubeSet = 0; cubeSet < 10; cubeSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;
        PointFilter::ExpressionFilter_Mid360 filter_InvalidHullIndexes;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);
        filter_InvalidHullIndexes.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);
        filter_InvalidHullIndexes.setTransform_RigToNED(transform_RigToNED);

        double hullMargins[3];
        int indexes[3];
        Eigen::AlignedBox3d hullBoxes[3];

        for (int i = 0; i < 3; i++)
        {
            hullMargins[i] = (randomGenerator.generateDouble() - 0.5) * 1.0;
            indexes[i] = randomGenerator.bounded(-7, 8);

            double x1, x2, y1, y2, z1, z2;

            do
            {
                // Randomize until the box is wide/tall/deep enough
                x1 = (randomGenerator.generateDouble() - 0.5) * 20;
                x2 = (randomGenerator.generateDouble() - 0.5) * 20;
                y1 = (randomGenerator.generateDouble() - 0.5) * 20;
                y2 = (randomGenerator.generateDouble() - 0.5) * 20;
                z1 = (randomGenerator.generateDouble() - 0.5) * 20;
                z2 = (randomGenerator.generateDouble() - 0.5) * 20;
            } while ((fabs(x2 - x1) < 0.5) || (fabs(y2 - y1) < 0.5) || (fabs(z2 - z1) < 0.5));

            hullBoxes[i] = Eigen::AlignedBox3d(Eigen::Vector3d(std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)), Eigen::Vector3d(std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)));
        }

        ConvexHull::Filter chFilter_Lidar_First;
        QVERIFY(getConvexHullBox(hullBoxes[0]).getFilter(chFilter_Lidar_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_First { .Name = "first", .filter = chFilter_Lidar_First };

        ConvexHull::Filter chFilter_Lidar_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1]).getFilter(chFilter_Lidar_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_Second { .Name = "second", .filter = chFilter_Lidar_Second };


        ConvexHull::Filter chFilter_Rig_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_LidarToRig).getFilter(chFilter_Rig_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_First { .Name = "first_rig", .filter = chFilter_Rig_First };

        ConvexHull::Filter chFilter_Rig_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_LidarToRig).getFilter(chFilter_Rig_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_Second { .Name = "second_rig", .filter = chFilter_Rig_Second };


        ConvexHull::Filter chFilter_NED_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_First { .Name = "first_ned", .filter = chFilter_NED_First };

        ConvexHull::Filter chFilter_NED_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Second { .Name = "second_ned", .filter = chFilter_NED_Second };

        ConvexHull::Filter chFilter_NED_Third;
        QVERIFY(getConvexHullBox(hullBoxes[2], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Third));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Third { .Name = "third_ned", .filter = chFilter_NED_Third };

        QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;

        convexHullFilters.push_back(chullFilter_Expr_Lidar_First);
        convexHullFilters.push_back(chullFilter_Expr_Lidar_Second);
        convexHullFilters.push_back(chullFilter_Expr_Rig_First);
        convexHullFilters.push_back(chullFilter_Expr_Rig_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_First);
        convexHullFilters.push_back(chullFilter_Expr_NED_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_Third);

        QVERIFY(filter_Lidar.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_Rig.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_NED.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_InvalidHullIndexes.setConvexHullFilters(convexHullFilters));

        filter_Lidar.setExpression_Filter(QString("lidar.in_convex_hull_indexed(chull_first, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) + ")");

        filter_Rig.setExpression_Filter(QString("rig.in_convex_hull_indexed(chull_first_rig, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) +
                                                ") || rig.in_convex_hull_indexed(chull_second_rig, " + QString::number(hullMargins[1], 'g', 14) + ", " + QString::number(indexes[1]) + ")");

        filter_NED.setExpression_Filter(QString("ned.in_convex_hull_indexed(chull_first_ned, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) +
                                                ") || ned.in_convex_hull_indexed(chull_second_ned, " + QString::number(hullMargins[1], 'g', 14) + ", " + QString::number(indexes[1]) +
                                                ") || ned.in_convex_hull_indexed(chull_third_ned, " + QString::number(hullMargins[2], 'g', 14) + ", " + QString::number(indexes[2]) + ")");

        filter_InvalidHullIndexes.setExpression_Filter(
            "lidar.in_convex_hull_indexed(chull_first - 1, 0, 0) || "
            "lidar.in_convex_hull_indexed(-1, 0, -65) || "
            "rig.in_convex_hull_indexed(7, 0, 9) || "
            "rig.in_convex_hull_indexed(8, 0, 42) || "
            "ned.in_convex_hull_indexed(42, 0, 1337) || "
            "ned.in_convex_hull_indexed(1337, 0, 0xabba) || "
            "ned.in_convex_hull_indexed(chull_third_ned + 1, 0, 12345678)"
            );

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
            filter_InvalidHullIndexes.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
            filter_InvalidHullIndexes.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);
            QCOMPARE(filter_InvalidHullIndexes.getFilteredPoint(out_InvalidHullIndexes), true);

            bool shouldBeInside[3] = { 0 };
            bool shouldBeOutside[3] = { 0 }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3] = { 0 };

            for (int i = 0; i < 3; i++)
            {
                LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[index - (filterBufferLength / 2) + indexes[i]];

                if ((sourcePoint.x > hullBoxes[i].min().x() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.x < hullBoxes[i].max().x() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.y > hullBoxes[i].min().y() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.y < hullBoxes[i].max().y() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.z > hullBoxes[i].min().z() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.z < hullBoxes[i].max().z() + hullMargins[i] - edgeMargin))
                {
                    shouldBeInside[i] = true;
                    insides[i]++;
                }
                else if (
                    (sourcePoint.x < hullBoxes[i].min().x() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.x > hullBoxes[i].max().x() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.y < hullBoxes[i].min().y() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.y > hullBoxes[i].max().y() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.z < hullBoxes[i].min().z() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.z > hullBoxes[i].max().z() + hullMargins[i] + edgeMargin))
                {
                    shouldBeOutside[i] = true;
                    outsides[i]++;
                }
                else
                {
                    indeterminate[i] = true;
                    indeterminates[i]++;
                }
            }

            if (!indeterminate[0])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0]);
            }

            if (!indeterminate[0] && !indeterminate[1])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[0] || shouldBeInside[1]);
            }

            if (!indeterminate[0] && !indeterminate[1] && !indeterminate[2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[0] || shouldBeInside[1] || shouldBeInside[2]);
            }

            QVERIFY(out_InvalidHullIndexes.filterResult == 0);
        }
    }

    //    int foo = 0; // Debug-trap
}


void TestExpressionFilter::in_aabb_MultipleRandomCubes_RandomTransforms()
{
    const double edgeMargin = 1e-3; // Edges are exact, but there will be some rounding errors on the rotated/translated ones

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3] = { 0 };
    int outsides[3] = { 0 };
    int indeterminates[3] = { 0 };

    for (int aabbSet = 0; aabbSet < 10; aabbSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        //        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);

        // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
        Eigen::AlignedBox3d aabbBoxes[3][3];

        for (int i = 0; i < 3; i++)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                double x1, x2, y1, y2, z1, z2;

                do
                {
                    // Randomize until the box is wide/tall/deep enough
                    x1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    x2 = (randomGenerator.generateDouble() - 0.5) * 20;
                    y1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    y2 = (randomGenerator.generateDouble() - 0.5) * 20;
                    z1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    z2 = (randomGenerator.generateDouble() - 0.5) * 20;
                } while ((fabs(x2 - x1) < 0.5) || (fabs(y2 - y1) < 0.5) || (fabs(z2 - z1) < 0.5));

                aabbBoxes[i][ii] = Eigen::AlignedBox3d(Eigen::Vector3d(std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)), Eigen::Vector3d(std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)));
            }
        }

        filter_Lidar.setExpression_Filter(
            QString("lidar.in_aabb(") +
                QString::number(aabbBoxes[0][0].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][0].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][0].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][0].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][0].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][0].max().z(), 'g', 14) +
            ") || lidar.in_aabb(" +
                QString::number(aabbBoxes[0][1].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][1].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][1].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][1].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][1].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][1].max().z(), 'g', 14) +
            ") || lidar.in_aabb(" +
                QString::number(aabbBoxes[0][2].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][2].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][2].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][2].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][2].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[0][2].max().z(), 'g', 14) +
            ")");

        filter_Rig.setExpression_Filter(
            QString("rig.in_aabb(") +
                QString::number(aabbBoxes[1][0].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][0].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][0].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][0].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][0].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][0].max().z(), 'g', 14) +
            ") || rig.in_aabb(" +
                QString::number(aabbBoxes[1][1].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][1].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][1].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][1].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][1].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][1].max().z(), 'g', 14) +
            ") || rig.in_aabb(" +
                QString::number(aabbBoxes[1][2].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][2].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][2].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][2].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][2].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[1][2].max().z(), 'g', 14) +
            ")");

        filter_NED.setExpression_Filter(
            QString("ned.in_aabb(") +
                QString::number(aabbBoxes[2][0].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][0].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][0].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][0].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][0].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][0].max().z(), 'g', 14) +
            ") || ned.in_aabb(" +
                QString::number(aabbBoxes[2][1].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][1].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][1].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][1].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][1].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][1].max().z(), 'g', 14) +
            ") || ned.in_aabb(" +
                QString::number(aabbBoxes[2][2].min().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][2].min().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][2].min().z(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][2].max().x(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][2].max().y(), 'g', 14) + ", " +
                QString::number(aabbBoxes[2][2].max().z(), 'g', 14) +
            ")");

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            int filteredItemIndex = index - (filterBufferLength / 2);
            LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[filteredItemIndex];
            Eigen::Vector3d eigenSourcePoint = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);

            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);

            // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
            bool shouldBeInside[3][3] = { {0, 0, 0 } };
            bool shouldBeOutside[3][3] = { {0, 0, 0 } }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3][3] = { {0, 0, 0 } };

            for (int i = 0; i < 3; i++)
            {
                Eigen::Vector3d transformedSourcePoint;

                switch (i)
                {
                    case 0:
                        transformedSourcePoint = eigenSourcePoint;
                        break;

                    case 1:
                        transformedSourcePoint = transform_LidarToRig * eigenSourcePoint;
                        break;

                    case 2:
                        transformedSourcePoint = transform_RigToNED * (transform_LidarToRig * eigenSourcePoint);
                        break;
                }

                for (int ii = 0; ii < 3; ii++)
                {
                    if ((transformedSourcePoint.x() > aabbBoxes[i][ii].min().x() + edgeMargin) &&
                        (transformedSourcePoint.x() < aabbBoxes[i][ii].max().x() - edgeMargin) &&
                        (transformedSourcePoint.y() > aabbBoxes[i][ii].min().y() + edgeMargin) &&
                        (transformedSourcePoint.y() < aabbBoxes[i][ii].max().y() - edgeMargin) &&
                        (transformedSourcePoint.z() > aabbBoxes[i][ii].min().z() + edgeMargin) &&
                        (transformedSourcePoint.z() < aabbBoxes[i][ii].max().z() - edgeMargin))
                    {
                        shouldBeInside[i][ii] = true;
                        insides[i]++;
                    }
                    else if (
                        (transformedSourcePoint.x() < aabbBoxes[i][ii].min().x() - edgeMargin) ||
                        (transformedSourcePoint.x() > aabbBoxes[i][ii].max().x() + edgeMargin) ||
                        (transformedSourcePoint.y() < aabbBoxes[i][ii].min().y() - edgeMargin) ||
                        (transformedSourcePoint.y() > aabbBoxes[i][ii].max().y() + edgeMargin) ||
                        (transformedSourcePoint.z() < aabbBoxes[i][ii].min().z() - edgeMargin) ||
                        (transformedSourcePoint.z() > aabbBoxes[i][ii].max().z() + edgeMargin))
                    {
                        shouldBeOutside[i][ii] = true;
                        outsides[i]++;
                    }
                    else
                    {
                        indeterminate[i][ii] = true;
                        indeterminates[i]++;
                    }
                }
            }

            if (!indeterminate[0][0] && !indeterminate[0][1] && !indeterminate[0][2])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0][0] || shouldBeInside[0][1] || shouldBeInside[0][2]);
            }

            if (!indeterminate[1][0] && !indeterminate[1][1] && !indeterminate[1][2])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[1][0] || shouldBeInside[1][1] || shouldBeInside[1][2]);
            }

            if (!indeterminate[2][0] && !indeterminate[2][1] && !indeterminate[2][2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[2][0] || shouldBeInside[2][1] || shouldBeInside[2][2]);
            }
        }
    }

//        int foo = 0; // Debug-trap
}

void TestExpressionFilter::in_aabb_MultipleRandomCubes_RandomTransforms_Indexed()
{
    const double edgeMargin = 1e-3; // Edges are exact, but there will be some rounding errors on the rotated/translated ones

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3] = { 0 };
    int outsides[3] = { 0 };
    int indeterminates[3] = { 0 };

    for (int aabbSet = 0; aabbSet < 10; aabbSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        //        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);

        // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
        Eigen::AlignedBox3d aabbBoxes[3][3];
        int indexes[3][3];

        for (int i = 0; i < 3; i++)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                indexes[i][ii] = randomGenerator.bounded(-7, 8);

                double x1, x2, y1, y2, z1, z2;

                do
                {
                    // Randomize until the box is wide/tall/deep enough
                    x1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    x2 = (randomGenerator.generateDouble() - 0.5) * 20;
                    y1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    y2 = (randomGenerator.generateDouble() - 0.5) * 20;
                    z1 = (randomGenerator.generateDouble() - 0.5) * 20;
                    z2 = (randomGenerator.generateDouble() - 0.5) * 20;
                } while ((fabs(x2 - x1) < 0.5) || (fabs(y2 - y1) < 0.5) || (fabs(z2 - z1) < 0.5));

                aabbBoxes[i][ii] = Eigen::AlignedBox3d(Eigen::Vector3d(std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)), Eigen::Vector3d(std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)));
            }
        }

        filter_Lidar.setExpression_Filter(
            QString("lidar.in_aabb_indexed(") +
            QString::number(aabbBoxes[0][0].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][0].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][0].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][0].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][0].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][0].max().z(), 'g', 14) + ", " +
            QString::number(indexes[0][0]) +
            ") || lidar.in_aabb_indexed(" +
            QString::number(aabbBoxes[0][1].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][1].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][1].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][1].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][1].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][1].max().z(), 'g', 14) + ", " +
            QString::number(indexes[0][1]) +
            ") || lidar.in_aabb_indexed(" +
            QString::number(aabbBoxes[0][2].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][2].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][2].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][2].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][2].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[0][2].max().z(), 'g', 14) + ", " +
            QString::number(indexes[0][2]) +
            ")");

        filter_Rig.setExpression_Filter(
            QString("rig.in_aabb_indexed(") +
            QString::number(aabbBoxes[1][0].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][0].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][0].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][0].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][0].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][0].max().z(), 'g', 14) + ", " +
            QString::number(indexes[1][0]) +
            ") || rig.in_aabb_indexed(" +
            QString::number(aabbBoxes[1][1].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][1].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][1].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][1].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][1].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][1].max().z(), 'g', 14) + ", " +
            QString::number(indexes[1][1]) +
            ") || rig.in_aabb_indexed(" +
            QString::number(aabbBoxes[1][2].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][2].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][2].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][2].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][2].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[1][2].max().z(), 'g', 14) + ", " +
            QString::number(indexes[1][2]) +
            ")");

        filter_NED.setExpression_Filter(
            QString("ned.in_aabb_indexed(") +
            QString::number(aabbBoxes[2][0].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][0].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][0].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][0].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][0].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][0].max().z(), 'g', 14) + ", " +
            QString::number(indexes[2][0]) +
            ") || ned.in_aabb_indexed(" +
            QString::number(aabbBoxes[2][1].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][1].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][1].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][1].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][1].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][1].max().z(), 'g', 14) + ", " +
            QString::number(indexes[2][1]) +
            ") || ned.in_aabb_indexed(" +
            QString::number(aabbBoxes[2][2].min().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][2].min().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][2].min().z(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][2].max().x(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][2].max().y(), 'g', 14) + ", " +
            QString::number(aabbBoxes[2][2].max().z(), 'g', 14) + ", " +
            QString::number(indexes[2][2]) +
            ")");

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);

            // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
            bool shouldBeInside[3][3] = { {0, 0, 0 } };
            bool shouldBeOutside[3][3] = { {0, 0, 0 } }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3][3] = { {0, 0, 0 } };

            for (int i = 0; i < 3; i++)
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    int filteredItemIndex = index - (filterBufferLength / 2) + indexes[i][ii];
                    LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[filteredItemIndex];
                    Eigen::Vector3d eigenSourcePoint = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);

                    Eigen::Vector3d transformedSourcePoint;

                    switch (i)
                    {
                    case 0:
                        transformedSourcePoint = eigenSourcePoint;
                        break;

                    case 1:
                        transformedSourcePoint = transform_LidarToRig * eigenSourcePoint;
                        break;

                    case 2:
                        transformedSourcePoint = transform_RigToNED * (transform_LidarToRig * eigenSourcePoint);
                        break;
                    }

                    if ((transformedSourcePoint.x() > aabbBoxes[i][ii].min().x() + edgeMargin) &&
                        (transformedSourcePoint.x() < aabbBoxes[i][ii].max().x() - edgeMargin) &&
                        (transformedSourcePoint.y() > aabbBoxes[i][ii].min().y() + edgeMargin) &&
                        (transformedSourcePoint.y() < aabbBoxes[i][ii].max().y() - edgeMargin) &&
                        (transformedSourcePoint.z() > aabbBoxes[i][ii].min().z() + edgeMargin) &&
                        (transformedSourcePoint.z() < aabbBoxes[i][ii].max().z() - edgeMargin))
                    {
                        shouldBeInside[i][ii] = true;
                        insides[i]++;
                    }
                    else if (
                        (transformedSourcePoint.x() < aabbBoxes[i][ii].min().x() - edgeMargin) ||
                        (transformedSourcePoint.x() > aabbBoxes[i][ii].max().x() + edgeMargin) ||
                        (transformedSourcePoint.y() < aabbBoxes[i][ii].min().y() - edgeMargin) ||
                        (transformedSourcePoint.y() > aabbBoxes[i][ii].max().y() + edgeMargin) ||
                        (transformedSourcePoint.z() < aabbBoxes[i][ii].min().z() - edgeMargin) ||
                        (transformedSourcePoint.z() > aabbBoxes[i][ii].max().z() + edgeMargin))
                    {
                        shouldBeOutside[i][ii] = true;
                        outsides[i]++;
                    }
                    else
                    {
                        indeterminate[i][ii] = true;
                        indeterminates[i]++;
                    }
                }
            }

            if (!indeterminate[0][0] && !indeterminate[0][1] && !indeterminate[0][2])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0][0] || shouldBeInside[0][1] || shouldBeInside[0][2]);
            }

            if (!indeterminate[1][0] && !indeterminate[1][1] && !indeterminate[1][2])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[1][0] || shouldBeInside[1][1] || shouldBeInside[1][2]);
            }

            if (!indeterminate[2][0] && !indeterminate[2][1] && !indeterminate[2][2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[2][0] || shouldBeInside[2][1] || shouldBeInside[2][2]);
            }
        }
    }

    //    int foo = 0; // Debug-trap
}

void TestExpressionFilter::in_sphere_MultipleRandomSpheres_RandomTransforms()
{
    const double edgeMargin = 1e-6;

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3][3] = { { 0, 0, 0 } };
    int outsides[3][3] = { { 0, 0, 0 } };
    int indeterminates[3][3] = { { 0, 0, 0 } };

    for (int sphereSet = 0; sphereSet < 10; sphereSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        //        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);

        struct Sphere
        {
            Eigen::Vector3d centerPoint;
            double radius;
        };

        // Indexing: [space (lidar = 0, rig = 1, NED = 2][sphere #]
        Sphere spheres[3][3];

        for (int i = 0; i < 3; i++)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                double centerX, centerY, centerZ, radius;

                centerX = (randomGenerator.generateDouble() - 0.5) * 20;
                centerY = (randomGenerator.generateDouble() - 0.5) * 20;
                centerZ = (randomGenerator.generateDouble() - 0.5) * 20;
                radius = randomGenerator.generateDouble() * 10;

                spheres[i][ii] = Sphere { .centerPoint = Eigen::Vector3d(centerX, centerY, centerZ), .radius = radius };
            }
        }

        filter_Lidar.setExpression_Filter(
            QString("lidar.in_sphere(") +
            QString::number(spheres[0][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][0].radius, 'g', 14) +
            ") || lidar.in_sphere(" +
            QString::number(spheres[0][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][1].radius, 'g', 14) +
            ") || lidar.in_sphere(" +
            QString::number(spheres[0][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][2].radius, 'g', 14) +
            ")");

        filter_Rig.setExpression_Filter(
            QString("rig.in_sphere(") +
            QString::number(spheres[1][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][0].radius, 'g', 14) +
            ") || rig.in_sphere(" +
            QString::number(spheres[1][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][1].radius, 'g', 14) +
            ") || rig.in_sphere(" +
            QString::number(spheres[1][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][2].radius, 'g', 14) +
            ")");

        filter_NED.setExpression_Filter(
            QString("ned.in_sphere(") +
            QString::number(spheres[2][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][0].radius, 'g', 14) +
            ") || ned.in_sphere(" +
            QString::number(spheres[2][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][1].radius, 'g', 14) +
            ") || ned.in_sphere(" +
            QString::number(spheres[2][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][2].radius, 'g', 14) +
            ")");

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            int filteredItemIndex = index - (filterBufferLength / 2);
            LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[filteredItemIndex];
            Eigen::Vector3d eigenSourcePoint = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);

            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);

            // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
            bool shouldBeInside[3][3] = { {0, 0, 0 } };
            bool shouldBeOutside[3][3] = { {0, 0, 0 } }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3][3] = { {0, 0, 0 } };

            for (int i = 0; i < 3; i++)
            {
                Eigen::Vector3d transformedSourcePoint;

                switch (i)
                {
                case 0:
                    transformedSourcePoint = eigenSourcePoint;
                    break;

                case 1:
                    transformedSourcePoint = transform_LidarToRig * eigenSourcePoint;
                    break;

                case 2:
                    transformedSourcePoint = transform_RigToNED * (transform_LidarToRig * eigenSourcePoint);
                    break;
                }

                for (int ii = 0; ii < 3; ii++)
                {
                    if ((transformedSourcePoint - spheres[i][ii].centerPoint).norm() < spheres[i][ii].radius - edgeMargin)
                    {
                        shouldBeInside[i][ii] = true;
                        insides[i][ii]++;
                    }
                    else if ((transformedSourcePoint - spheres[i][ii].centerPoint).norm() > spheres[i][ii].radius + edgeMargin)
                    {
                        shouldBeOutside[i][ii] = true;
                        outsides[i][ii]++;
                    }
                    else
                    {
                        indeterminate[i][ii] = true;
                        indeterminates[i][ii]++;
                    }
                }
            }

            if (!indeterminate[0][0] && !indeterminate[0][1] && !indeterminate[0][2])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0][0] || shouldBeInside[0][1] || shouldBeInside[0][2]);
            }

            if (!indeterminate[1][0] && !indeterminate[1][1] && !indeterminate[1][2])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[1][0] || shouldBeInside[1][1] || shouldBeInside[1][2]);
            }

            if (!indeterminate[2][0] && !indeterminate[2][1] && !indeterminate[2][2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[2][0] || shouldBeInside[2][1] || shouldBeInside[2][2]);
            }
        }
    }

//    int foo = 0; // Debug-trap
}

void TestExpressionFilter::in_sphere_MultipleRandomSpheres_RandomTransforms_Indexed()
{
    const double edgeMargin = 1e-6;

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3][3] = { { 0, 0, 0 } };
    int outsides[3][3] = { { 0, 0, 0 } };
    int indeterminates[3][3] = { { 0, 0, 0 } };

    for (int sphereSet = 0; sphereSet < 10; sphereSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        //        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);

        struct Sphere
        {
            Eigen::Vector3d centerPoint;
            double radius;
        };

        // Indexing: [space (lidar = 0, rig = 1, NED = 2][sphere #]
        Sphere spheres[3][3];
        int indexes[3][3];

        for (int i = 0; i < 3; i++)
        {
            for (int ii = 0; ii < 3; ii++)
            {
                indexes[i][ii] = randomGenerator.bounded(-7, 8);

                double centerX, centerY, centerZ, radius;

                centerX = (randomGenerator.generateDouble() - 0.5) * 20;
                centerY = (randomGenerator.generateDouble() - 0.5) * 20;
                centerZ = (randomGenerator.generateDouble() - 0.5) * 20;
                radius = randomGenerator.generateDouble() * 10;

                spheres[i][ii] = Sphere { .centerPoint = Eigen::Vector3d(centerX, centerY, centerZ), .radius = radius };
            }
        }

        filter_Lidar.setExpression_Filter(
            QString("lidar.in_sphere_indexed(") +
            QString::number(spheres[0][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][0].radius, 'g', 14) + ", " +
            QString::number(indexes[0][0]) +
            ") || lidar.in_sphere_indexed(" +
            QString::number(spheres[0][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][1].radius, 'g', 14) + ", " +
            QString::number(indexes[0][1]) +
            ") || lidar.in_sphere_indexed(" +
            QString::number(spheres[0][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[0][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[0][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[0][2].radius, 'g', 14) + ", " +
            QString::number(indexes[0][2]) +
            ")");

        filter_Rig.setExpression_Filter(
            QString("rig.in_sphere_indexed(") +
            QString::number(spheres[1][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][0].radius, 'g', 14) + ", " +
            QString::number(indexes[1][0]) +
            ") || rig.in_sphere_indexed(" +
            QString::number(spheres[1][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][1].radius, 'g', 14) + ", " +
            QString::number(indexes[1][1]) +
            ") || rig.in_sphere_indexed(" +
            QString::number(spheres[1][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[1][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[1][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[1][2].radius, 'g', 14) + ", " +
            QString::number(indexes[1][2]) +
            ")");

        filter_NED.setExpression_Filter(
            QString("ned.in_sphere_indexed(") +
            QString::number(spheres[2][0].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][0].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][0].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][0].radius, 'g', 14) + ", " +
            QString::number(indexes[2][0]) +
            ") || ned.in_sphere_indexed(" +
            QString::number(spheres[2][1].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][1].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][1].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][1].radius, 'g', 14) + ", " +
            QString::number(indexes[2][1]) +
            ") || ned.in_sphere_indexed(" +
            QString::number(spheres[2][2].centerPoint.x(), 'g', 14) + ", " +
            QString::number(spheres[2][2].centerPoint.y(), 'g', 14) + ", " +
            QString::number(spheres[2][2].centerPoint.z(), 'g', 14) + ", " +
            QString::number(spheres[2][2].radius, 'g', 14) + ", " +
            QString::number(indexes[2][2]) +
            ")");

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
        }

        for (; index < defaultTestRounds; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);

            // Indexing: [space (lidar = 0, rig = 1, NED = 2][box #]
            bool shouldBeInside[3][3] = { {0, 0, 0 } };
            bool shouldBeOutside[3][3] = { {0, 0, 0 } }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3][3] = { {0, 0, 0 } };

            for (int i = 0; i < 3; i++)
            {
                for (int ii = 0; ii < 3; ii++)
                {
                    int filteredItemIndex = index - (filterBufferLength / 2) + indexes[i][ii];
                    LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[filteredItemIndex];
                    Eigen::Vector3d eigenSourcePoint = Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);

                    Eigen::Vector3d transformedSourcePoint;

                    switch (i)
                    {
                    case 0:
                        transformedSourcePoint = eigenSourcePoint;
                        break;

                    case 1:
                        transformedSourcePoint = transform_LidarToRig * eigenSourcePoint;
                        break;

                    case 2:
                        transformedSourcePoint = transform_RigToNED * (transform_LidarToRig * eigenSourcePoint);
                        break;
                    }

                    if ((transformedSourcePoint - spheres[i][ii].centerPoint).norm() < spheres[i][ii].radius - edgeMargin)
                    {
                        shouldBeInside[i][ii] = true;
                        insides[i][ii]++;
                    }
                    else if ((transformedSourcePoint - spheres[i][ii].centerPoint).norm() > spheres[i][ii].radius + edgeMargin)
                    {
                        shouldBeOutside[i][ii] = true;
                        outsides[i][ii]++;
                    }
                    else
                    {
                        indeterminate[i][ii] = true;
                        indeterminates[i][ii]++;
                    }
                }
            }

            if (!indeterminate[0][0] && !indeterminate[0][1] && !indeterminate[0][2])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0][0] || shouldBeInside[0][1] || shouldBeInside[0][2]);
            }

            if (!indeterminate[1][0] && !indeterminate[1][1] && !indeterminate[1][2])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[1][0] || shouldBeInside[1][1] || shouldBeInside[1][2]);
            }

            if (!indeterminate[2][0] && !indeterminate[2][1] && !indeterminate[2][2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[2][0] || shouldBeInside[2][1] || shouldBeInside[2][2]);
            }
        }
    }

//    int foo = 0; // Debug-trap
}

void TestExpressionFilter::copyingFilters()
{
    // This is heavily based on convexHulls_MultipleRandomCubes_RandomTransforms_Indexed
    // Just randomized copy-operations added

    const double edgeMargin = 0.01;

    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter_Mid360::OutItem out_Lidar;
    PointFilter::ExpressionFilter_Mid360::OutItem out_Rig;
    PointFilter::ExpressionFilter_Mid360::OutItem out_NED;
    PointFilter::ExpressionFilter_Mid360::OutItem out_InvalidHullIndexes;

    for (unsigned int i = 0; i < defaultTestRounds; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint(0x3f, -10, 10);
    }

    int insides[3] = { 0 };
    int outsides[3] = { 0 };
    int indeterminates[3] = { 0 };

    for (int cubeSet = 0; cubeSet < 10; cubeSet++)
    {
        PointFilter::ExpressionFilter_Mid360 filter_Lidar;
        PointFilter::ExpressionFilter_Mid360 filter_Rig;
        PointFilter::ExpressionFilter_Mid360 filter_NED;
        PointFilter::ExpressionFilter_Mid360 filter_InvalidHullIndexes;

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();

        filter_Lidar.setTransform_LidarToRig(transform_LidarToRig);
        filter_Rig.setTransform_LidarToRig(transform_LidarToRig);
        filter_NED.setTransform_LidarToRig(transform_LidarToRig);
        filter_InvalidHullIndexes.setTransform_LidarToRig(transform_LidarToRig);

        filter_Lidar.setTransform_RigToNED(transform_RigToNED);
        filter_Rig.setTransform_RigToNED(transform_RigToNED);
        filter_NED.setTransform_RigToNED(transform_RigToNED);
        filter_InvalidHullIndexes.setTransform_RigToNED(transform_RigToNED);

        double hullMargins[3];
        int indexes[3];
        Eigen::AlignedBox3d hullBoxes[3];

        for (int i = 0; i < 3; i++)
        {
            hullMargins[i] = (randomGenerator.generateDouble() - 0.5) * 1.0;
            indexes[i] = randomGenerator.bounded(-7, 8);

            double x1, x2, y1, y2, z1, z2;

            do
            {
                // Randomize until the box is wide/tall/deep enough
                x1 = (randomGenerator.generateDouble() - 0.5) * 20;
                x2 = (randomGenerator.generateDouble() - 0.5) * 20;
                y1 = (randomGenerator.generateDouble() - 0.5) * 20;
                y2 = (randomGenerator.generateDouble() - 0.5) * 20;
                z1 = (randomGenerator.generateDouble() - 0.5) * 20;
                z2 = (randomGenerator.generateDouble() - 0.5) * 20;
            } while ((fabs(x2 - x1) < 0.5) || (fabs(y2 - y1) < 0.5) || (fabs(z2 - z1) < 0.5));

            hullBoxes[i] = Eigen::AlignedBox3d(Eigen::Vector3d(std::min(x1, x2), std::min(y1, y2), std::min(z1, z2)), Eigen::Vector3d(std::max(x1, x2), std::max(y1, y2), std::max(z1, z2)));
        }

        ConvexHull::Filter chFilter_Lidar_First;
        QVERIFY(getConvexHullBox(hullBoxes[0]).getFilter(chFilter_Lidar_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_First { .Name = "first", .filter = chFilter_Lidar_First };

        ConvexHull::Filter chFilter_Lidar_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1]).getFilter(chFilter_Lidar_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Lidar_Second { .Name = "second", .filter = chFilter_Lidar_Second };


        ConvexHull::Filter chFilter_Rig_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_LidarToRig).getFilter(chFilter_Rig_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_First { .Name = "first_rig", .filter = chFilter_Rig_First };

        ConvexHull::Filter chFilter_Rig_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_LidarToRig).getFilter(chFilter_Rig_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_Rig_Second { .Name = "second_rig", .filter = chFilter_Rig_Second };


        ConvexHull::Filter chFilter_NED_First;
        QVERIFY(getConvexHullBox(hullBoxes[0], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_First));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_First { .Name = "first_ned", .filter = chFilter_NED_First };

        ConvexHull::Filter chFilter_NED_Second;
        QVERIFY(getConvexHullBox(hullBoxes[1], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Second));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Second { .Name = "second_ned", .filter = chFilter_NED_Second };

        ConvexHull::Filter chFilter_NED_Third;
        QVERIFY(getConvexHullBox(hullBoxes[2], transform_RigToNED * transform_LidarToRig).getFilter(chFilter_NED_Third));
        PointFilter::ExpressionFilter_Mid360::ConvexHullFilter chullFilter_Expr_NED_Third { .Name = "third_ned", .filter = chFilter_NED_Third };

        QVector<PointFilter::ExpressionFilter_Mid360::ConvexHullFilter> convexHullFilters;

        convexHullFilters.push_back(chullFilter_Expr_Lidar_First);
        convexHullFilters.push_back(chullFilter_Expr_Lidar_Second);
        convexHullFilters.push_back(chullFilter_Expr_Rig_First);
        convexHullFilters.push_back(chullFilter_Expr_Rig_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_First);
        convexHullFilters.push_back(chullFilter_Expr_NED_Second);
        convexHullFilters.push_back(chullFilter_Expr_NED_Third);

        QVERIFY(filter_Lidar.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_Rig.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_NED.setConvexHullFilters(convexHullFilters));
        QVERIFY(filter_InvalidHullIndexes.setConvexHullFilters(convexHullFilters));

        filter_Lidar.setExpression_Filter(QString("lidar.in_convex_hull_indexed(chull_first, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) + ")");

        filter_Rig.setExpression_Filter(QString("rig.in_convex_hull_indexed(chull_first_rig, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) +
                                                ") || rig.in_convex_hull_indexed(chull_second_rig, " + QString::number(hullMargins[1], 'g', 14) + ", " + QString::number(indexes[1]) + ")");

        filter_NED.setExpression_Filter(QString("ned.in_convex_hull_indexed(chull_first_ned, ") + QString::number(hullMargins[0], 'g', 14) + ", " + QString::number(indexes[0]) +
                                                ") || ned.in_convex_hull_indexed(chull_second_ned, " + QString::number(hullMargins[1], 'g', 14) + ", " + QString::number(indexes[1]) +
                                                ") || ned.in_convex_hull_indexed(chull_third_ned, " + QString::number(hullMargins[2], 'g', 14) + ", " + QString::number(indexes[2]) + ")");

        filter_InvalidHullIndexes.setExpression_Filter(
            "lidar.in_convex_hull_indexed(chull_first - 1, 0, 0) || "
            "lidar.in_convex_hull_indexed(-1, 0, -65) || "
            "rig.in_convex_hull_indexed(7, 0, 9) || "
            "rig.in_convex_hull_indexed(8, 0, 42) || "
            "ned.in_convex_hull_indexed(42, 0, 1337) || "
            "ned.in_convex_hull_indexed(1337, 0, 0xabba) || "
            "ned.in_convex_hull_indexed(chull_third_ned + 1, 0, 12345678)"
            );

        unsigned int index = 0;

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
            filter_InvalidHullIndexes.addPoint(sourcePoints[index], index);
        }

        // Reducing rounds here, since copying filters take a long time
        // (Not gonna optimize, since in real use they are rarely copied).
        for (; index < defaultTestRounds / 10; index++)
        {
            // Copy things around randomly
            int copyRand = randomGenerator.generate() % 10;

            switch (copyRand)
            {
            case 0:
            {
                PointFilter::ExpressionFilter_Mid360 newFilter(filter_Lidar);
                filter_Lidar = newFilter;
                break;
            }
            case 1:
            {
                PointFilter::ExpressionFilter_Mid360 newFilter(filter_Rig);
                filter_Rig = newFilter;
                break;
            }
            case 2:
            {
                PointFilter::ExpressionFilter_Mid360 newFilter(filter_NED);
                filter_NED = newFilter;
                break;
            }
            case 3:
            {
                PointFilter::ExpressionFilter_Mid360 newFilter(filter_InvalidHullIndexes);
                filter_InvalidHullIndexes = newFilter;
                break;
            }
            case 4:
            {
                PointFilter::ExpressionFilter_Mid360 swapStorage = filter_Lidar;
                filter_Lidar = filter_NED;
                filter_NED = swapStorage;
                swapStorage = filter_NED;
                filter_NED = filter_Lidar;
                filter_Lidar = swapStorage;
                break;
            }

            } // switch

            filter_Lidar.addPoint(sourcePoints[index], index);
            filter_Rig.addPoint(sourcePoints[index], index);
            filter_NED.addPoint(sourcePoints[index], index);
            filter_InvalidHullIndexes.addPoint(sourcePoints[index], index);

            QCOMPARE(filter_Lidar.getFilteredPoint(out_Lidar), true);
            QCOMPARE(filter_Rig.getFilteredPoint(out_Rig), true);
            QCOMPARE(filter_NED.getFilteredPoint(out_NED), true);
            QCOMPARE(filter_InvalidHullIndexes.getFilteredPoint(out_InvalidHullIndexes), true);

            bool shouldBeInside[3] = { 0 };
            bool shouldBeOutside[3] = { 0 }; (void) shouldBeOutside;    // For debugging
            bool indeterminate[3] = { 0 };

            for (int i = 0; i < 3; i++)
            {
                LivoxMid360::PointCloudData::Point sourcePoint = sourcePoints[index - (filterBufferLength / 2) + indexes[i]];

                if ((sourcePoint.x > hullBoxes[i].min().x() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.x < hullBoxes[i].max().x() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.y > hullBoxes[i].min().y() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.y < hullBoxes[i].max().y() + hullMargins[i] - edgeMargin) &&
                    (sourcePoint.z > hullBoxes[i].min().z() - hullMargins[i] + edgeMargin) &&
                    (sourcePoint.z < hullBoxes[i].max().z() + hullMargins[i] - edgeMargin))
                {
                    shouldBeInside[i] = true;
                    insides[i]++;
                }
                else if (
                    (sourcePoint.x < hullBoxes[i].min().x() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.x > hullBoxes[i].max().x() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.y < hullBoxes[i].min().y() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.y > hullBoxes[i].max().y() + hullMargins[i] + edgeMargin) ||
                    (sourcePoint.z < hullBoxes[i].min().z() - hullMargins[i] - edgeMargin) ||
                    (sourcePoint.z > hullBoxes[i].max().z() + hullMargins[i] + edgeMargin))
                {
                    shouldBeOutside[i] = true;
                    outsides[i]++;
                }
                else
                {
                    indeterminate[i] = true;
                    indeterminates[i]++;
                }
            }

            if (!indeterminate[0])
            {
                QCOMPARE(out_Lidar.filterResult, shouldBeInside[0]);
            }

            if (!indeterminate[0] && !indeterminate[1])
            {
                QCOMPARE(out_Rig.filterResult, shouldBeInside[0] || shouldBeInside[1]);
            }

            if (!indeterminate[0] && !indeterminate[1] && !indeterminate[2])
            {
                QCOMPARE(out_NED.filterResult, shouldBeInside[0] || shouldBeInside[1] || shouldBeInside[2]);
            }

            QVERIFY(out_InvalidHullIndexes.filterResult == 0);
        }
    }

    //    int foo = 0; // Debug-trap
}

void TestExpressionFilter::copyingFilters_RPLidar()
{
    // This is heavily based on rigAndNEDCoords_Indexed_RandomTransforms_RPLidar
    // Just randomized copy-operations added

    for (int rigOffset = -((filterBufferLength / 2) - 1); rigOffset < int(filterBufferLength / 2); rigOffset++)
    {
        int nedOffset = -rigOffset;

        RPLidarThread::DistanceItem sourceItems[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordX;
        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordY;
        PointFilter::ExpressionFilter_RPLidar filter_Rig_CoordZ;

        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_X;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Y;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_Rig_Z;

        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordX;
        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordY;
        PointFilter::ExpressionFilter_RPLidar filter_NED_CoordZ;

        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_X;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Y;
        PointFilter::ExpressionFilter_RPLidar::OutItem out_NED_Z;

        filter_Rig_CoordX.setExpression_Filter(QString("rig.coord_indexed.x(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordY.setExpression_Filter(QString("rig.coord_indexed.y(" + QString::number(rigOffset) + ")"));
        filter_Rig_CoordZ.setExpression_Filter(QString("rig.coord_indexed.z(" + QString::number(rigOffset) + ")"));

        filter_NED_CoordX.setExpression_Filter(QString("ned.coord_indexed.x(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordY.setExpression_Filter(QString("ned.coord_indexed.y(" + QString::number(nedOffset) + ")"));
        filter_NED_CoordZ.setExpression_Filter(QString("ned.coord_indexed.z(" + QString::number(nedOffset) + ")"));

        Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = getRandomTransform();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_LidarToRig = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        //    Eigen::Transform<double, 3, Eigen::Affine> transform_RigToNED = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

        unsigned int transformChanges_LidarToRig = 0;
        unsigned int transformChanges_RigToNED = 0;

        unsigned int index = 0;

        for (unsigned int i = 0; i < defaultTestRounds; i++)
        {
            sourceItems[i] = getRandomRPLidarDistanceItem();
            transforms_LidarToRig[i] = transform_LidarToRig;
            transforms_RigToNED[i] = transform_RigToNED;

            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_LidarToRig = getRandomTransform();
                transformChanges_LidarToRig++;
            }
            if ((randomGenerator.generate() % 20) == 0)
            {
                transform_RigToNED = getRandomTransform();
                transformChanges_RigToNED++;
            }
        }

        Q_ASSERT(transformChanges_LidarToRig > 3);
        Q_ASSERT(transformChanges_RigToNED > 3);
        Q_ASSERT(transformChanges_LidarToRig < defaultTestRounds - 10);
        Q_ASSERT(transformChanges_RigToNED < defaultTestRounds - 10);

        // Prefill buffers
        for (index = 0; index < filterBufferLength - 1; index++)
        {
            if ((index == 0) || (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index]))))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if ((index == 0) || (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index]))))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourceItems[index], index);
            filter_Rig_CoordY.addPoint(sourceItems[index], index);
            filter_Rig_CoordZ.addPoint(sourceItems[index], index);

            filter_NED_CoordX.addPoint(sourceItems[index], index);
            filter_NED_CoordY.addPoint(sourceItems[index], index);
            filter_NED_CoordZ.addPoint(sourceItems[index], index);
        }

        // Reducing rounds here, since copying filters take a long time
        // (Not gonna optimize, since in real use they are rarely copied).
        for (; index < defaultTestRounds / 10; index++)
        {
            // Copy things around randomly
            int copyRand = randomGenerator.generate() % 10;

            switch (copyRand)
            {
            case 0:
            {
                PointFilter::ExpressionFilter_RPLidar newFilter(filter_Rig_CoordX);
                filter_Rig_CoordX = newFilter;
                break;
            }
            case 1:
            {
                PointFilter::ExpressionFilter_RPLidar newFilter(filter_NED_CoordY);
                filter_NED_CoordY = newFilter;
                break;
            }
            case 2:
            {
                PointFilter::ExpressionFilter_RPLidar newFilter(filter_NED_CoordZ);
                filter_NED_CoordZ = newFilter;
                break;
            }
            case 3:
            {
                PointFilter::ExpressionFilter_RPLidar swapStorage = filter_Rig_CoordX;
                filter_Rig_CoordX = filter_NED_CoordZ;
                filter_NED_CoordZ = swapStorage;
                swapStorage = filter_NED_CoordZ;
                filter_NED_CoordZ = filter_Rig_CoordX;
                filter_Rig_CoordX = swapStorage;
                break;
            }

            } // switch

            if (!(transforms_LidarToRig[index - 1].isApprox(transforms_LidarToRig[index])))
            {
                filter_Rig_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_Rig_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);

                filter_NED_CoordX.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordY.setTransform_LidarToRig(transforms_LidarToRig[index]);
                filter_NED_CoordZ.setTransform_LidarToRig(transforms_LidarToRig[index]);
            }

            if (!(transforms_RigToNED[index - 1].isApprox(transforms_RigToNED[index])))
            {
                filter_Rig_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_Rig_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);

                filter_NED_CoordX.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordY.setTransform_RigToNED(transforms_RigToNED[index]);
                filter_NED_CoordZ.setTransform_RigToNED(transforms_RigToNED[index]);
            }

            filter_Rig_CoordX.addPoint(sourceItems[index], index);
            filter_Rig_CoordY.addPoint(sourceItems[index], index);
            filter_Rig_CoordZ.addPoint(sourceItems[index], index);

            filter_NED_CoordX.addPoint(sourceItems[index], index);
            filter_NED_CoordY.addPoint(sourceItems[index], index);
            filter_NED_CoordZ.addPoint(sourceItems[index], index);

            QCOMPARE(filter_Rig_CoordX.getFilteredPoint(out_Rig_X), true);
            QCOMPARE(filter_Rig_CoordY.getFilteredPoint(out_Rig_Y), true);
            QCOMPARE(filter_Rig_CoordZ.getFilteredPoint(out_Rig_Z), true);

            QCOMPARE(filter_NED_CoordX.getFilteredPoint(out_NED_X), true);
            QCOMPARE(filter_NED_CoordY.getFilteredPoint(out_NED_Y), true);
            QCOMPARE(filter_NED_CoordZ.getFilteredPoint(out_NED_Z), true);

            int rigOffsettedIndex = index - (filterBufferLength / 2) + rigOffset;
            int nedOffsettedIndex = index - (filterBufferLength / 2) + nedOffset;

            RPLidarThread::DistanceItem rigSourceItem = sourceItems[rigOffsettedIndex];
            RPLidarThread::DistanceItem nedSourceItem = sourceItems[nedOffsettedIndex];

            Eigen::Vector3d rigSourceVector(sin(rigSourceItem.angle) * rigSourceItem.distance, cos(rigSourceItem.angle) * rigSourceItem.distance, 0.0);
            Eigen::Vector3d nedSourceVector(sin(nedSourceItem.angle) * nedSourceItem.distance, cos(nedSourceItem.angle) * nedSourceItem.distance, 0.0);

            Eigen::Vector3d rigVector(out_Rig_X.filterResult, out_Rig_Y.filterResult, out_Rig_Z.filterResult);
            Eigen::Vector3d nedVector(out_NED_X.filterResult, out_NED_Y.filterResult, out_NED_Z.filterResult);

            QVERIFY(compareVectors(rigVector, transforms_LidarToRig[rigOffsettedIndex] * rigSourceVector));
            QVERIFY(compareVectors(nedVector, transforms_RigToNED[nedOffsettedIndex] * (transforms_LidarToRig[nedOffsettedIndex] * nedSourceVector)));

            RPLidarThread::DistanceItem lidarItem = sourceItems[index - (filterBufferLength / 2)];
            Eigen::Vector3d lidarVector(sin(lidarItem.angle) * lidarItem.distance, cos(lidarItem.angle) * lidarItem.distance, 0);

            Eigen::Vector3d expectedNEDOutVector = transforms_RigToNED[index - (filterBufferLength / 2)] * (transforms_LidarToRig[index - (filterBufferLength / 2)] * lidarVector);

            QVERIFY(compareVectors(out_NED_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_NED_Z.coords, expectedNEDOutVector));

            QVERIFY(compareVectors(out_Rig_X.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Y.coords, expectedNEDOutVector));
            QVERIFY(compareVectors(out_Rig_Z.coords, expectedNEDOutVector));
        }
    }
}
