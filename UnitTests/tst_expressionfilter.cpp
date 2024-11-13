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

PointFilter::ExpressionFilter::OutItem TestExpressionFilter::getRandomOutItem(void)
{
    PointFilter::ExpressionFilter::OutItem item;
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

Eigen::Transform<double, 3, Eigen::Affine> TestExpressionFilter::getRandomTransform(double translateLowLimit, double translateHighLimit)
{
    // Doesn't return very evenly distributed transforms, but should suffice in this context.

    Eigen::AngleAxisd orientation(randomGenerator.generateDouble() * (2 * M_PI), getRandomVec(-1.0, 1.0).normalized());
    Eigen::Transform<double, 3, Eigen::Affine> ret;
    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, getRandomVec(-10.0, 10.0));

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
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

    QCOMPARE(filter.setExpression_Filter("1"), true);
    QCOMPARE(filter.setExpression_Quality("1"), true);
}

void TestExpressionFilter::expressionValidity_InvalidExpressions()
{
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

    QCOMPARE(filter.setExpression_Filter("invalid"), false);
    QCOMPARE(filter.setExpression_Quality("invalid"), false);

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

    QCOMPARE(filter.setExpression_Filter("1"), true);
    QCOMPARE(filter.setExpression_Quality("invalid"), false);

    for (index = 0; index < defaultTestRounds; index++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), index);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QVERIFY(std::isnan(out.quality));
    }

    QString errorString;
    int errorPosition;

    QCOMPARE(filter.setExpression_Filter("#", &errorString, &errorPosition), false);
    QCOMPARE(errorPosition, 0);

    QCOMPARE(filter.setExpression_Quality("#", &errorString, &errorPosition), false);
    QCOMPARE(errorPosition, 0);

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

void TestExpressionFilter::noData()
{
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

    for (int i = 0; i < 100; i++)
    {
        out = getRandomOutItem();
        QCOMPARE(filter.getFilteredPoint(out), false);
        QCOMPARE(out.valid, false);
    }
}

void TestExpressionFilter::defaultExpressions()
{
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

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

void TestExpressionFilter::pureFunctions()
{
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

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

        QCOMPARE(filter.setExpression_Filter(expression.str), true);
        QCOMPARE(filter.setExpression_Quality("1"), true);

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

        QCOMPARE(filter.setExpression_Filter("1"), true); // Quality will only be calculated if filter returns true (1)
        QCOMPARE(filter.setExpression_Quality(expression.str), true);

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

void TestExpressionFilter::lidarCoords()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter filter_CoordX;
    PointFilter::ExpressionFilter filter_CoordY;
    PointFilter::ExpressionFilter filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

void TestExpressionFilter::lidarCoords_Indexed()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter filter_CoordX_Index0;
    PointFilter::ExpressionFilter filter_CoordY_Index0;
    PointFilter::ExpressionFilter filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter filter_Distance;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

void TestExpressionFilter::lidarDistance_Indexed()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter filter_Distance_Index0;
    PointFilter::ExpressionFilter filter_Distance_IndexMinus7;
    PointFilter::ExpressionFilter filter_Distance_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

void TestExpressionFilter::lidarProperties()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter filter_Properties;
    PointFilter::ExpressionFilter filter_Properties_other;
    PointFilter::ExpressionFilter filter_Properties_dust;
    PointFilter::ExpressionFilter filter_Properties_glue;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out_Properties;
    PointFilter::ExpressionFilter::OutItem out_Properties_other;
    PointFilter::ExpressionFilter::OutItem out_Properties_dust;
    PointFilter::ExpressionFilter::OutItem out_Properties_glue;

    filter_Properties.setExpression_Filter("lidar.properties");
    filter_Properties_other.setExpression_Filter("lidar.properties.other");
    filter_Properties_dust.setExpression_Filter("lidar.properties.dust");
    filter_Properties_glue.setExpression_Filter("lidar.properties.glue");

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

        PointFilter::ExpressionFilter filter_Properties;
        PointFilter::ExpressionFilter filter_Properties_other;
        PointFilter::ExpressionFilter filter_Properties_dust;
        PointFilter::ExpressionFilter filter_Properties_glue;

        unsigned int index = 0;
        PointFilter::ExpressionFilter::OutItem out_Properties;
        PointFilter::ExpressionFilter::OutItem out_Properties_other;
        PointFilter::ExpressionFilter::OutItem out_Properties_dust;
        PointFilter::ExpressionFilter::OutItem out_Properties_glue;

        filter_Properties.setExpression_Filter(QString("lidar.properties_indexed(") + QString::number(offset) + ")");
        filter_Properties_other.setExpression_Filter(QString("lidar.properties_indexed.other(") + QString::number(offset) + ")");
        filter_Properties_dust.setExpression_Filter(QString("lidar.properties_indexed.dust(") + QString::number(offset) + ")");
        filter_Properties_glue.setExpression_Filter(QString("lidar.properties_indexed.glue(") + QString::number(offset) + ")");

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

    PointFilter::ExpressionFilter filter_Reflectivity;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out_Reflectivity;

    filter_Reflectivity.setExpression_Filter("lidar.reflectivity");

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

        PointFilter::ExpressionFilter filter_Reflectivity;

        unsigned int index = 0;
        PointFilter::ExpressionFilter::OutItem out_Reflectivity;

        filter_Reflectivity.setExpression_Filter(QString("lidar.reflectivity_indexed(") + QString::number(offset) + ")");

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

void TestExpressionFilter::rigCoords_DefaultTransform()
{
    LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];

    PointFilter::ExpressionFilter filter_CoordX;
    PointFilter::ExpressionFilter filter_CoordY;
    PointFilter::ExpressionFilter filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter filter_CoordX_Index0;
    PointFilter::ExpressionFilter filter_CoordY_Index0;
    PointFilter::ExpressionFilter filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter filter_CoordX;
    PointFilter::ExpressionFilter filter_CoordY;
    PointFilter::ExpressionFilter filter_CoordZ;
    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter filter_CoordX_Index0;
    PointFilter::ExpressionFilter filter_CoordY_Index0;
    PointFilter::ExpressionFilter filter_CoordZ_Index0;

    // Use just some random indexes
    PointFilter::ExpressionFilter filter_CoordX_IndexMinus1;
    PointFilter::ExpressionFilter filter_CoordX_IndexPlus2;

    PointFilter::ExpressionFilter filter_CoordY_IndexMinus2;
    PointFilter::ExpressionFilter filter_CoordY_IndexPlus1;

    // Min/max indexes for z
    PointFilter::ExpressionFilter filter_CoordZ_IndexMinus7;
    PointFilter::ExpressionFilter filter_CoordZ_IndexPlus7;

    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter filter_Rig_CoordX;
    PointFilter::ExpressionFilter filter_Rig_CoordY;
    PointFilter::ExpressionFilter filter_Rig_CoordZ;

    PointFilter::ExpressionFilter::OutItem out_Rig_X;
    PointFilter::ExpressionFilter::OutItem out_Rig_Y;
    PointFilter::ExpressionFilter::OutItem out_Rig_Z;

    PointFilter::ExpressionFilter filter_NED_CoordX;
    PointFilter::ExpressionFilter filter_NED_CoordY;
    PointFilter::ExpressionFilter filter_NED_CoordZ;

    PointFilter::ExpressionFilter::OutItem out_NED_X;
    PointFilter::ExpressionFilter::OutItem out_NED_Y;
    PointFilter::ExpressionFilter::OutItem out_NED_Z;

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

void TestExpressionFilter::rigAndNEDCoords_Indexed_RandomTransforms()
{
    for (int rigOffset = -((filterBufferLength / 2) - 1); rigOffset < int(filterBufferLength / 2); rigOffset++)
    {
        int nedOffset = -rigOffset;

        LivoxMid360::PointCloudData::Point sourcePoints[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_LidarToRig[defaultTestRounds];
        Eigen::Transform<double, 3, Eigen::Affine> transforms_RigToNED[defaultTestRounds];

        PointFilter::ExpressionFilter filter_Rig_CoordX;
        PointFilter::ExpressionFilter filter_Rig_CoordY;
        PointFilter::ExpressionFilter filter_Rig_CoordZ;

        PointFilter::ExpressionFilter::OutItem out_Rig_X;
        PointFilter::ExpressionFilter::OutItem out_Rig_Y;
        PointFilter::ExpressionFilter::OutItem out_Rig_Z;

        PointFilter::ExpressionFilter filter_NED_CoordX;
        PointFilter::ExpressionFilter filter_NED_CoordY;
        PointFilter::ExpressionFilter filter_NED_CoordZ;

        PointFilter::ExpressionFilter::OutItem out_NED_X;
        PointFilter::ExpressionFilter::OutItem out_NED_Y;
        PointFilter::ExpressionFilter::OutItem out_NED_Z;

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

void TestExpressionFilter::convexHullIndexes()
{
    QVector<PointFilter::ExpressionFilter::ConvexHullFilter> convexHullFilters;

    PointFilter::ExpressionFilter::ConvexHullFilter firstFilter { .Name = "first", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter::ConvexHullFilter secondFilter { .Name = "second", .filter = ConvexHull::Filter() };
    PointFilter::ExpressionFilter::ConvexHullFilter thirdFilter { .Name = "tHiRd", .filter = ConvexHull::Filter() };

    convexHullFilters.push_back(firstFilter);
    convexHullFilters.push_back(secondFilter);
    convexHullFilters.push_back(thirdFilter);

    PointFilter::ExpressionFilter filter_First;
    PointFilter::ExpressionFilter filter_Second;
    PointFilter::ExpressionFilter filter_Third;
    PointFilter::ExpressionFilter filter_InvalidHullIndexIdent;

    QVERIFY(filter_First.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_Second.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_Third.setConvexHullFilters(convexHullFilters));
    QVERIFY(filter_InvalidHullIndexIdent.setConvexHullFilters(convexHullFilters));

    // Calling this again should not change anything
    QVERIFY(filter_Third.setConvexHullFilters(convexHullFilters));

    PointFilter::ExpressionFilter::OutItem out_First;
    PointFilter::ExpressionFilter::OutItem out_Second;
    PointFilter::ExpressionFilter::OutItem out_Third;

    QCOMPARE(filter_First.setExpression_Filter("chull_first"), true);
    QCOMPARE(filter_Second.setExpression_Filter("chull_sEcOnD"), true);
    QCOMPARE(filter_Third.setExpression_Filter("chull_third"), true);
    QCOMPARE(filter_InvalidHullIndexIdent.setExpression_Filter("chull_InValid"), false);

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

    Eigen::Vector3d transformedCorner1 = transform * corner1;
    Eigen::Vector3d transformedCorner2 = transform * corner2;

    hull.addPoint(transformedCorner1);
    hull.addPoint(Eigen::Vector3d(transformedCorner1.x(), transformedCorner1.y(), transformedCorner2.z()));
    hull.addPoint(Eigen::Vector3d(transformedCorner1.x(), transformedCorner2.y(), transformedCorner1.z()));
    hull.addPoint(Eigen::Vector3d(transformedCorner1.x(), transformedCorner2.y(), transformedCorner2.z()));
    hull.addPoint(Eigen::Vector3d(transformedCorner2.x(), transformedCorner1.y(), transformedCorner1.z()));
    hull.addPoint(Eigen::Vector3d(transformedCorner2.x(), transformedCorner1.y(), transformedCorner2.z()));
    hull.addPoint(Eigen::Vector3d(transformedCorner2.x(), transformedCorner2.y(), transformedCorner1.z()));
    hull.addPoint(transformedCorner2);

    return hull;
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

    PointFilter::ExpressionFilter::ConvexHullFilter hugeOriginBoxCHullFilter { .Name = "hugeoriginbox", .filter = cHullFilter };

    PointFilter::ExpressionFilter exprFilter_Lidar;
    PointFilter::ExpressionFilter exprFilter_Rig;
    PointFilter::ExpressionFilter exprFilter_NED;

    QVector<PointFilter::ExpressionFilter::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(hugeOriginBoxCHullFilter);
    QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));

    // Use invalid hull indexes so "in_convex_hull"-functions should always return false (0)
    // "hugeoriginbox" would be index 0 so that's skipped here
    QVERIFY(exprFilter_Lidar.setExpression_Filter("lidar.in_convex_hull(-1, 0)"));
    QVERIFY(exprFilter_Rig.setExpression_Filter("rig.in_convex_hull(1, 0)"));
    QVERIFY(exprFilter_NED.setExpression_Filter("NED.In_Convex_Hull(2, 0)"));

    unsigned int index;

    // Prefill buffers
    for (index = 0; index < filterBufferLength - 1; index++)
    {
        exprFilter_Lidar.addPoint(sourcePoints[index], index);
        exprFilter_Rig.addPoint(sourcePoints[index], index);
        exprFilter_NED.addPoint(sourcePoints[index], index);
    }

    PointFilter::ExpressionFilter::OutItem out;

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

    PointFilter::ExpressionFilter::ConvexHullFilter originBoxCHullFilter { .Name = "originbox", .filter = cHullFilter };

    PointFilter::ExpressionFilter exprFilter_Lidar;
    PointFilter::ExpressionFilter exprFilter_Rig;
    PointFilter::ExpressionFilter exprFilter_NED;

    QVector<PointFilter::ExpressionFilter::ConvexHullFilter> convexHullFilters;
    convexHullFilters.push_back(originBoxCHullFilter);
    QVERIFY(exprFilter_Lidar.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_Rig.setConvexHullFilters(convexHullFilters));
    QVERIFY(exprFilter_NED.setConvexHullFilters(convexHullFilters));

    QVERIFY(exprFilter_Lidar.setExpression_Filter("lidar.in_convex_hull(chull_originbox, 0)"));
    QVERIFY(exprFilter_Rig.setExpression_Filter("rig.in_convex_hull(chull_originbox, 0)"));
    QVERIFY(exprFilter_NED.setExpression_Filter("NED.In_Convex_Hull(CHULL_OriginBox, 0)"));

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

    PointFilter::ExpressionFilter::OutItem out;

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

















