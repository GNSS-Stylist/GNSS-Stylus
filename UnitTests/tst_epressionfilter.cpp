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

LivoxMid360::PointCloudData::Point TestExpressionFilter::getRandomLidarSourcePoint(const quint8 propertyMask)
{
    LivoxMid360::PointCloudData::Point point;

    const double pointCoordLowLimit = 0.01;
    const double pointCoordHighLimit = 40.0;

    point.x = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;
    point.y = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;
    point.z = randomGenerator.generateDouble() * (pointCoordHighLimit - pointCoordLowLimit) + pointCoordLowLimit;

    point.reflectivity = randomGenerator.generate();    // TODO: Check range!
    point.properties = randomGenerator.generate() & propertyMask;

    return point;
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
        QCOMPARE(out.uptime_ms, i + filterBufferLength / 2);
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















