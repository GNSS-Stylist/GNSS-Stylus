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

    for (unsigned int i = 0; i < filter.bufferLength - 1; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), i);
        QCOMPARE(filter.getFilteredPoint(out), false);
    }

    for (int i = 0; i < 10000; i++)
    {
        out = getRandomOutItem();
        filter.addPoint(getRandomLidarSourcePoint(), i + filter.bufferLength - 1);
        QCOMPARE(filter.getFilteredPoint(out), true);
        QCOMPARE(out.valid, true);
        QCOMPARE(out.filterResult, 1);
        QCOMPARE(out.uptime_ms, i + 16);
        QCOMPARE(out.quality, 1);
    }
}

void TestExpressionFilter::pureFunctions()
{
    PointFilter::ExpressionFilter filter;
    PointFilter::ExpressionFilter::OutItem out;

    unsigned int index = 0;
    // Prefill buffer
    for (index = 0; index < filter.bufferLength - 1; index++)
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
    const int numOfTestValues = 1000;

    LivoxMid360::PointCloudData::Point sourcePoints[numOfTestValues];

    PointFilter::ExpressionFilter filter_coord_x;
    PointFilter::ExpressionFilter filter_coord_y;
    unsigned int index = 0;
    PointFilter::ExpressionFilter::OutItem out;

    filter_coord_x.setExpression_Filter("lidar.coord.x");
    filter_coord_y.setExpression_Filter("lidar.coord.y");

    for (int i = 0; i < numOfTestValues; i++)
    {
        sourcePoints[i] = getRandomLidarSourcePoint();
    }

    // Prefill buffer
    for (index = 0; index < filter_coord_x.bufferLength - 1; index++)
    {
        filter_coord_x.addPoint(sourcePoints[index], index);
        filter_coord_y.addPoint(sourcePoints[index], index);
        QCOMPARE(filter_coord_x.getFilteredPoint(out), false);
    }

    for (; index < numOfTestValues; index++)
    {
        filter_coord_x.addPoint(sourcePoints[index], index);
        filter_coord_y.addPoint(sourcePoints[index], index);

        QCOMPARE(filter_coord_x.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filter_coord_x.bufferLength / 2)].x);

        QCOMPARE(filter_coord_y.getFilteredPoint(out), true);
        QCOMPARE(out.filterResult, sourcePoints[index - (filter_coord_y.bufferLength / 2)].y);
    }





}
















