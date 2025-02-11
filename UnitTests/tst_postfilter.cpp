/*
    tst_postfilter.cpp (part of GNSS-Stylus)
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

#include "tst_postfilter.h"
#include "PostProcessing/Lidar/PointFilter/postfilter.h"

TestPostFilter::TestPostFilter()
{
}

TestPostFilter::~TestPostFilter()
{
}

void TestPostFilter::initTestCase()
{
}

void TestPostFilter::cleanupTestCase()
{
}

void TestPostFilter::inits()
{
    PointFilter::PostFilter::Params params;

    PointFilter::PostFilter defaultFilter(params);

    Eigen::Vector3d zeroVec = Eigen::Vector3d::Zero();
    qint64 time = 0;

    for (int i = 0; i < 10; i++)
    {
        QVERIFY(defaultFilter.filter(zeroVec, time));
    }

    params.minDistDiff = 1;

    PointFilter::PostFilter distFilter(params);

    // First point should always return true
    QVERIFY(distFilter.filter(zeroVec, time));

    // As all limits are off, subsequent identical points should return true

    for (int i = 0; i < 10; i++)
    {
        QVERIFY(distFilter.filter(zeroVec, time));
    }

    // reinit should start everything from the start again
    distFilter.reinit();

    // First point should always return true
    QVERIFY(distFilter.filter(zeroVec, time));

    // As all limits are off, subsequent identical points should return true

    for (int i = 0; i < 10; i++)
    {
        QVERIFY(distFilter.filter(zeroVec, time));
    }

    // Swtich to time diff test
    params.minDistDiff = 0;
    params.minTimeDiff = 1;

    // reinit should start everything from the beginning again
    distFilter.reinit();

    // First point should always return true
    QVERIFY(distFilter.filter(zeroVec, time));

    // As all limits are off, subsequent identical points should return true

    for (int i = 0; i < 10; i++)
    {
        QVERIFY(distFilter.filter(zeroVec, time));
    }
}

void TestPostFilter::gradualDistanceChange()
{
    PointFilter::PostFilter::Params params;

    params.minDistDiff = 1;
    params.minTimeDiff = 10000;

    PointFilter::PostFilter filter(params);
    qint64 time = 0;

    Eigen::Vector3d point(1,2,3);
    Eigen::Vector3d oldPoint = point;
    qint64 oldTime = time;

    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Limit shouldn't be reached below 1 m distance between points
    for (int i = 0; i < 7; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.x() += 0.125;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    // Now the distance (1 m) should be reached (note that adding 0.125 to a small value doesn't cause any rounding errors)
    oldPoint = point;
    oldTime = time;
    point.x() += 0.125;
    time++;
    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Again limit shouldn't be reached below 1 m distance between points
    for (int i = 0; i < 7; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.y() += 0.125;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    // Now the distance (1 m) should be reached again
    oldPoint = point;
    oldTime = time;
    point.y() += 0.125;
    time++;
    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Just throwing in some more points...
    for (int i = 0; i < 100; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.z() += 0.125;
        time += 10;
        if ((i % 8) == 7)
        {
            QVERIFY(filter.filter(point, time));
        }
        else
        {
            QVERIFY(!filter.filter(point, time));
        }

        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }
}

void TestPostFilter::gradualTimeChange()
{
    PointFilter::PostFilter::Params params;

    params.minDistDiff = 1;
    params.minTimeDiff = 10;

    PointFilter::PostFilter filter(params);
    qint64 time = 0;

    Eigen::Vector3d point(1,2,3);
    Eigen::Vector3d oldPoint = point;
    qint64 oldTime = time;

    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Limit shouldn't be reached below 10 units time difference between points
    for (int i = 0; i < 9; i++)
    {
        oldPoint = point;
        oldTime = time;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    // Now the time limit (10 units) should be reached
    oldPoint = point;
    oldTime = time;
    time++;
    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Again time limit shouldn't be reached below 10 units between points
    for (int i = 0; i < 9; i++)
    {
        oldPoint = point;
        oldTime = time;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    // Now the time limit (10 units) should be reached again
    oldPoint = point;
    oldTime = time;
    time++;
    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Just throwing in some more points...
    for (int i = 0; i < 100; i++)
    {
        oldPoint = point;
        oldTime = time;
        time++;
        if ((i % 10) == 9)
        {
            QVERIFY(filter.filter(point, time));
        }
        else
        {
            QVERIFY(!filter.filter(point, time));
        }

        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }
}

void TestPostFilter::distanceJumps()
{
    PointFilter::PostFilter::Params params;

    params.minDistDiff = 10;
    params.minTimeDiff = 10000;

    PointFilter::PostFilter filter(params);
    qint64 time = 0;

    Eigen::Vector3d point(1,2,3);
    Eigen::Vector3d oldPoint = point;
    qint64 oldTime = time;

    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Limit shouldn't be reached below 10 m distance between points
    for (int i = 0; i < 20; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.x() += 0.125;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    oldPoint = point;
    oldTime = time;
    point.x() += 9;
    time++;
    // Now the distance (10 m) should be reached
    QVERIFY(filter.filter(point, time));

    // As the last jump was below 10 m, checkBack should return false
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Again limit shouldn't be reached below 10 m distance between points
    for (int i = 0; i < 30; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.y() -= 0.125;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    oldPoint = point;
    oldTime = time;
    point.y() -= 20;
    // Now the distance (10 m) should be reached again
    QVERIFY(filter.filter(point, time));

    // As the last jump was above 10 m checkBack should also return true
    QVERIFY(filter.checkBack(oldPoint, oldTime));

    // Just throwing in some more points jumping far enough...
    for (int i = 0; i < 100; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.z() += 10 + i;
        time += 3;
        QVERIFY(filter.filter(point, time));
        QVERIFY(filter.checkBack(oldPoint, oldTime));
    }
}

void TestPostFilter::timeJumps()
{
    PointFilter::PostFilter::Params params;

    params.minDistDiff = 10000;
    params.minTimeDiff = 100;

    PointFilter::PostFilter filter(params);
    qint64 time = 0;

    Eigen::Vector3d point(1,2,3);
    Eigen::Vector3d oldPoint = point;
    qint64 oldTime = time;

    QVERIFY(filter.filter(point, time));
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Limit shouldn't be reached below 100 units time difference between points
    for (int i = 0; i < 20; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.x() += 0.125;
        time++;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    oldPoint = point;
    oldTime = time;
    point.x() += 9;
    time += 80;
    // Now the distance (100 units) should be reached
    QVERIFY(filter.filter(point, time));

    // As the last time jump was below 100 units, checkBack should return false
    QVERIFY(!filter.checkBack(oldPoint, oldTime));

    // Again limit shouldn't be reached below 100 units time difference between points
    for (int i = 0; i < 30; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.y() -= 0.125;
        time += 2;
        QVERIFY(!filter.filter(point, time));
        QVERIFY(!filter.checkBack(oldPoint, oldTime));
    }

    oldPoint = point;
    oldTime = time;
    point.y() -= 20;
    time += 100;
    // Now the time difference (100 units) should be reached again
    QVERIFY(filter.filter(point, time));

    // As the last jump was at least 100 units checkBack should also return true
    QVERIFY(filter.checkBack(oldPoint, oldTime));

    // Just throwing in some more points jumping far enough (in time)...
    for (int i = 0; i < 1000; i++)
    {
        oldPoint = point;
        oldTime = time;
        point.z() += 10;
        time += 100 + i;
        QVERIFY(filter.filter(point, time));
        QVERIFY(filter.checkBack(oldPoint, oldTime));
    }
}
