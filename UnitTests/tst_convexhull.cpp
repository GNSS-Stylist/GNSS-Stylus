/*
    tst_lidarfiltering.cpp (part of GNSS-Stylus)
    Copyright (C) 2020-2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "tst_convexhull.h"
#include "../RPLidar/rplidarplausibilityfilter.h"
#include "../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.h"

TestConvexHull::TestConvexHull()
{

}

TestConvexHull::~TestConvexHull()
{

}

Eigen::Vector3d TestConvexHull::getRandomVec(double lowLimit, double highLimit)
{
    return Eigen::Vector3d(randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
                           randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit
                           );
}

void TestConvexHull::initTestCase()
{
    randomGenerator.seed(1337);
}

void TestConvexHull::cleanupTestCase()
{

}

void TestConvexHull::uninitialized()
{
    ConvexHull hull;
    ConvexHull::Filter filter;
    QVERIFY(!(hull.getFilter(filter)));
    QVERIFY(!(filter.isValid()));
}

void TestConvexHull::pointCountCheck()
{
    ConvexHull hull;
    ConvexHull::Filter filter;

    for (unsigned int i = 0; i < 3; i++)
    {
        Eigen::Vector3d vecToAdd = getRandomVec();
        QVERIFY(hull.addPoint(vecToAdd));
        QVERIFY(!hull.addPoint(vecToAdd)); // Should not add a non-unique vector
        QCOMPARE(hull.getNumOfUniquePoints(), i + 1);
        QVERIFY(!(hull.getFilter(filter)));
        QVERIFY(!filter.isInside(getRandomVec()));
        QVERIFY(!(filter.isValid()));
    }

    for (unsigned int i = 0; i < 100; i++)
    {
        Eigen::Vector3d vecToAdd = getRandomVec();
        QVERIFY(hull.addPoint(vecToAdd));
        QVERIFY(!hull.addPoint(vecToAdd)); // Should not add a non-unique vector
        QCOMPARE(hull.getNumOfUniquePoints(), i + 1 + 3);
        QVERIFY((hull.getFilter(filter)));
        QVERIFY((filter.isValid()));
    }

    hull.clearPoints();

    // Test again after clearing

    for (unsigned int i = 0; i < 3; i++)
    {
        Eigen::Vector3d vecToAdd = getRandomVec();
        QVERIFY(hull.addPoint(vecToAdd));
        QVERIFY(!hull.addPoint(vecToAdd)); // Should not add a non-unique vector
        QCOMPARE(hull.getNumOfUniquePoints(), i + 1);
        QVERIFY(!(hull.getFilter(filter)));
        QVERIFY(!filter.isInside(getRandomVec()));
        QVERIFY(!(filter.isValid()));
    }

    for (unsigned int i = 0; i < 100; i++)
    {
        Eigen::Vector3d vecToAdd = getRandomVec();
        QVERIFY(hull.addPoint(vecToAdd));
        QVERIFY(!hull.addPoint(vecToAdd)); // Should not add a non-unique vector
        QCOMPARE(hull.getNumOfUniquePoints(), i + 1 + 3);
        QVERIFY((hull.getFilter(filter)));
        QVERIFY((filter.isValid()));
    }

    hull.clearPoints();

    // Test again by adding multiple points at once

    QVector<Eigen::Vector3d> vecVec;

    for (unsigned int i = 0; i < 3; i++)
    {
        vecVec.push_back(getRandomVec());
    }

    QCOMPARE(hull.addPoints(vecVec), 3U);
    QVERIFY(!(hull.getFilter(filter)));
    QVERIFY(!(filter.isValid()));
    QVERIFY(!filter.isInside(getRandomVec()));
    QCOMPARE(hull.getNumOfUniquePoints(), 3U);

    // Add one point to make sure there are enough point during the next loop
    // (Happened to me once that on the first round the number of points to add was 0...)
    QVERIFY(hull.addPoint(getRandomVec()));
    QVERIFY((hull.getFilter(filter)));
    QVERIFY((filter.isValid()));
    QCOMPARE(hull.getNumOfUniquePoints(), 4U);

    unsigned int totalPointsAdded = 4;

    for (unsigned int i = 0; i < 10; i++)
    {
        vecVec.clear();
        unsigned int numOfVecsToAdd = (randomGenerator.generate() % 10);
        for (unsigned int ii = 0; ii < numOfVecsToAdd; ii++)
        {
            // This test just assumes that every vector randomized is unique (which is very likely)
            Eigen::Vector3d vecToAdd = getRandomVec();
            vecVec.push_back(vecToAdd);
        }

        QCOMPARE(hull.addPoints(vecVec), numOfVecsToAdd);
        totalPointsAdded += numOfVecsToAdd;
        QCOMPARE(hull.addPoints(vecVec), 0U); // Should not add non-unique vectors
        QCOMPARE(hull.getNumOfUniquePoints(), totalPointsAdded);
        QVERIFY((hull.getFilter(filter)));
        QVERIFY((filter.isValid()));
    }
}

void TestConvexHull::cubeInTheOrigin()
{
    ConvexHull hull;
    ConvexHull::Filter filter;

    hull.addPoint(Eigen::Vector3d(-1, -1, -1));
    hull.addPoint(Eigen::Vector3d(-1, -1,  1));
    hull.addPoint(Eigen::Vector3d(-1,  1, -1));
    hull.addPoint(Eigen::Vector3d(-1,  1,  1));
    hull.addPoint(Eigen::Vector3d(1,  -1, -1));
    hull.addPoint(Eigen::Vector3d(1,  -1,  1));
    hull.addPoint(Eigen::Vector3d(1,   1, -1));
    hull.addPoint(Eigen::Vector3d(1,   1,  1));

    QVERIFY((hull.getFilter(filter)));
    QVERIFY((filter.isValid()));
    QCOMPARE(hull.getNumOfUniquePoints(), 8U);

    // Add random "noise" points inside the cube (should not affect the convex hull)

    for (unsigned int i = 0; i < 100; i++)
    {
        Eigen::Vector3d randomVec = getRandomVec(-1, 1);
        hull.addPoint(randomVec);
        QCOMPARE(hull.getNumOfUniquePoints(), i + 8 + 1);
    }

    QVERIFY((hull.getFilter(filter)));
    QVERIFY((filter.isValid()));

    // Test some random points that should definitely be inside the hull

    QVERIFY(filter.isInside(Eigen::Vector3d(0, 0, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0.5, 0, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0.5, 0.5, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0.5, 0.5, 0.5)));
    QVERIFY(filter.isInside(Eigen::Vector3d(-0.5, 0, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(-0.5, -0.5, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(-0.5, -0.5, -0.5)));

    // Close calls:

    QVERIFY(filter.isInside(Eigen::Vector3d(-0.999, 0, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0.999, 0, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0, -0.999, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0, 0.999, 0)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0, 0, -0.999)));
    QVERIFY(filter.isInside(Eigen::Vector3d(0, 0, 0.999)));

    // Test some random points that should definitely be outside the hull

    QVERIFY(!filter.isInside(Eigen::Vector3d(5, 0, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 5, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 0, 5)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(-5, 0, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, -5, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 0, -5)));

    // Close calls:

    QVERIFY(!filter.isInside(Eigen::Vector3d(-1.001, 0, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(1.001, 0, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, -1.001, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 1.001, 0)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 0, -1.001)));
    QVERIFY(!filter.isInside(Eigen::Vector3d(0, 0, 1.001)));

    for (int intX = -1950; intX < 2000; intX += 100)
    {
        for (int intY = -1950; intY < 2000; intY += 100)
        {
            for (int intZ = -1950; intZ < 2000; intZ += 100)
            {
                // Note: Due to some rounding errors the hull area is not exactly on -1, 1 limits
                // (which is quite interesting, as these limits are -1 & 1, which should be exactly representable with floats.
                // But not studying the details of quickhull-algorithm now).
                // This effective 0.05 unit "safe zone" is quite big, but maybe we can trust that the hull is ok enough...

                Eigen::Vector3d testPoint(intX * 0.001, intY * 0.001, intZ * 0.001);    // This actually also adds some rounding errors

                if ((intX >= -1000) && (intX <= 1000) &&
                    (intY >= -1000) && (intY <= 1000) &&
                    (intZ >= -1000) && (intZ <= 1000))
                {
                    // Should be inside the hull

                    QVERIFY(filter.isInside(testPoint));
                }
                else
                {
                    QVERIFY(!(filter.isInside(testPoint)));
                }
            }
        }
    }
}

void TestConvexHull::randomCubes()
{
    const double lowLimit_Hull = -10;
    const double highLimit_Hull = 10;

    const double lowLimit_TestArea = -10;
    const double highLimit_TestArea = 10;

    // Counters for testing the test
    int outsides = 0;
    int insides = 0;
    int indeterminates = 0;

    // Disable clang/compiler warnings (set but not used)
    (void) outsides;
    (void) insides;
    (void) indeterminates;

    for (int cube = 0; cube < 10; cube++)
    {
        ConvexHull hull;
        ConvexHull::Filter filter;

        double x1 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;
        double x2 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;
        double y1 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;
        double y2 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;
        double z1 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;
        double z2 = randomGenerator.generateDouble() * (highLimit_Hull - lowLimit_Hull) + lowLimit_Hull;

        hull.addPoint(Eigen::Vector3d(x1,y1,z1));
        hull.addPoint(Eigen::Vector3d(x1,y1,z2));
        hull.addPoint(Eigen::Vector3d(x1,y2,z1));
        hull.addPoint(Eigen::Vector3d(x1,y2,z2));
        hull.addPoint(Eigen::Vector3d(x2,y1,z1));
        hull.addPoint(Eigen::Vector3d(x2,y1,z2));
        hull.addPoint(Eigen::Vector3d(x2,y2,z1));
        hull.addPoint(Eigen::Vector3d(x2,y2,z2));

        if (x1 > x2)
        {
            double temp = x1;
            x1 = x2;
            x2 = temp;
        }

        if (y1 > y2)
        {
            double temp = y1;
            y1 = y2;
            y2 = temp;
        }

        if (z1 > z2)
        {
            double temp = z1;
            z1 = z2;
            z2 = temp;
        }

        QVERIFY((hull.getFilter(filter)));
        QVERIFY((filter.isValid()));
        QCOMPARE(hull.getNumOfUniquePoints(), 8U);

        double margin = 1.0e-3;

        for (int i = 0; i < 100; i++)
        {
            double pointX = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;
            double pointY = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;
            double pointZ = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;

            // Increase the likelihood of insides a bit (seems to be very unlikely otherwise)

            if ((randomGenerator.generate() % 5) == 0)
            {
                pointX = randomGenerator.generateDouble() * (x2 - x1) + x1;
                pointY = randomGenerator.generateDouble() * (y2 - y1) + y1;
                pointZ = randomGenerator.generateDouble() * (z2 - z1) + z1;
            }

            bool shouldBeInside = false;
            bool shouldBeOutside = false;
            if ((pointX > x1 + margin) &&
                (pointX < x2 - margin) &&
                (pointY > y1 + margin) &&
                (pointY < y2 - margin) &&
                (pointZ > z1 + margin) &&
                (pointZ < z2 - margin))
            {
                shouldBeInside = true;
                insides++;
            }
            if ((pointX < x1 - margin) ||
                (pointX > x2 + margin) ||
                (pointY < y1 - margin) ||
                (pointY > y2 + margin) ||
                (pointZ < z1 - margin) ||
                (pointZ > z2 + margin))
            {
                shouldBeOutside = true;
                outsides++;
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates++;
                continue;
            }

            Eigen::Vector3d vec(pointX, pointY, pointZ);

            QCOMPARE(filter.isInside(vec), shouldBeInside);
        }
    }
}

void TestConvexHull::randomSpheres()
{
    const double lowLimit_SphereCenter = -10;
    const double highLimit_SphereCenter = 10;
    const double lowLimit_SphereRadius = 0.001;
    const double highLimit_SphereRadius = 10;

    const int lowLimit_LatDivs = 8;
    const int highLimit_LatDivs = 16;
    const int lowLimit_LonDivs = 16;
    const int highLimit_LonDivs = 32;

    const double lowLimit_TestArea = -10;
    const double highLimit_TestArea = 10;

    const double inMargin = 0.1; // Needs to be quite big as the hull "extends" inwards from the expected radius.
    const double outMargin = 1e-6;

    // Counters for testing the test
    int outsides = 0;
    int insides = 0;
    int indeterminates = 0;

    // Disable clang/compiler warnings (set but not used)
    (void) outsides;
    (void) insides;
    (void) indeterminates;

    for (int sphere = 0; sphere < 100; sphere++)
    {
        ConvexHull hull;
        ConvexHull::Filter filter;

        double centerX = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        double centerY = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        double centerZ = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        Eigen::Vector3d centerPoint = Eigen::Vector3d(centerX, centerY, centerZ);
        double radius = randomGenerator.generateDouble() * (highLimit_SphereRadius - lowLimit_SphereRadius) - lowLimit_SphereRadius;

        int numOfLatitudes = randomGenerator.bounded(lowLimit_LatDivs, highLimit_LatDivs);

        // "Latitude" is used here quite loosely. Lat 0 is at the pole and maxlat at the other pole.

        // 3d-quickhull actually doesn't seem to like many points on the same plane
        // (like happens when there is no single point on the pole) so add points to the poles.
        // Due to rounding errors the for-loop below can not be used to this.
        // Without those points some hulls break badly.
        // (see https://github.com/karimnaaji/3d-quickhull/issues/2 )

        hull.addPoint(Eigen::Vector3d(centerX, centerY - radius, centerZ));
        hull.addPoint(Eigen::Vector3d(centerX, centerY + radius, centerZ));

        for (int lat = 1; lat < numOfLatitudes; lat++)
        {
            // This always adds several points to the poles, but they are filtered out by ConvexHull when adding.

            double lonRad = radius * sin(M_PI * double(lat) / numOfLatitudes);
            // here Y-axis is pole to pole axis
            double y = radius * cos(M_PI * double(lat) / numOfLatitudes);

            int numOfLongitudes = randomGenerator.bounded(lowLimit_LonDivs, highLimit_LonDivs);
            double randomRotation = randomGenerator.generateDouble() * M_PI * 2.0;
            for (int lon = 0; lon < numOfLongitudes; lon++)
            {
                double x = lonRad * sin((2 * M_PI * lon + randomRotation) / numOfLongitudes);
                double z = lonRad * cos((2 * M_PI * lon + randomRotation) / numOfLongitudes);

                Eigen::Vector3d point(x, y, z);
                point += centerPoint;

                hull.addPoint(point);
            }
        }

        // Add random points to the sphere surface
        int numOfRandomSurfacePoints = randomGenerator.bounded(100);
        for (int i = 0; i < numOfRandomSurfacePoints; i++)
        {
            Eigen::Vector3d point = getRandomVec().normalized() * radius + centerPoint;
            hull.addPoint(point);
        }

        // Add random points inside the sphere
        int numOfRandomInsidePoints = randomGenerator.bounded(100);
        for (int i = 0; i < numOfRandomInsidePoints; i++)
        {
            Eigen::Vector3d point = getRandomVec().normalized() * radius * randomGenerator.generateDouble() + centerPoint;
            hull.addPoint(point);
        }

//        hull.exportHullToObjFile("hullout/hull_" + QString::number(sphere) + ".obj");

        QVERIFY(hull.getFilter(filter));

        for (int i = 0; i < 100; i++)
        {
            Eigen::Vector3d point = getRandomVec(lowLimit_TestArea, highLimit_TestArea);

            // Increase the likelihood of insides a bit

            if ((randomGenerator.generate() % 5) == 0)
            {
                point = centerPoint + getRandomVec().normalized() * radius * randomGenerator.generateDouble();
            }

            bool shouldBeInside = false;
            bool shouldBeOutside = false;

            double dist = (point - centerPoint).norm();

            if (dist < radius * (1.0 - inMargin))
            {
                shouldBeInside = true;
                insides++;
            }

            if (dist > radius * (1.0 + outMargin))
            {
                shouldBeOutside = true;
                outsides++;
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates++;
                continue;
            }

/*            if (filter.isInside(point) != shouldBeInside)
            {
                // Just a debug-trap
                hull.getNumOfUniquePoints();
            }
*/

            QCOMPARE(filter.isInside(point), shouldBeInside);
        }
    }
    Q_ASSERT(insides > 10);
    Q_ASSERT(outsides > 10);
}
