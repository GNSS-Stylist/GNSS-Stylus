/*
    tst_convexhull.cpp (part of GNSS-Stylus)
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

#include "tst_convexhull.h"
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

Eigen::Transform<double, 3, Eigen::Affine> TestConvexHull::getRandomTransform(double translateLowLimit, double translateHighLimit)
{
    // Doesn't return very evenly distributed transforms, but should suffice in this context.

    Eigen::AngleAxisd orientation(randomGenerator.generateDouble() * (2 * M_PI), getRandomVec(-1.0, 1.0).normalized());
    Eigen::Transform<double, 3, Eigen::Affine> ret;
    //    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, getRandomVec(-10.0, 10.0));
    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, Eigen::Vector3d(1,1,1));

    return ret;
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
    int outsides_NoHullMargin = 0;
    int insides_NoHullMargin = 0;
    int indeterminates_NoHullMargin = 0;
    int outsides_WithHullMargin = 0;
    int insides_WithHullMargin = 0;
    int indeterminates_WithHullMargin = 0;
    int nullHulls_WithHullMargin = 0;

    // Disable clang/compiler warnings (set but not used)
    (void) outsides_NoHullMargin;
    (void) insides_NoHullMargin;
    (void) indeterminates_NoHullMargin;
    (void) outsides_WithHullMargin;
    (void) insides_WithHullMargin;
    (void) indeterminates_WithHullMargin;
    (void) nullHulls_WithHullMargin;

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

        double edgeMargin = 1.0e-3;

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
            if ((pointX > x1 + edgeMargin) &&
                (pointX < x2 - edgeMargin) &&
                (pointY > y1 + edgeMargin) &&
                (pointY < y2 - edgeMargin) &&
                (pointZ > z1 + edgeMargin) &&
                (pointZ < z2 - edgeMargin))
            {
                shouldBeInside = true;
                insides_NoHullMargin++;
            }
            if ((pointX < x1 - edgeMargin) ||
                (pointX > x2 + edgeMargin) ||
                (pointY < y1 - edgeMargin) ||
                (pointY > y2 + edgeMargin) ||
                (pointZ < z1 - edgeMargin) ||
                (pointZ > z2 + edgeMargin))
            {
                shouldBeOutside = true;
                outsides_NoHullMargin++;
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates_NoHullMargin++;
                continue;
            }

            Eigen::Vector3d vec(pointX, pointY, pointZ);

            QCOMPARE(filter.isInside(vec), shouldBeInside);
        }

        // Separate test with hull margins (to leave the previous loop less obfuscated if something breaks)
        double minDim = std::min(x2-x1, std::min(y2 - y1, z2 - z1));
        double minMargin = -minDim * 0.75;   // Margin can be so large that the hull effectively ceases to exists (which is ok)
        const double maxMargin = 1.0;

        for (int i = 0; i < 100; i++)
        {
            double pointX = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;
            double pointY = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;
            double pointZ = randomGenerator.generateDouble() * (highLimit_TestArea - lowLimit_TestArea) + lowLimit_TestArea;
            double margin = randomGenerator.generateDouble() * (maxMargin - minMargin) + minMargin;

            double xx1 = x1 - margin;
            double xx2 = x2 + margin;
            double yy1 = y1 - margin;
            double yy2 = y2 + margin;
            double zz1 = z1 - margin;
            double zz2 = z2 + margin;

            // Increase the likelihood of insides a bit (seems to be very unlikely otherwise)

            if ((randomGenerator.generate() % 5) == 0)
            {
                pointX = randomGenerator.generateDouble() * (xx2 - xx1) + xx1;
                pointY = randomGenerator.generateDouble() * (yy2 - yy1) + yy1;
                pointZ = randomGenerator.generateDouble() * (zz2 - zz1) + zz1;
            }

            bool shouldBeInside = false;
            bool shouldBeOutside = false;
            if ((pointX > xx1 + edgeMargin) &&
                (pointX < xx2 - edgeMargin) &&
                (pointY > yy1 + edgeMargin) &&
                (pointY < yy2 - edgeMargin) &&
                (pointZ > zz1 + edgeMargin) &&
                (pointZ < zz2 - edgeMargin))
            {
                shouldBeInside = true;
                insides_WithHullMargin++;
            }
            if ((pointX < xx1 - edgeMargin) ||
                (pointX > xx2 + edgeMargin) ||
                (pointY < yy1 - edgeMargin) ||
                (pointY > yy2 + edgeMargin) ||
                (pointZ < zz1 - edgeMargin) ||
                (pointZ > zz2 + edgeMargin))
            {
                shouldBeOutside = true;
                outsides_WithHullMargin++;
            }

            if ((xx2 < xx1) || (yy2 < yy1) || (zz2 < zz1))
            {
                // Margin is bigger than the dimension of the box in some axis
                nullHulls_WithHullMargin++;
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates_WithHullMargin++;
                continue;
            }

            Eigen::Vector3d vec(pointX, pointY, pointZ);
/*
            if (filter.isInside(vec, margin) != shouldBeInside)
            {
                // debug-trap
                filter.isValid();
            }
*/
            QCOMPARE(filter.isInside(vec, margin), shouldBeInside);
        }
    }

//    int foo = insides_NoHullMargin; // debug-trap
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

    const double edgeMargin_Inside_NoHullMargins = 0.1; // Needs to be quite big as the hull "extends" inwards from the expected radius.
    const double edgeMargin_Outside_NoHullMargins = 1e-6;

    // Need separate edge margins for hulls with margins as hull margins in this case affect the faces instead of vertices
    const double edgeMargin_Inside_WithHullMargins = 0.1; // Needs to be quite big as the hull "extends" inwards from the expected radius.
    const double edgeMargin_Outside_WithHullMargins = 0.1; // Needs to be quite big as hull margin can extend the hull vertices more than it's value

    // Counters for testing the test
    int outsides_NoHullMargin = 0;
    int insides_NoHullMargin = 0;
    int indeterminates_NoHullMargin = 0;
    int outsides_WithHullMargin = 0;
    int insides_WithHullMargin = 0;
    int indeterminates_WithHullMargin = 0;
    int nullHulls_WithHullMargin = 0;

    // Disable clang/compiler warnings (set but not used)
    (void) outsides_NoHullMargin;
    (void) insides_NoHullMargin;
    (void) indeterminates_NoHullMargin;
    (void) outsides_WithHullMargin;
    (void) insides_WithHullMargin;
    (void) indeterminates_WithHullMargin;
    (void) nullHulls_WithHullMargin;

    for (int sphere = 0; sphere < 10; sphere++)
    {
        ConvexHull hull;
        ConvexHull::Filter filter;

        double centerX = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        double centerY = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        double centerZ = randomGenerator.generateDouble() * (highLimit_SphereCenter - lowLimit_SphereCenter) + lowLimit_SphereCenter;
        Eigen::Vector3d centerPoint = Eigen::Vector3d(centerX, centerY, centerZ);
        double radius = randomGenerator.generateDouble() * (highLimit_SphereRadius - lowLimit_SphereRadius) - lowLimit_SphereRadius;

        // Add points to both poles.
        hull.addPoint(Eigen::Vector3d(centerX, centerY - radius, centerZ));
        hull.addPoint(Eigen::Vector3d(centerX, centerY + radius, centerZ));

        // "Latitude" is used here quite loosely. Lat 0 is at the pole and maxlat at the other pole.
        int numOfLatitudes = randomGenerator.bounded(lowLimit_LatDivs, highLimit_LatDivs);

        for (int lat = 1; lat < numOfLatitudes; lat++)
        {
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

            if (dist < radius * (1.0 - edgeMargin_Inside_NoHullMargins))
            {
                shouldBeInside = true;
                insides_NoHullMargin++;
            }

            if (dist > radius * (1.0 + edgeMargin_Outside_NoHullMargins))
            {
                shouldBeOutside = true;
                outsides_NoHullMargin++;
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates_NoHullMargin++;
                continue;
            }

            /*
            if (filter.isInside(point) != shouldBeInside)
            {
                // Output for reporting a 3d-quickhull-bug
                //std::cout << "Broken hull points:\n";
                //for (int i = 0; i < hull.points.size(); i++)
                //{
                //    std::cout << "    { " << QString::number(hull.points[i].x(),'g', 14).toStdString() << ", " << QString::number(hull.points[i].y(),'g', 14).toStdString() << ", " << QString::number(hull.points[i].z(),'g', 14).toStdString() << " },\n";
                //}

                // Just a debug-trap
                filter.isInside(point);
            }
            */

            QCOMPARE(filter.isInside(point), shouldBeInside);
        }

        // Separate test with hull margins (to leave the previous loop less obfuscated if something breaks)
        double minMargin = -radius * 0.5;   // Margin could be so large that the hull effectively ceases to exists (which is ok).
                                            // With "non-uniform" spheres, however, the shape can get distorted, so don't go too low with this.
        const double maxMargin = 1.0;

        for (int i = 0; i < 100; i++)
        {
            Eigen::Vector3d point = getRandomVec(lowLimit_TestArea, highLimit_TestArea);
            double margin = randomGenerator.generateDouble() * (maxMargin - minMargin) + minMargin;

            // Increase the likelihood of insides a bit

            if ((randomGenerator.generate() % 5) == 0)
            {
                point = centerPoint + getRandomVec().normalized() * (radius + margin) * randomGenerator.generateDouble();
            }

            bool shouldBeInside = false;
            bool shouldBeOutside = false;

            double dist = (point - centerPoint).norm();

            if (dist < (radius + margin) * (1.0 - edgeMargin_Inside_WithHullMargins))
            {
                shouldBeInside = true;
                insides_WithHullMargin++;
            }

            if (dist > (radius + margin) * (1.0 + edgeMargin_Outside_WithHullMargins))
            {
                shouldBeOutside = true;
                outsides_WithHullMargin++;
            }

            if ((radius + margin) < 0)
            {
                nullHulls_WithHullMargin++;
                QCOMPARE(filter.isInside(point, margin), false);
            }

            if ((!shouldBeInside) && (!shouldBeOutside))
            {
                // Inside margin(s) -> just skip
                indeterminates_NoHullMargin++;
                continue;
            }

            /*
            if (filter.isInside(point, margin) != shouldBeInside)
            {
                // Just a debug-trap
                filter.isInside(point, margin);
            }
            */

            QCOMPARE(filter.isInside(point, margin), shouldBeInside);
        }
    }
    Q_ASSERT(insides_NoHullMargin > 10);
    Q_ASSERT(outsides_NoHullMargin > 10);
}

void TestConvexHull::filterOptimization()
{
    const int rounds = 100;
    const int testPoints = 100;
    const double baseEdgeWidth = 1e-3;  // Used for "bar" ends and added to circumference
    const double edgeWidthMultiplier = 2e-2; // Relative error on the circumference (found by experimenting, too lazy to calculate)

    Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    int round = 0;
    int divs = 16;
    double height = 1.0;
    double radius = 1.0;
    double angleShift = 0.0;

    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    (void) insides;
    (void) outsides;
    (void) indeterminates;

    do
    {
        ConvexHull hull;

        for (int i = 0; i < divs; i++)
        {
            double angle = angleShift + i * 2 * M_PI / divs;
            hull.addPoint(transform * Eigen::Vector3d(sin(angle) * radius, cos(angle) * radius, height / 2));
            hull.addPoint(transform * Eigen::Vector3d(sin(angle) * radius, cos(angle) * radius, -height / 2));
        }

        // hull.exportHullToObjFile(QString("bars/") + QString::number(round));

        ConvexHull::Filter filter;

        QVERIFY(hull.getFilter(filter));

        // This needs changes to ConvexHull:Filter (facedefs needs to be public)
        // So not normally tested.
        // Optimization should combine faces to this
        // (only one def per end and one for every side section)
        // For cube def count should halve.
        // These ends consisting of 16-gons seem to originally have 14 faces optimized to 1 here.
//        QCOMPARE(filter.faceDefs.size(), 2 + divs);

        for (int i = 0; i < testPoints; i++)
        {
            // Limit test points around the bar

            Eigen::Vector3d testPoint((randomGenerator.generateDouble() * 2.0 - 0.5) * 1.2 * radius,
                                      (randomGenerator.generateDouble() * 2.0 - 0.5) * 1.2 * radius,
                                      (randomGenerator.generateDouble() * 2.0 - 0.5) / 2 * 1.2 * height);

            double distXY = sqrt(testPoint.x() * testPoint.x() + testPoint.y() * testPoint.y());

            bool shouldBeInside = false;
            bool indeterminate = false;

            if ((distXY < radius - baseEdgeWidth - edgeWidthMultiplier * radius) &&
                (testPoint.z() < height / 2.0 - baseEdgeWidth))
            {
                shouldBeInside = true;
                insides++;
            }
            else if ((distXY > radius + baseEdgeWidth + edgeWidthMultiplier * radius) ||
                (fabs(testPoint.z()) > height / 2.0 + baseEdgeWidth))

            {
                outsides++;
            }
            else
            {
                indeterminate = true;
                indeterminates++;
            }

            if (!indeterminate)
            {
                QCOMPARE(filter.isInside(transform * testPoint), shouldBeInside);
            }
        }

        transform = getRandomTransform();
        divs = randomGenerator.bounded(16, 24); // too high number here may cause assertion failure on convhull_3d (apparently convex hull algorithms do not like many coplanar points...)
        height = 0.1 + randomGenerator.generateDouble() * 5;
        radius = 0.1 + randomGenerator.generateDouble() * 5;
        angleShift = randomGenerator.generateDouble() * 2 * M_PI;
    } while (round++ < rounds);

//    int foo = 42;
}
