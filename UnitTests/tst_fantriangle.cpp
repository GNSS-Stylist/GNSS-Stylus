/*
    tst_fantriangle.cpp (part of GNSS-Stylus)
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

#include <QtTest>

#include "tst_fantriangle.h"
#include "../PostProcessing/PointFan/fantriangle.h"

TestFanTriangle::TestFanTriangle()
{
}

TestFanTriangle::~TestFanTriangle()
{
}

void TestFanTriangle::initTestCase()
{
    randomGenerator.seed(0xdeadf00d);
}

void TestFanTriangle::cleanupTestCase()
{
}

void TestFanTriangle::simplePlanarTriangle()
{
    Eigen::Vector3d vertices[3] = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(1, 1, 0),
    };

    Eigen::Vector2d vertices_2D[3];

    for (int i = 0; i < 3; i++)
    {
        vertices_2D[i] = Eigen::Vector2d(vertices[i].x(), vertices[i].y());
    }

    FanTriangle triangle(vertices[0], vertices[1], vertices[2]);

    QCOMPARE(triangle.getNormal(), Eigen::Vector3d(0, 0, -1));

    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    // Supress clang-warnings
    (void) outsides;
    (void) insides;
    (void) indeterminates;

    // Points exactly on the corners should be regarded as being inside
    QVERIFY(triangle.testHit_2D(vertices_2D[0]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[0]), Eigen::Vector3d(vertices[0]));

    QVERIFY(triangle.testHit_2D(vertices_2D[1]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[1]), Eigen::Vector3d(vertices[1]));

    QVERIFY(triangle.testHit_2D(vertices_2D[2]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[2]), Eigen::Vector3d(vertices[2]));

    // Points close enough ("on" the edge, but with limited precision) to the edges should be regarded as being inside
    for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
    {
        for (int i = 0; i < 1000; i++)
        {
            Eigen::Vector2d point = vertices_2D[vertexIndex] + (vertices_2D[(vertexIndex + 1) % 3] - vertices_2D[vertexIndex]) * double(i) / 1000.0;
            QVERIFY(triangle.testHit_2D(point));
        }
    }

    // Test "grid" of points
    for (int intX = -1000; intX < 2000; intX += 10)
    {
        for (int intY = -1000; intY < 2000; intY += 10)
        {
            double x = double(intX) / 1000.0;
            double y = double(intY) / 1000.0;

            bool shouldBeOutside = false;
            bool shouldBeInside = false;

            if (x < -FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (y < -FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (x > 1.0 + FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (y > 1.0 + FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (x / y > 1 + (FanTriangle::edgeLimit * sqrt(2) * 2))
            {
                shouldBeOutside = true;
            }

            if (!shouldBeOutside)
            {
                shouldBeInside = (x > FanTriangle::edgeLimit * 2) &&
                                 (y > FanTriangle::edgeLimit * 2) &&
                                 (x < 1 - (FanTriangle::edgeLimit * 2)) &&
                                 (y < 1 - (FanTriangle::edgeLimit * 2)) &&
                                 (x / y < (1 - (FanTriangle::edgeLimit * sqrt(2) * 2)));
            }

            if (shouldBeInside)
            {
                QVERIFY(triangle.testHit_2D(Eigen::Vector2d(x, y)));
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
                insides++;
            }
            else if (shouldBeOutside)
            {
                QVERIFY(!triangle.testHit_2D(Eigen::Vector2d(x, y)));
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
                outsides++;
            }
            else
            {
                indeterminates++;
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
            }
        }
    }

    bool dbgTrap = 0; (void) dbgTrap;
}


void TestFanTriangle::simplePlanarTriangle_ReverseWindingOrder()
{
    // This test is otherwise identical to the one above, but winding order is reversed
    // (also causes the normal to be reversed)
    Eigen::Vector3d vertices[3] = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1, 1, 0),
        Eigen::Vector3d(0, 1, 0),
    };

    Eigen::Vector2d vertices_2D[3];

    for (int i = 0; i < 3; i++)
    {
        vertices_2D[i] = Eigen::Vector2d(vertices[i].x(), vertices[i].y());
    }

    FanTriangle triangle(vertices[0], vertices[1], vertices[2]);

    QCOMPARE(triangle.getNormal(), Eigen::Vector3d(0, 0, 1));

    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    // Supress clang-warnings
    (void) outsides;
    (void) insides;
    (void) indeterminates;

    // Points exactly on the corners should be regarded as being inside
    QVERIFY(triangle.testHit_2D(vertices_2D[0]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[0]), Eigen::Vector3d(vertices[0]));

    QVERIFY(triangle.testHit_2D(vertices_2D[1]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[1]), Eigen::Vector3d(vertices[1]));

    QVERIFY(triangle.testHit_2D(vertices_2D[2]));
    QCOMPARE(triangle.getHitPoint(vertices_2D[2]), Eigen::Vector3d(vertices[2]));

    // Points close enough ("on" the edge, but with limited precision) to the edges should be regarded as being inside
    for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
    {
        for (int i = 0; i < 1000; i++)
        {
            Eigen::Vector2d point = vertices_2D[vertexIndex] + (vertices_2D[(vertexIndex + 1) % 3] - vertices_2D[vertexIndex]) * double(i) / 1000.0;
            QVERIFY(triangle.testHit_2D(point));
        }
    }

    // Test "grid" of points
    for (int intX = -1000; intX < 2000; intX += 10)
    {
        for (int intY = -1000; intY < 2000; intY += 10)
        {
            double x = double(intX) / 1000.0;
            double y = double(intY) / 1000.0;

            bool shouldBeOutside = false;
            bool shouldBeInside = false;

            if (x < -FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (y < -FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (x > 1.0 + FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (y > 1.0 + FanTriangle::edgeLimit * 2)
            {
                shouldBeOutside = true;
            }

            if (x / y > 1 + (FanTriangle::edgeLimit * sqrt(2) * 2))
            {
                shouldBeOutside = true;
            }

            if (!shouldBeOutside)
            {
                shouldBeInside = (x > FanTriangle::edgeLimit * 2) &&
                                 (y > FanTriangle::edgeLimit * 2) &&
                                 (x < 1 - (FanTriangle::edgeLimit * 2)) &&
                                 (y < 1 - (FanTriangle::edgeLimit * 2)) &&
                                 (x / y < (1 - (FanTriangle::edgeLimit * sqrt(2) * 2)));
            }

            if (shouldBeInside)
            {
                QVERIFY(triangle.testHit_2D(Eigen::Vector2d(x, y)));
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
                insides++;
            }
            else if (shouldBeOutside)
            {
                QVERIFY(!triangle.testHit_2D(Eigen::Vector2d(x, y)));
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
                outsides++;
            }
            else
            {
                indeterminates++;
                QCOMPARE(triangle.getHitPoint(Eigen::Vector2d(x, y)), Eigen::Vector3d(x, y, 0));
            }
        }
    }

    bool dbgTrap = 0; (void) dbgTrap;
}


void TestFanTriangle::randomTriangles()
{
    int insides = 0;
    int outsides = 0;
    int indeterminates = 0;

    // Supress clang-warnings
    (void) outsides;
    (void) insides;
    (void) indeterminates;

    for (int round = 0; round < 100; round++)
    {
        Eigen::Vector3d vertices[3];
        Eigen::Vector2d vertices_2D[3];

        for (int i = 0; i < 3; i++)
        {
            vertices[i] = Eigen::Vector3d(randomGenerator.generateDouble() * 2 - 1, randomGenerator.generateDouble() * 2 - 1, randomGenerator.generateDouble() * 2 - 1);
            vertices_2D[i] = Eigen::Vector2d(vertices[i].x(), vertices[i].y());
        }

        FanTriangle triangle(vertices[0], vertices[1], vertices[2]);

        QCOMPARE(triangle.getNormal(), ((vertices[1] - vertices[0]).cross(vertices[2] - vertices[0])).normalized());

        // Points exactly on the corners should be regarded as being inside
        QVERIFY(triangle.testHit_2D(vertices_2D[0]));
        QCOMPARE(triangle.getHitPoint(vertices_2D[0]), vertices[0]);

        Eigen::Vector3d toTest = triangle.getHitPoint(vertices_2D[1]);
        Eigen::Vector3d expected = vertices[1];
        Eigen::Vector3d diff = toTest - expected;

        QVERIFY(triangle.testHit_2D(vertices_2D[1]));
        QVERIFY(compareVectors(triangle.getHitPoint(vertices_2D[1]), vertices[1]));

        toTest = triangle.getHitPoint(vertices_2D[2]);
        expected = vertices[2];
        diff = toTest - expected;

        QVERIFY(triangle.testHit_2D(vertices_2D[2]));
        QVERIFY(compareVectors(triangle.getHitPoint(vertices_2D[2]), vertices[2]));

        // Points close enough ("on" the edge, but with limited precision) to the edges should be regarded as being inside
        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
        {
            for (int i = 0; i < 1000; i++)
            {
                Eigen::Vector2d point_2D = vertices_2D[vertexIndex] + (vertices_2D[(vertexIndex + 1) % 3] - vertices_2D[vertexIndex]) * double(i) / 1000.0;
                Eigen::Vector3d point_3D = vertices[vertexIndex] + (vertices[(vertexIndex + 1) % 3] - vertices[vertexIndex]) * double(i) / 1000.0;
                QVERIFY(triangle.testHit_2D(point_2D));
                QVERIFY(compareVectors(triangle.getHitPoint(point_2D), point_3D));
            }
        }

        // Test centerpoint
        Eigen::Vector2d centerPoint(0, 0);

        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
        {
            centerPoint += vertices_2D[vertexIndex];
        }

        centerPoint /= 3;
        QVERIFY(triangle.testHit_2D(centerPoint));

        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::min();
        double minY = std::numeric_limits<double>::max();
        double maxY = std::numeric_limits<double>::min();

        for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
        {
            minX = std::min(minX, vertices[vertexIndex].x());
            maxX = std::max(maxX, vertices[vertexIndex].x());

            minY = std::min(minY, vertices[vertexIndex].y());
            maxY = std::max(maxY, vertices[vertexIndex].y());
        }

        minX -= FanTriangle::edgeLimit * 2;
        maxX += FanTriangle::edgeLimit * 2;

        minY -= FanTriangle::edgeLimit * 2;
        maxY += FanTriangle::edgeLimit * 2;

        // Test "grid" of points
        // Testing only definitely outside points (= outside the bounding box + margin) here. Edge cases are already tested above.
        for (int intX = -1000; intX < 1000; intX += 10)
        {
            for (int intY = -1000; intY < 1000; intY += 10)
            {
                double x = double(intX) / 1000.0;
                double y = double(intY) / 1000.0;

                bool shouldBeOutside = false;

                if (x < minX)
                {
                    shouldBeOutside = true;
                }

                if (y < minY)
                {
                    shouldBeOutside = true;
                }

                if (x > maxX)
                {
                    shouldBeOutside = true;
                }

                if (y > maxY)
                {
                    shouldBeOutside = true;
                }

                if (shouldBeOutside)
                {
                    QVERIFY(!triangle.testHit_2D(Eigen::Vector2d(x, y)));
                    outsides++;
                }
                else
                {
                    indeterminates++;
                }
            }
        }
    }

    bool dbgTrap = 0; (void) dbgTrap;

}


bool TestFanTriangle::compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2)
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

