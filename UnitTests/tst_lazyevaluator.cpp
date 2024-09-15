/*
    tst_lazyevaluator.cpp (part of GNSS-Stylus)
    Copyright (C) 2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "tst_lazyevaluator.h"
#include "../PostProcessing/Lidar/PointFilter/lazyevaluator.h"

Eigen::Vector3d TestLazyEvaluator::getRandomVec(double lowLimit, double highLimit)
{
    return Eigen::Vector3d(randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
        randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit,
        randomGenerator.generateDouble() * (highLimit - lowLimit) + lowLimit
        );
}

Eigen::Transform<double, 3, Eigen::Affine> TestLazyEvaluator::getRandomTransform(double translateLowLimit, double translateHighLimit)
{
    // Doesn't return very evenly distributed transforms, but should suffice in this context.

    Eigen::AngleAxisd orientation(randomGenerator.generateDouble() * (2 * M_PI), getRandomVec(-1.0, 1.0).normalized());
    Eigen::Transform<double, 3, Eigen::Affine> ret;
    ret.fromPositionOrientationScale(getRandomVec(translateLowLimit, translateHighLimit), orientation, getRandomVec(-10.0, 10.0));

    return ret;
}


TestLazyEvaluator::TestLazyEvaluator()
{

}

TestLazyEvaluator::~TestLazyEvaluator()
{

}

void TestLazyEvaluator::initTestCase()
{
    randomGenerator.seed(42);
}

void TestLazyEvaluator::cleanupTestCase()
{

}

void TestLazyEvaluator::singlePrimaryEvaluator_NoTransform_Discrete()
{
    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
        PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

        QCOMPARE(singlePrimaryEvaluator.getSourceVector(), sourceVector);
        QCOMPARE(singlePrimaryEvaluator.getTransformedVector(), sourceVector);
    }
}

void TestLazyEvaluator::singlePrimaryEvaluator_NoTransform_InvalidateAll()
{
    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        singlePrimaryEvaluator.invalidate();

        QCOMPARE(singlePrimaryEvaluator.getSourceVector(), sourceVector);
        QCOMPARE(singlePrimaryEvaluator.getTransformedVector(), sourceVector);
    }
}

void TestLazyEvaluator::singlePrimaryEvaluator_RandomTransform_Discrete()
{
    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transform = getRandomTransform();
        PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

        QCOMPARE(singlePrimaryEvaluator.getSourceVector(), sourceVector);
        QCOMPARE(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector);
    }
}


void TestLazyEvaluator::singlePrimaryEvaluator_RandomTransform_InvalidateAll()
{
    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        transform = getRandomTransform();
        singlePrimaryEvaluator.invalidate();

        QCOMPARE(singlePrimaryEvaluator.getSourceVector(), sourceVector);
        QCOMPARE(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector);
    }
}






//QTEST_MAIN(LazyEvaluator)

//#include "tst_lazyevaluator.moc"
