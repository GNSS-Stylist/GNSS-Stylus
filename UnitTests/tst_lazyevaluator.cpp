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

bool TestLazyEvaluator::compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2)
{
    // Compares if two vectors are close enough (< 1 millionth error)
    // "Expanded" if/else to allow breakpoints

    if ((vec1 - vec2).norm() < vec1.norm() * 1e-6)
    {
        return true;
    }
    else
    {
        return false;
    }
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
    // Tests single primary evaluator without transforms (so that the transform is always identity).
    // Evaluator is not manipulated in any way, it is just created and used.
    // SourceVector is randomized on every round.

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
        PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

        QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
        QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), sourceVector));
    }
}

void TestLazyEvaluator::singlePrimaryEvaluator_NoTransform_InvalidateAll()
{
    // Tests single primary evaluator without transforms (so that the transform is always identity).
    // Evaluator's values are invalidated and SourceVector is randomized on every round.

    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        singlePrimaryEvaluator.invalidate();

        QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
        QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), sourceVector));
    }
}

void TestLazyEvaluator::singlePrimaryEvaluator_RandomTransform_Discrete()
{
    // Tests single primary evaluator.
    // Evaluator is not manipulated in any way, it is just created and used.
    // SourceVector and transform are randomized on every round.

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transform = getRandomTransform();
        PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

        QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
        QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector));
    }
}


void TestLazyEvaluator::singlePrimaryEvaluator_RandomTransform_InvalidateAll()
{
    // Tests single primary evaluator.
    // Evaluator's values are invalidated and SourceVector and transform are randomized on every round.

    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transform = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &transform);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        transform = getRandomTransform();
        singlePrimaryEvaluator.invalidate();

        QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
        QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector));
    }
}






//QTEST_MAIN(LazyEvaluator)

//#include "tst_lazyevaluator.moc"
