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

    if ((vec1 - vec2).norm() < std::max(vec1.norm() * 1e-6, 1e-100))    // Very small minimum value to prevent failing with _very_ short vecs
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

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), sourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Evaluator's output should not react to changed source vector before invalidation so test it
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        sourceVector = getRandomVec();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), prevSourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), prevSourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }
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

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), sourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Evaluator's output should not react to changed source vector before invalidation so test it
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        sourceVector = getRandomVec();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), prevSourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), prevSourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }
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

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Evaluator's output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransform = transform;
        sourceVector = getRandomVec();
        transform = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), prevSourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), prevTransform * prevSourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }
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

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), transform * sourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Evaluator's output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransform = transform;
        sourceVector = getRandomVec();
        transform = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), prevSourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), prevTransform * prevSourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }
    }
}

void TestLazyEvaluator::chainOfTwoEvaluators_RandomTransforms_Discrete()
{
    // Tests two "chained" evaluators (first = primary)

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transformA = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transformB = getRandomTransform();
        PointFilter::LazyEvaluator evaluatorA(&sourceVector, &transformA);
        PointFilter::LazyEvaluator evaluatorB(&evaluatorA, &transformB);

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), transformB * (transformA * sourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), transformA * sourceVector));

            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), sourceVector));

            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }

        // Evaluators' output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformA = transformA;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformB = transformB;
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), prevTransformB * (prevTransformA * prevSourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), prevTransformA * prevSourceVector));

            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), prevSourceVector));

            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }
    }
}

void TestLazyEvaluator::chainOfTwoEvaluators_RandomTransforms_InvalidateAll()
{
    // Tests two "chained" evaluators (first = primary)

    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transformA = Eigen::Affine3d::Identity();
    Eigen::Transform<double, 3, Eigen::Affine> transformB = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator evaluatorA(&sourceVector, &transformA);
    PointFilter::LazyEvaluator evaluatorB(&evaluatorA, &transformB);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();

        evaluatorA.invalidate();
        evaluatorB.invalidate();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), transformB * (transformA * sourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), transformA * sourceVector));

            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), sourceVector));

            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }

        // Evaluators' output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformA = transformA;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformB = transformB;
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), prevTransformB * (prevTransformA * prevSourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), prevTransformA * prevSourceVector));

            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), prevSourceVector));

            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }
    }
}

void TestLazyEvaluator::chainOfThreeEvaluators_RandomTransforms_Discrete()
{
    // Tests three "chained" evaluators (first = primary)

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transformA = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transformB = getRandomTransform();
        Eigen::Transform<double, 3, Eigen::Affine> transformC = getRandomTransform();
        PointFilter::LazyEvaluator evaluatorA(&sourceVector, &transformA);
        PointFilter::LazyEvaluator evaluatorB(&evaluatorA, &transformB);
        PointFilter::LazyEvaluator evaluatorC(&evaluatorB, &transformC);

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorC.getTransformedVector(), transformC * (transformB * (transformA * sourceVector))));
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), transformB * (transformA * sourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), transformA * sourceVector));

            QVERIFY(compareVectors(evaluatorC.getSourceVector(), evaluatorB.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), sourceVector));

            QCOMPARE(evaluatorC.getDistance(), evaluatorC.getTransformedVector().norm());
            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }

        // Evaluators' output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformA = transformA;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformB = transformB;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformC = transformC;
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorC.getTransformedVector(), prevTransformC * (prevTransformB * (prevTransformA * prevSourceVector))));
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), prevTransformB * (prevTransformA * prevSourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), prevTransformA * prevSourceVector));

            QVERIFY(compareVectors(evaluatorC.getSourceVector(), evaluatorB.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), prevSourceVector));

            QCOMPARE(evaluatorC.getDistance(), evaluatorC.getTransformedVector().norm());
            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }
    }
}

void TestLazyEvaluator::chainOfThreeEvaluators_RandomTransforms_InvalidateAll()
{
    // Tests three "chained" evaluators (first = primary)

    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transformA = Eigen::Affine3d::Identity();
    Eigen::Transform<double, 3, Eigen::Affine> transformB = Eigen::Affine3d::Identity();
    Eigen::Transform<double, 3, Eigen::Affine> transformC = Eigen::Affine3d::Identity();
    PointFilter::LazyEvaluator evaluatorA(&sourceVector, &transformA);
    PointFilter::LazyEvaluator evaluatorB(&evaluatorA, &transformB);
    PointFilter::LazyEvaluator evaluatorC(&evaluatorB, &transformC);

    for (int i = 0; i < 100; i++)
    {
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();
        transformC = getRandomTransform();

        evaluatorA.invalidate();
        evaluatorB.invalidate();
        evaluatorC.invalidate();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorC.getTransformedVector(), transformC * (transformB * (transformA * sourceVector))));
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), transformB * (transformA * sourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), transformA * sourceVector));

            QVERIFY(compareVectors(evaluatorC.getSourceVector(), evaluatorB.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), sourceVector));

            QCOMPARE(evaluatorC.getDistance(), evaluatorC.getTransformedVector().norm());
            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }

        // Evaluators' output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformA = transformA;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformB = transformB;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransformC = transformC;
        sourceVector = getRandomVec();
        transformA = getRandomTransform();
        transformB = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(evaluatorC.getTransformedVector(), prevTransformC * (prevTransformB * (prevTransformA * prevSourceVector))));
            QVERIFY(compareVectors(evaluatorB.getTransformedVector(), prevTransformB * (prevTransformA * prevSourceVector)));
            QVERIFY(compareVectors(evaluatorA.getTransformedVector(), prevTransformA * prevSourceVector));

            QVERIFY(compareVectors(evaluatorC.getSourceVector(), evaluatorB.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorB.getSourceVector(), evaluatorA.getTransformedVector()));
            QVERIFY(compareVectors(evaluatorA.getSourceVector(), prevSourceVector));

            QCOMPARE(evaluatorC.getDistance(), evaluatorC.getTransformedVector().norm());
            QCOMPARE(evaluatorB.getDistance(), evaluatorB.getTransformedVector().norm());
            QCOMPARE(evaluatorA.getDistance(), evaluatorA.getTransformedVector().norm());
        }
    }
}

void TestLazyEvaluator::setTransform()
{
    // Tests setting a new (randomized) transform using single primary evaluator.

    Eigen::Vector3d sourceVector;
    Eigen::Transform<double, 3, Eigen::Affine> transforms[100];
    Eigen::Transform<double, 3, Eigen::Affine> initTransform;
    PointFilter::LazyEvaluator singlePrimaryEvaluator(&sourceVector, &initTransform);

    for (int i = 0; i < 100; i++)
    {
        transforms[i] = getRandomTransform();
    }

    sourceVector = getRandomVec();

    for (int i = 0; i < 100; i++)
    {
        singlePrimaryEvaluator.setTransform(&transforms[i]);

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), transforms[i] * sourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Evaluator's output should not react to changed source vector or transform before invalidation so test these
        // (Works also as a caching test)

        Eigen::Vector3d prevSourceVector = sourceVector;
        Eigen::Transform<double, 3, Eigen::Affine> prevTransform = transforms[i];
        sourceVector = getRandomVec();
        transforms[i] = getRandomTransform();

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), prevSourceVector));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), prevTransform * prevSourceVector));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }

        // Invalidate here to get the previous source vector into use
        singlePrimaryEvaluator.invalidate();
    }
}

void TestLazyEvaluator::singlePrimaryEvaluator_DefaultConstructor()
{
    // Tests single primary evaluator created with the default constructor.
    // Evaluator is not manipulated in any way, it is just created and used.

    PointFilter::LazyEvaluator primaryEvaluatorCreatedWithDefaultConstructor;

    for (int i = 0; i < 100; i++)
    {
        PointFilter::LazyEvaluator singlePrimaryEvaluator;

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(singlePrimaryEvaluator.getSourceVector(), Eigen::Vector3d::Zero()));
            QVERIFY(compareVectors(singlePrimaryEvaluator.getTransformedVector(), Eigen::Vector3d::Zero()));

            QCOMPARE(singlePrimaryEvaluator.getDistance(), singlePrimaryEvaluator.getTransformedVector().norm());
        }
    }

    // Test that the transform and sourceVector are overridden correctly

    for (int i = 0; i < 100; i++)
    {
        Eigen::Vector3d sourceVector = getRandomVec();
        Eigen::Transform<double, 3, Eigen::Affine> transform = getRandomTransform();

        primaryEvaluatorCreatedWithDefaultConstructor.setPrimarySourceVector(&sourceVector);
        primaryEvaluatorCreatedWithDefaultConstructor.setTransform(&transform);

        for (int ii = 0; ii < 5; ii++)
        {
            QVERIFY(compareVectors(primaryEvaluatorCreatedWithDefaultConstructor.getSourceVector(), sourceVector));
            QVERIFY(compareVectors(primaryEvaluatorCreatedWithDefaultConstructor.getTransformedVector(), transform * sourceVector));

            QCOMPARE(primaryEvaluatorCreatedWithDefaultConstructor.getDistance(), primaryEvaluatorCreatedWithDefaultConstructor.getTransformedVector().norm());
        }
    }
}


//QTEST_MAIN(LazyEvaluator)

//#include "tst_lazyevaluator.moc"
