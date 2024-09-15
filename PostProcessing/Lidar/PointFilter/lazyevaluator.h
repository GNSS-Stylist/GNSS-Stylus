#ifndef LAZYEVALUATOR_H
#define LAZYEVALUATOR_H

#include "Eigen/Geometry"
#include "qglobal.h"

namespace PointFilter
{

/*
class LazyEvaluatorSource
{
public:
    Eigen::Vector3d vector;
protected:
    Eigen::Vector3d getVector(void);

};
*/

//class LazyEvaluator : public LazyEvaluatorSource
class LazyEvaluator
{
public:
    LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine>* transform);
    LazyEvaluator(Eigen::Vector3d* source, Eigen::Transform<double, 3, Eigen::Affine>* transform);
    inline void invalidate(void);

    inline void setSourceEvaluator(LazyEvaluator* evaluator);
    inline void setPrimarySourceVector(Eigen::Vector3d* newVector);

    inline void setTransform(Eigen::Transform<double, 3, Eigen::Affine>* newTransform);

    inline Eigen::Vector3d getSourceVector(void);

    inline void setTransformedVector(Eigen::Vector3d& newVector);
    inline Eigen::Vector3d getTransformedVector(void);
    inline double getDistance(void);

private:
    bool primarySource; // True: This evaluator doesn't get it's source vector from another evaluator but uses value directly from primarySourceVector instead.

    // Sources:
    Eigen::Vector3d* primarySourceVector;
    LazyEvaluator* sourceEvaluator;
    Eigen::Transform<double, 3, Eigen::Affine>* transform;

    // Cached values:
    Eigen::Vector3d sourceVector;
    Eigen::Vector3d transformedVector;
    double distance;

    // Bitmasks of values already evaluated ("EV" from Evaluated Value):
    static const unsigned char EV_SOURCE_VECTOR =      (1 << 0);
    static const unsigned char EV_TRANSFORMED_VECTOR = (1 << 1);
    static const unsigned char EV_DISTANCE =           (1 << 2);

    // Bitmasks above are used here:
    unsigned char evaluatedFields;
};

inline void LazyEvaluator::invalidate(void)
{
    evaluatedFields = 0;
}

inline void LazyEvaluator::setSourceEvaluator(LazyEvaluator* evaluator)
{
    sourceEvaluator = evaluator;
    primarySource = false;

    invalidate();
}

inline void LazyEvaluator::setPrimarySourceVector(Eigen::Vector3d* newVector)
{
    primarySourceVector = newVector;
    primarySource = true;

    invalidate();
}

inline void LazyEvaluator::setTransform(Eigen::Transform<double, 3, Eigen::Affine>* newTransform)
{
    transform = newTransform;
    evaluatedFields &= ~(EV_SOURCE_VECTOR);
}



inline Eigen::Vector3d LazyEvaluator::getSourceVector(void)
{
    if (!(evaluatedFields & EV_SOURCE_VECTOR))
    {
        // This value could not be found from the cache yet so evaluate it
        if (primarySource)
        {
            // Value for primary source can be returned right away.
            Q_ASSERT(primarySourceVector);
            sourceVector = *primarySourceVector;
        }
        else
        {
            Q_ASSERT(sourceEvaluator);
            sourceVector = sourceEvaluator->getTransformedVector();
        }

        // We now have cached this value, so flag it as so
        evaluatedFields |= EV_SOURCE_VECTOR;
    }

    return sourceVector;
}

inline void LazyEvaluator::setTransformedVector(Eigen::Vector3d& newVector)
{
    transformedVector = newVector;
    evaluatedFields |= EV_TRANSFORMED_VECTOR;
}

inline Eigen::Vector3d LazyEvaluator::getTransformedVector(void)
{
    if (!(evaluatedFields & EV_TRANSFORMED_VECTOR))
    {
        Q_ASSERT(transform);
        transformedVector = *transform * getSourceVector();
        evaluatedFields |= EV_TRANSFORMED_VECTOR;
    }

    return transformedVector;
}

inline double LazyEvaluator::getDistance(void)
{
    if (!(evaluatedFields & EV_DISTANCE))
    {
        distance = getTransformedVector().norm();
        evaluatedFields |= EV_DISTANCE;
    }

    return distance;
}



}; // namespace PointFilter

#endif // LAZYEVALUATOR_H
