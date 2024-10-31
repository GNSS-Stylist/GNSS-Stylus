#ifndef LAZYEVALUATOR_H
#define LAZYEVALUATOR_H

#include "Eigen/Geometry"
#include "qglobal.h"

namespace PointFilter
{

class LazyEvaluator
{
public:
    inline LazyEvaluator();
    inline LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine>* transform);
    inline LazyEvaluator(Eigen::Vector3d* source, Eigen::Transform<double, 3, Eigen::Affine>* transform);
    inline void invalidate(void);

    inline void setSourceEvaluator(LazyEvaluator* evaluator);
    inline void setPrimarySourceVector(Eigen::Vector3d const* newVector);

    inline void setTransform(Eigen::Transform<double, 3, Eigen::Affine> const* newTransform);

    inline Eigen::Vector3d getSourceVector(void);

    inline void setTransformedVector(const Eigen::Vector3d& newVector);
    inline Eigen::Vector3d getTransformedVector(void);
    inline double getDistance(void);

private:
    inline static const Eigen::Vector3d defaultNullSourceVector = Eigen::Vector3d();
    inline static const Eigen::Transform<double, 3, Eigen::Affine> defaultIdentityTransform = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    bool primarySource; // True: This evaluator doesn't get it's source vector from another evaluator but uses value directly from primarySourceVector instead.

    // Sources:
    Eigen::Vector3d const* primarySourceVector;
    LazyEvaluator* sourceEvaluator;
    Eigen::Transform<double, 3, Eigen::Affine> const* transform;

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

inline LazyEvaluator::LazyEvaluator()
{
    setPrimarySourceVector(&defaultNullSourceVector);
    this->transform = &defaultIdentityTransform;
}

inline LazyEvaluator::LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setSourceEvaluator(source);
    this->transform = transform;
}

inline LazyEvaluator::LazyEvaluator(Eigen::Vector3d* source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setPrimarySourceVector(source);
    this->transform = transform;
}

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

inline void LazyEvaluator::setPrimarySourceVector(Eigen::Vector3d const* newVector)
{
    primarySourceVector = newVector;
    primarySource = true;

    invalidate();
}

inline void LazyEvaluator::setTransform(Eigen::Transform<double, 3, Eigen::Affine> const* newTransform)
{
    transform = newTransform;
    evaluatedFields &= ~(EV_TRANSFORMED_VECTOR | EV_DISTANCE);
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


inline void LazyEvaluator::setTransformedVector(const Eigen::Vector3d& newVector)
{
    transformedVector = newVector;
    evaluatedFields |= EV_TRANSFORMED_VECTOR;
    evaluatedFields &= ~(EV_DISTANCE);
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
