/*
    lazyevaluator.h (part of GNSS-Stylus)
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
    inline LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine>* transform = nullptr);
    inline LazyEvaluator(const Eigen::Vector3d* source, Eigen::Transform<double, 3, Eigen::Affine>* transform = nullptr);
    inline LazyEvaluator(double const* horizontalAngle2D, double const* horizontalDistance2D, Eigen::Transform<double, 3, Eigen::Affine> *transform = nullptr);
    inline void invalidate(void);

    inline void setSourceEvaluator(LazyEvaluator* evaluator);
    inline void setPrimarySourceVector(Eigen::Vector3d const* newVector);
    inline void setPrimarySourceVector2D(double const* horizontalAngle2D, double const* horizontalDistance2D);

    inline void setTransform(Eigen::Transform<double, 3, Eigen::Affine> const* newTransform);
    inline void clearTransform(void); // Use default (identity) transform (= no transform)

    inline Eigen::Vector3d getSourceVector(void);
    inline const Eigen::Vector3d* getSourceVectorPtr(void);

    inline Eigen::Vector3d getTransformedVector(void);
    inline const Eigen::Vector3d* getTransformedVectorPtr(void);
    inline double getDistance(void);
    inline const double* getDistancePtr(void);
    inline double getHorizontalAngle(void);
    inline const double* getHorizontalAnglePtr(void);
    inline double getVerticalAngle(void);
    inline const double* getVerticalAnglePtr(void);

private:
    inline static const Eigen::Vector3d defaultNullSourceVector = Eigen::Vector3d(0, 0, 0);

    enum SourceType
    {
        ST_EVALUATOR = 0,
        ST_VECTOR,
        ST_ANGLE_DIST_2D,
    };

    struct
    {
        SourceType type;

        union
        {
            // Sources (pointers used to prevent unnecessary copying):
            Eigen::Vector3d const* primarySourceVector;
            // Angles & distance (used for RPLidar, when vertical angle is 0):
            struct
            {
                double const* primaryHorizontalAngle2D;
                double const* primaryHorizontalDistance2D;
            } angleDistance2D;
            // TODO: Add 3D angles & distance if/when needed
            LazyEvaluator* sourceEvaluator;
        };
    } source;


    Eigen::Transform<double, 3, Eigen::Affine> const* transform;
    bool transformIsInUse;

    // Cached values:
    Eigen::Vector3d sourceVector;
    Eigen::Vector3d transformedVector;
    double distance;
    double horizontalAngle;
    double verticalAngle;

    // Bitmasks of values already evaluated ("EV" from Evaluated Value):
    static constexpr unsigned char EV_SOURCE_VECTOR =      (1 << 0);
    static constexpr unsigned char EV_SOURCE_DISTANCE_2D = (1 << 1);
    static constexpr unsigned char EV_SOURCE_ANGLE_2D =    (1 << 2);
    static constexpr unsigned char EV_TRANSFORMED_VECTOR = (1 << 3);
    static constexpr unsigned char EV_DISTANCE =           (1 << 4);
    static constexpr unsigned char EV_HORIZONTAL_ANGLE =   (1 << 5);
    static constexpr unsigned char EV_VERTICAL_ANGLE =     (1 << 6);

    static constexpr unsigned char EVMASK_SOURCE_VALUES =  EV_SOURCE_VECTOR | EV_SOURCE_DISTANCE_2D | EV_SOURCE_ANGLE_2D;

    // Bitmasks above are used here:
    unsigned char evaluatedFields;
};

inline LazyEvaluator::LazyEvaluator()
{
    setPrimarySourceVector(&defaultNullSourceVector);
    clearTransform();
    source.type = ST_VECTOR;
    evaluatedFields = 0;
}

inline LazyEvaluator::LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setSourceEvaluator(source);
    this->transform = transform;
    transformIsInUse = transform != nullptr;
    this->source.type = ST_EVALUATOR;
    evaluatedFields = 0;
}

inline LazyEvaluator::LazyEvaluator(const Eigen::Vector3d *source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setPrimarySourceVector(source);
    this->transform = transform;
    transformIsInUse = transform != nullptr;
    this->source.type = ST_VECTOR;
    evaluatedFields = 0;
}

inline LazyEvaluator::LazyEvaluator(double const* horizontalAngle2D, double const* horizontalDistance2D, Eigen::Transform<double, 3, Eigen::Affine>* transform)
{
    setPrimarySourceVector2D(horizontalAngle2D, horizontalDistance2D);
    this->transform = transform;
    transformIsInUse = transform != nullptr;
    source.type = ST_ANGLE_DIST_2D;
    evaluatedFields = 0;
}

inline void LazyEvaluator::invalidate(void)
{
    evaluatedFields = 0;
}

inline void LazyEvaluator::setSourceEvaluator(LazyEvaluator* evaluator)
{
    source.sourceEvaluator = evaluator;
    source.type = ST_EVALUATOR;

    invalidate();
}

inline void LazyEvaluator::setPrimarySourceVector(Eigen::Vector3d const* newVector)
{
    source.primarySourceVector = newVector;
    source.type = ST_VECTOR;

    invalidate();
}

inline void LazyEvaluator::setPrimarySourceVector2D(const double *horizontalAngle2D, const double *horizontalDistance2D)
{
    source.angleDistance2D.primaryHorizontalAngle2D = horizontalAngle2D;
    source.angleDistance2D.primaryHorizontalDistance2D = horizontalDistance2D;
    source.type = ST_ANGLE_DIST_2D;

    invalidate();
}

inline void LazyEvaluator::setTransform(Eigen::Transform<double, 3, Eigen::Affine> const* newTransform)
{
    transform = newTransform;
    evaluatedFields &= EVMASK_SOURCE_VALUES;
    transformIsInUse = newTransform != nullptr;
}

inline void LazyEvaluator::clearTransform(void)
{
    this->transform = nullptr;
    this->transformIsInUse = false;
    evaluatedFields &= EVMASK_SOURCE_VALUES;
}

inline Eigen::Vector3d LazyEvaluator::getSourceVector(void)
{
    return *getSourceVectorPtr();
}

inline const Eigen::Vector3d* LazyEvaluator::getSourceVectorPtr(void)
{
    if (!(evaluatedFields & EV_SOURCE_VECTOR))
    {
        // This value could not be found from the cache yet so evaluate it

        switch (source.type)
        {
        case ST_EVALUATOR:
            Q_ASSERT(source.sourceEvaluator);
            sourceVector = source.sourceEvaluator->getTransformedVector();
            break;

        case ST_VECTOR:
            // Value for primary source can be returned right away.
            Q_ASSERT(source.primarySourceVector);
            sourceVector = *source.primarySourceVector;
            break;

        case ST_ANGLE_DIST_2D:
        {
            Q_ASSERT(source.angleDistance2D.primaryHorizontalDistance2D);
            Q_ASSERT(source.angleDistance2D.primaryHorizontalAngle2D);
            sourceVector = Eigen::Vector3d(*source.angleDistance2D.primaryHorizontalDistance2D * sin(*source.angleDistance2D.primaryHorizontalAngle2D), *source.angleDistance2D.primaryHorizontalDistance2D * cos(*source.angleDistance2D.primaryHorizontalAngle2D), 0.0);
            break;
        }

        default:
            qFatal("Unhandled source type.");
            sourceVector = defaultNullSourceVector;
            break;
        }

        // We now have cached this value, so flag it as so
        evaluatedFields |= EV_SOURCE_VECTOR;
    }

    return &sourceVector;
}


inline Eigen::Vector3d LazyEvaluator::getTransformedVector(void)
{
    return *getTransformedVectorPtr();
}

inline const Eigen::Vector3d* LazyEvaluator::getTransformedVectorPtr(void)
{
    if (!(evaluatedFields & EV_TRANSFORMED_VECTOR))
    {
        if (transformIsInUse)
        {
            Q_ASSERT(transform);
            transformedVector = *transform * *getSourceVectorPtr();
        }
        else
        {
            transformedVector = *getSourceVectorPtr();
        }
        evaluatedFields |= EV_TRANSFORMED_VECTOR;
    }

    return &transformedVector;
}


inline double LazyEvaluator::getDistance(void)
{
    return *getDistancePtr();
}

inline const double* LazyEvaluator::getDistancePtr(void)
{
    if (!(evaluatedFields & EV_DISTANCE))
    {
        if ((transformIsInUse) || (source.type != ST_ANGLE_DIST_2D))
        {
            distance = getTransformedVectorPtr()->norm();
        }
        else
        {
            // If source is 2D-angle&distance and no transform is in use, we can read the distance straight away
            distance = *source.angleDistance2D.primaryHorizontalDistance2D;
            evaluatedFields |= EV_SOURCE_DISTANCE_2D;
        }

        evaluatedFields |= EV_DISTANCE;
    }

    return &distance;
}

inline double LazyEvaluator::getHorizontalAngle(void)
{
    return *getHorizontalAnglePtr();
}

inline const double* LazyEvaluator::getHorizontalAnglePtr(void)
{
    if (!(evaluatedFields & EV_HORIZONTAL_ANGLE))
    {
        if ((transformIsInUse) || (source.type != ST_ANGLE_DIST_2D))
        {
            const Eigen::Vector3d* vec = getTransformedVectorPtr();
            horizontalAngle = atan2(vec->x(), vec->y());
        }
        else
        {
            // If source is 2D-angle&distance and no transform is in use, we can read the angle straight away
            horizontalAngle = *source.angleDistance2D.primaryHorizontalAngle2D;
            evaluatedFields |= EV_SOURCE_ANGLE_2D;
        }
        evaluatedFields |= EV_HORIZONTAL_ANGLE;
    }

    return &horizontalAngle;
}


inline double LazyEvaluator::getVerticalAngle(void)
{
    return *getVerticalAnglePtr();
}

inline const double* LazyEvaluator::getVerticalAnglePtr(void)
{
    if (!(evaluatedFields & EV_VERTICAL_ANGLE))
    {
        if ((transformIsInUse) || (source.type != ST_ANGLE_DIST_2D))
        {
            const Eigen::Vector3d* vec = getTransformedVectorPtr();
            verticalAngle = atan2(vec->z(), sqrt(vec->x() * vec->x() + vec->y() * vec->y()));
        }
        else
        {
            // If source is 2D-angle&distance and no transform is in use, vertical angle is always 0
            verticalAngle = 0;
        }
        evaluatedFields |= EV_VERTICAL_ANGLE;
    }

    return &verticalAngle;
}


}; // namespace PointFilter

#endif // LAZYEVALUATOR_H
