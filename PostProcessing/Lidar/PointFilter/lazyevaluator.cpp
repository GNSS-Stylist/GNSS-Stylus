#include "lazyevaluator.h"

namespace PointFilter
{

#include "lazyevaluator.h"

LazyEvaluator::LazyEvaluator(LazyEvaluator* source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setSourceEvaluator(source);
    this->transform = transform;
}

LazyEvaluator::LazyEvaluator(Eigen::Vector3d* source, Eigen::Transform<double, 3, Eigen::Affine> *transform)
{
    setPrimarySourceVector(source);
    this->transform = transform;
}








} // namespace PointFilter
