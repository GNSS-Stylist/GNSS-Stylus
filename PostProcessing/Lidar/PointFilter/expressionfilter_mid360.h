#ifndef EXPRESSIONFILTER_MID360_H
#define EXPRESSIONFILTER_MID360_H

#include "expressionfilter_base.h"

namespace PointFilter
{

class ExpressionFilter_Mid360 : public ExpressionFilter_Base
{
public:
    ExpressionFilter_Mid360();
    ~ExpressionFilter_Mid360();

    void addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms);
    void initBuffer(void);

private:
    void setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters = QVector<ConvexHullFilter>());
};

}; // namespace PointFilter

#endif // EXPRESSIONFILTER_MID360_H
