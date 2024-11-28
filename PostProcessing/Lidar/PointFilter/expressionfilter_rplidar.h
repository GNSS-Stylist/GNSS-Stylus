#ifndef ExpressionFilter_RPLIDAR_H
#define ExpressionFilter_RPLIDAR_H

#include "expressionfilter_base.h"

namespace PointFilter
{

class ExpressionFilter_RPLIDAR : public ExpressionFilter_Base
{
public:
    ExpressionFilter_RPLIDAR();
    ~ExpressionFilter_RPLIDAR();

    void addPoint(const RPLidarThread::DistanceItem& lidarPoint, const int uptime_ms);
    void initBuffer(void);

private:
    void setCustomVariablesAndFunctions(const QVector<ConvexHullFilter>& newConvexHullFilters = QVector<ConvexHullFilter>());
};

}; // namespace PointFilter

#endif // ExpressionFilter_RPLIDAR_H
