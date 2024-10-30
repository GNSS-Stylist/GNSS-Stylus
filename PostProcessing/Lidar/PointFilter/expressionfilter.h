#ifndef EXPRESSIONFILTER_H
#define EXPRESSIONFILTER_H

#include <QString>
#include <Eigen/Geometry>
#include "lazyevaluator.h"
#include "tinyexpr-plusplus/tinyexpr.h"
#include "livoxmid360pointcloudandimudata.h"

namespace PointFilter
{

class ExpressionFilter;

class TinyExprCustomFuncHandler : public te_expr
{
public:
    explicit TinyExprCustomFuncHandler(const te_variable_flags type, ExpressionFilter* const filter) noexcept :
        te_expr(type) { this->filter = filter; }

    inline te_type lidar_coord_x() const;
    inline te_type lidar_coord_x_indexed(te_type a) const;
    inline te_type lidar_coord_y() const;

private:
    ExpressionFilter* filter;
};

class ExpressionFilter
{
public:
    ExpressionFilter();
    ~ExpressionFilter();

    static const unsigned int bufferLength = 32;

    class OutItem
    {
    public:
        bool valid;
        double filterResult;
        int uptime_ms;
        Eigen::Vector3d coords;
        double quality;
    };

    bool setExpression_Filter(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);
    bool setExpression_Quality(const QString newExpression, QString* const errorMessage = nullptr, int* const errorPosition = nullptr);

    void addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms);
    bool getFilteredPoint(OutItem& outPoint);
    void setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);
    void setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform);

private:
    class BufferItem
    {
    public:
        LivoxMid360::PointCloudData::Point Point_Lidar_Source;
        LazyEvaluator Point_Lidar;
        LazyEvaluator Point_Rig;
        LazyEvaluator Point_Final;
        int uptime_ms;
    };

    QString expression_Filter;
    te_parser parser_Filter;

    QString expression_Quality;
    te_parser parser_Quality;

    BufferItem buffer[bufferLength];
    unsigned int bufferIndex;

    Eigen::Transform<double, 3, Eigen::Affine> transformCache_LidarToRig[256];
    unsigned char transformCacheIndex_LidarToRig;

    Eigen::Transform<double, 3, Eigen::Affine> transformCache_RigToNED[256];
    unsigned char transformCacheIndex_RigToNED;

    TinyExprCustomFuncHandler* customFuncHandler;

    friend class TinyExprCustomFuncHandler;
};

inline te_type TinyExprCustomFuncHandler::lidar_coord_x() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].Point_Lidar_Source.x;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_y() const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1) % filter->bufferLength].Point_Lidar_Source.y;
}

inline te_type TinyExprCustomFuncHandler::lidar_coord_x_indexed(te_type a) const
{
    Q_ASSERT(filter->bufferIndex >= filter->bufferLength);
    return filter->buffer[(filter->bufferIndex - (filter->bufferLength / 2) - 1 + int(a)) % filter->bufferLength].Point_Lidar_Source.x;
}


}; // namespace PointFilter
#endif // EXPRESSIONFILTER_H
