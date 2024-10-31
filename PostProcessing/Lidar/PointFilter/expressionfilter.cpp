#include "expressionfilter.h"
#include "PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h"

namespace PointFilter{

ExpressionFilter::ExpressionFilter()
{
    bufferIndex = 0;

    transformCacheIndex_LidarToRig = 0;
    transformCache_LidarToRig[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_RigToNED = 0;
    transformCache_RigToNED[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    customFuncHandler = new TinyExprCustomFuncHandler(TE_DEFAULT, this);

    for (auto item : buffer)
    {
        item.Point_Rig.setSourceEvaluator(&item.Point_Lidar);
        item.Point_Final.setSourceEvaluator(&item.Point_Rig);

        item.Point_Rig.setTransform(&transformCache_LidarToRig[0]);
        item.Point_Final.setTransform(&transformCache_RigToNED[0]);
    }

    std::set<te_variable> customFunctions =
    {
        { "rad_to_deg", rad_to_deg, TE_PURE },
        { "deg_to_rad", deg_to_rad, TE_PURE },

        { "lidar.coord.x", lidar_coord_x, TE_DEFAULT, customFuncHandler },
        { "lidar.coord.y", lidar_coord_y, TE_DEFAULT, customFuncHandler },
        { "lidar.coord.z", lidar_coord_z, TE_DEFAULT, customFuncHandler },

        { "lidar.coord_indexed.x", lidar_coord_indexed_x, TE_DEFAULT, customFuncHandler },
        { "lidar.coord_indexed.y", lidar_coord_indexed_y, TE_DEFAULT, customFuncHandler },
        { "lidar.coord_indexed.z", lidar_coord_indexed_z, TE_DEFAULT, customFuncHandler },

        };

    parser_Filter.set_variables_and_functions(customFunctions);
    parser_Quality.set_variables_and_functions(customFunctions);

    setExpression_Filter("1");
    setExpression_Quality("1");
}

ExpressionFilter::~ExpressionFilter()
{
    delete customFuncHandler;
}

bool ExpressionFilter::setExpression_Filter(const QString newExpression, QString* const errorMessage, int* const errorPosition)
{
    expression_Filter = newExpression;
    parser_Filter.compile(expression_Filter.toUtf8().constData());
    if (parser_Filter.success())
    {
        return true;
    }
    else
    {
        if (errorMessage)
        {
            *errorMessage = QString::fromStdString(parser_Filter.get_last_error_message());
        }
        if (errorPosition)
        {
            *errorPosition = parser_Filter.get_last_error_position();
        }

        return false;
    }
}

bool ExpressionFilter::setExpression_Quality(const QString newExpression, QString* const errorMessage, int* const errorPosition)
{
    expression_Quality = newExpression;
    parser_Quality.compile(expression_Quality.toUtf8().constData());
    if (parser_Quality.success())
    {
        return true;
    }
    else
    {
        if (errorMessage)
        {
            *errorMessage = QString::fromStdString(parser_Quality.get_last_error_message());
        }
        if (errorPosition)
        {
            *errorPosition = parser_Quality.get_last_error_position();
        }

        return false;
    }
}

void ExpressionFilter::addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const int uptime_ms)
{
    buffer[bufferIndex % bufferLength].Point_Lidar_Source = lidarPoint;
    buffer[bufferIndex % bufferLength].uptime_ms = uptime_ms;
    bufferIndex++;

    // TODO: Implement rest
}

bool ExpressionFilter::getFilteredPoint(OutItem& outPoint)
{
    if (bufferIndex < bufferLength)
    {
        outPoint.valid = false;
        return false;
    }

    outPoint.valid = true;
    outPoint.filterResult = parser_Filter.evaluate();

    if (outPoint.filterResult == 1.0)
    {
        outPoint.quality = parser_Quality.evaluate();
    }
    else
    {
        outPoint.quality = 0;
    }

    outPoint.uptime_ms = buffer[(bufferIndex - (bufferLength / 2)) % bufferLength].uptime_ms;
    outPoint.coords = buffer[(bufferIndex - (bufferLength / 2)) % bufferLength].Point_Final.getTransformedVector();

    return true;
}


}; // namespace PointFilter



















