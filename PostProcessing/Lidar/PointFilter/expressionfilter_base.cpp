/*
    expressionfilter.cpp (part of GNSS-Stylus)
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

#include "expressionfilter_base.h"

namespace PointFilter
{

ExpressionFilter_Base::ExpressionFilter_Base()
{
//    initBuffer();

    transformCacheIndex_LidarToRig = 0;
    transformCache_LidarToRig[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    transformCacheIndex_RigToNED = 0;
    transformCache_RigToNED[0] = Eigen::Transform<double, 3, Eigen::Affine>::Identity();

    customFuncHandler = new TinyExprCustomFuncHandler(TE_DEFAULT, this);

    convexHullFilterIndexes = nullptr;
//    setCustomVariablesAndFunctions();

    setExpression_Filter("1");
    setExpression_Quality("1");
}

ExpressionFilter_Base::~ExpressionFilter_Base()
{
    delete customFuncHandler;
}


bool ExpressionFilter_Base::setExpression_Filter(const QString newExpression, QString* const errorMessage, int* const errorPosition)
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

bool ExpressionFilter_Base::setExpression_Quality(const QString newExpression, QString* const errorMessage, int* const errorPosition)
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

void ExpressionFilter_Base::setTransform_LidarToRig(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_LidarToRig++;
    transformCache_LidarToRig[transformCacheIndex_LidarToRig] = newTransform;
}

void ExpressionFilter_Base::setTransform_RigToNED(const Eigen::Transform<double, 3, Eigen::Affine>& newTransform)
{
    transformCacheIndex_RigToNED++;
    transformCache_RigToNED[transformCacheIndex_RigToNED] = newTransform;
}

bool ExpressionFilter_Base::setConvexHullFilters(const QVector<ConvexHullFilter>& newConvexHullFilters)
{
    try
    {
        setCustomVariablesAndFunctions(newConvexHullFilters);
    }
    catch (...)
    {
        // tinyexpr++ set_variables_and_functions-function may thow an expection if there's unsupported characters
        // in the function names etc. Not making any deeper error checking here (should be done on a higher level).
        return false;
    }

    return true;
}

bool ExpressionFilter_Base::getFilteredPoint(OutItem& outPoint)
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

    outPoint.uptime_ms = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].uptime_ms;
    outPoint.coords = buffer[(bufferIndex - (bufferLength / 2) - 1) % bufferLength].point_NED.getTransformedVector();

    return true;
}




}; // namespace PointFilter



















