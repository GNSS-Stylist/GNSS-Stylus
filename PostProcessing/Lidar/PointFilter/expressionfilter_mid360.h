/*
    expressionfilter_mid360.h (part of GNSS-Stylus)
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

#ifndef EXPRESSIONFILTER_MID360_H
#define EXPRESSIONFILTER_MID360_H

#include "expressionfilter_base.h"

namespace PointFilter
{

class ExpressionFilter_Mid360 : public ExpressionFilter_Base
{
public:
    ExpressionFilter_Mid360();
    ExpressionFilter_Mid360(const ExpressionFilter_Mid360& source);
    ~ExpressionFilter_Mid360();

    ExpressionFilter_Mid360 operator=(const ExpressionFilter_Mid360& source);

    void addPoint(const LivoxMid360::PointCloudData::Point& lidarPoint, const qint64 timestamp);
    virtual void initBuffer(void);

private:
    // clang nags "Call to virtual method 'ExpressionFilter_Mid360::initBuffer' during construction bypasses virtual dispatch"
    // when calling virtual initBuffer from constructor, so using a separate private function to perform the real work.
    void initBuffer_Local(void);
    void setCustomVariablesAndFunctions(void);
};

}; // namespace PointFilter

#endif // EXPRESSIONFILTER_MID360_H
