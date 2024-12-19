/*
    expressionfilter_rplidar.h (part of GNSS-Stylus)
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

#ifndef EXPRESSIONFILTER_RPLIDAR_H
#define EXPRESSIONFILTER_RPLIDAR_H

#include "expressionfilter_base.h"
#include "../RPLidar/rplidarthread.h"

namespace PointFilter
{

class ExpressionFilter_RPLidar : public ExpressionFilter_Base
{
public:
    ExpressionFilter_RPLidar();
    ExpressionFilter_RPLidar(const ExpressionFilter_RPLidar& source);
    ~ExpressionFilter_RPLidar();

    ExpressionFilter_RPLidar operator =(const ExpressionFilter_RPLidar& source);

    void addPoint(const RPLidarThread::DistanceItem& lidarPoint, const int uptime_ms);
    virtual void initBuffer(void);

private:
    void setCustomVariablesAndFunctions(void);
};

}; // namespace PointFilter

#endif // EXPRESSIONFILTER_RPLIDAR_H
