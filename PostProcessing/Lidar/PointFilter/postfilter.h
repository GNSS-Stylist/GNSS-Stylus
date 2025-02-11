/*
    postfilter.h (part of GNSS-Stylus)
    Copyright (C) 2025-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef POSTFILTER_H
#define POSTFILTER_H

#include "Eigen/Geometry"
#include "qglobal.h"

namespace PointFilter
{

class PostFilter
{
public:
    struct Params
    {
        double minDistDiff = -1;
        qint64 minTimeDiff = -1;
    };

    inline PostFilter(const Params& params);
    inline void init(const Params& params);
    inline void reinit(void);
    inline bool filter(const Eigen::Vector3d &newVec, const qint64 newTime);
    inline bool checkBack(const Eigen::Vector3d &oldVec, const qint64 oldTime);

private:
    double minDistDiff_Squared;
    qint64 minTimeDiff;

    bool first;
    Eigen::Vector3d lastValidVector;
    qint64 lastValidTime;
};

inline PostFilter::PostFilter(const Params& params)
{
    init(params);
}

inline void PostFilter::init(const Params& params)
{
    this->minDistDiff_Squared = params.minDistDiff * params.minDistDiff;
    this->minTimeDiff = params.minTimeDiff;
    this->first = true;
}

inline void PostFilter::reinit(void)
{
    this->first = true;
}

inline bool PostFilter::filter(const Eigen::Vector3d& newVec, const qint64 newTime)
{
    if (first)
    {
        lastValidVector = newVec;
        lastValidTime = newTime;
        first = false;
        return true;
    }

    if ((std::abs(newTime - lastValidTime) < minTimeDiff) &&
        (((newVec - lastValidVector).squaredNorm()) < minDistDiff_Squared))
    {
        return false;
    }

    lastValidVector = newVec;
    lastValidTime = newTime;

    return true;
}

inline bool PostFilter::checkBack(const Eigen::Vector3d& oldVec, const qint64 oldTime)
{
    if (!first && ((((oldVec - lastValidVector).squaredNorm()) >= minDistDiff_Squared) ||
                   (std::abs(oldTime - lastValidTime) >= minTimeDiff)))
    {
        return true;
    }

    return false;
}


} // namespace

#endif // POSTFILTER_H
