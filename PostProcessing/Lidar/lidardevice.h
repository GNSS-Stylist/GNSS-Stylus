/*
    lidardevice.h (part of GNSS-Stylus)
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

#ifndef LIDARDEVICE_H
#define LIDARDEVICE_H

#include "qglobal.h"

class LidarDevice
{
public:
    typedef enum
    {
        DT_UNDEFINED = 0,
        DT_RPLIDAR,
        DT_LIVOX_MID360,
    } Type;

    LidarDevice() { this->type = DT_UNDEFINED; this->data = 0; };
    LidarDevice(const Type type, const quint32 data = 0) { this->type = type; this->data = data; };

    friend bool operator<(const LidarDevice& l, const LidarDevice& r) { return std::tie(l.type, l.data) < std::tie(r.type, r.data); };

    Type type;
    //        QVariant data = int(0);
    quint32 data;
};

#endif // LIDARDEVICE_H
