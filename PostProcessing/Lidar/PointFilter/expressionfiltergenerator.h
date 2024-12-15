/*
    expressionfiltergenerator.h (part of GNSS-Stylus)
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

#ifndef EXPRESSIONFILTERGENERATOR_H
#define EXPRESSIONFILTERGENERATOR_H

#include <QStringList>
#include <QMap>
#include "expressionfilter_base.h"

namespace PointFilter
{

class ExpressionFilterGenerator
{
public:
    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    class Device
    {
    public:
        typedef enum
        {
            DT_UNDEFINED = 0,
            DT_RPLIDAR,
            DT_LIVOX_MID360,
        } Type;

        Device() { this->type = DT_UNDEFINED; this->data = 0; };
        Device(const Type type, const quint32 data = 0) { this->type = type; this->data = data; };

        friend bool operator<(const Device& l, const Device& r) { return std::tie(l.type, l.data) < std::tie(r.type, r.data); };

        Type type;
        //        QVariant data = int(0);
        quint32 data;
    };

    static QMap<Device, std::shared_ptr<ExpressionFilter_Base>> generateMap(const QString& plainText, const QVector<ExpressionFilter_Base::ConvexHullFilter>& convexHullFilters = QVector<ExpressionFilter_Base::ConvexHullFilter>());
};

}; // namespace PointFilter


#endif // EXPRESSIONFILTERGENERATOR_H
