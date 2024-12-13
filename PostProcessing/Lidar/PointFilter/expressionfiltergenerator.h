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
#include "Eigen/Geometry"
#include "expressionfilter_base.h"

namespace PointFilter
{

class ExpressionFilterGenerator
{
    using FilterPair = std::pair<std::shared_ptr<ExpressionFilter_Base>, std::shared_ptr<ExpressionFilter_Base> >;

public:
    class Item
    {
    public:
        int lineNumber = -1;
        int firstCol = -1;
        int lastCol = -1;
        QByteArray text;

        Item(const QByteArray& text, const int lineNumber = -1, const int firstCol = -1, const int lastCol = -1);
        Item() {};
    };

    class Issue
    {
    public:
        Item item;
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

    ExpressionFilterGenerator();

//    QMap<Device, std::pair<std::shared_ptr<ExpressionFilter_Base>, std::shared_ptr<ExpressionFilter_Base> > > generateMap(const QStringList& lines, const QVector<ExpressionFilter_Base::ConvexHullFilter>& convexHullFilters);
    QMap<Device, FilterPair> generateMap(const QStringList& lines, const QVector<ExpressionFilter_Base::ConvexHullFilter>& convexHullFilters);

private:
    class State
    {
    public:
        Device currentDevice;
        bool deviceDefined = false;
        QVector<Item> command;
    };

    void processBlockHeader(State& state);
    bool skipComments(const QStringList& lines, int& lineNum, int& column);
    bool skipWhitespaces(const QStringList& lines, int& lineNum, int& column);
    bool skipWhitespacesAndComments(const QStringList& lines, int& lineNum, int& column);
    QString getExpressionString(const QStringList& lines, int& lineNum, int& column, QMap<int, std::pair<int, int> >& charMap);
};

}; // namespace PointFilter


#endif // EXPRESSIONFILTERGENERATOR_H
