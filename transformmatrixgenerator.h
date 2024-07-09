/*
    transformmatrixgenerator.h (part of GNSS-Stylus)
    Copyright (C) 2020-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef TRANSFORMMATRIXGENERATOR_H
#define TRANSFORMMATRIXGENERATOR_H

#include <QStringList>
#include <QMap>
#include <QVariant>

#include "Eigen/Geometry"

class TransformMatrixGenerator
{
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
            DT_RPLIDAR = 0,
            DT_LIVOX_MID360,
        } Type;

        friend bool operator<(const Device& l, const Device& r) { return std::tie(l.type, l.data) < std::tie(r.type, r.data); };

        Type type = DT_RPLIDAR;
//        QVariant data = int(0);
        quint32 data = 0;
    };

    TransformMatrixGenerator();

//    QList<Issue> warnings;

    QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> > generate(const QStringList& lines);

private:

    Eigen::Transform<double, 3, Eigen::Affine> processCommand(const QVector<Item>& command);
    void processBlockHeader(const QVector<Item>& command, Device& device, QVector<Eigen::Transform<double, 3, Eigen::Affine> >& matrices, QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> >& deviceMatrices);
    QVector<double> convertItemsToDoubles(const QVector<Item>& command, const unsigned int startItem, const unsigned int numOfItems);
    double getAngleMultiplier(const Item& string);
    void checkArgumentCount(const QVector<Item>& command, const int argsNeeded);

    Eigen::Transform<double, 3, Eigen::Affine> cmd_Rotate(const QVector<Item>& command);
    Eigen::Transform<double, 3, Eigen::Affine> cmd_Translate(const QVector<Item>& command);
    Eigen::Transform<double, 3, Eigen::Affine> cmd_Multiply(const QVector<Item>& command);
};

#endif // TRANSFORMMATRIXGENERATOR_H
