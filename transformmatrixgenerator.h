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

    TransformMatrixGenerator();

    QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> > generateMap(const QStringList& lines, const Device& defaultDevice, const bool requireDeviceDefinition = false, const bool singleDevice = false);
    Eigen::Transform<double, 3, Eigen::Affine> generateSingle(const QStringList& lines);

private:

    class State
    {
    public:
        bool requireDeviceDefinition = false;
        bool singleDevice = false;
        Device currentDevice;
        bool deviceDefined = false;
        QVector<Item> command;
        QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> > deviceMatrices;
        QVector<Eigen::Transform<double, 3, Eigen::Affine> > subMatrices;
    };

    Eigen::Transform<double, 3, Eigen::Affine> processCommand(State& state);
    void processBlockHeader(State& state);
    QVector<double> convertItemsToDoubles(const QVector<Item>& command, const unsigned int startItem, const unsigned int numOfItems);
    double getAngleMultiplier(const Item& string);
    void checkArgumentCount(const QVector<Item>& command, const int argsNeeded);

    Eigen::Transform<double, 3, Eigen::Affine> cmd_Rotate(const QVector<Item>& command);
    Eigen::Transform<double, 3, Eigen::Affine> cmd_Translate(const QVector<Item>& command);
    Eigen::Transform<double, 3, Eigen::Affine> cmd_Multiply(const QVector<Item>& command);
};

#endif // TRANSFORMMATRIXGENERATOR_H
