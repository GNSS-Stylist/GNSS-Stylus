/*
    transformmatrixgenerator.cpp (part of GNSS-Stylus)
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

#include "transformmatrixgenerator.h"

#include <QHostAddress>


#if 0
#define TRANSFORMMATRIXGENERATORDEBUGOUTPUT
#include <QtDebug>

inline QDebug operator<<(QDebug dbg, const Eigen::Matrix3d& m)
{
    dbg << QString::number(m(0,0),'f',3) << ", " << QString::number(m(0,1),'f',3) << ", " << QString::number(m(0,2),'f',3) << "\n" <<
           QString::number(m(1,0),'f',3) << ", " << QString::number(m(1,1),'f',3) << ", " << QString::number(m(1,2),'f',3) << "\n" <<
           QString::number(m(2,0),'f',3) << ", " << QString::number(m(2,1),'f',3) << ", " << QString::number(m(2,2),'f',3);
    return dbg.space();
}

inline QDebug operator<<(QDebug dbg, const Eigen::Transform<double, 3, Eigen::Affine> t)
{
    Eigen::Matrix4d m = t.matrix();

    dbg << QString::number(m(0,0),'f',3) << ", " << QString::number(m(0,1),'f',3) << ", " << QString::number(m(0,2),'f',3) << ", " << QString::number(m(0,3),'f',3) << "\n" <<
           QString::number(m(1,0),'f',3) << ", " << QString::number(m(1,1),'f',3) << ", " << QString::number(m(1,2),'f',3) << ", " << QString::number(m(1,3),'f',3) << "\n" <<
           QString::number(m(2,0),'f',3) << ", " << QString::number(m(2,1),'f',3) << ", " << QString::number(m(2,2),'f',3) << ", " << QString::number(m(2,3),'f',3) << "\n" <<
           QString::number(m(3,0),'f',3) << ", " << QString::number(m(3,1),'f',3) << ", " << QString::number(m(3,2),'f',3) << ", " << QString::number(m(3,3),'f',3);
    return dbg.space();
}
#endif

TransformMatrixGenerator::TransformMatrixGenerator()
{

}

TransformMatrixGenerator::Item::Item(const QByteArray& text, const int lineNumber, const int firstCol, const int lastCol)
{
    this->text = text;
    this->lineNumber =lineNumber;
    this->firstCol = firstCol;
    this->lastCol = lastCol;
};

Eigen::Transform<double, 3, Eigen::Affine> TransformMatrixGenerator::generateSingle(const QStringList& lines)
{
    Device dummyDevice;
    dummyDevice.type = Device::DT_UNDEFINED;

    auto map = generateMap(lines, dummyDevice, false, true);

    Q_ASSERT(map.size() == 1);
    Q_ASSERT(map.contains(dummyDevice));

    return map.value(dummyDevice);
}


QMap<TransformMatrixGenerator::Device, Eigen::Transform<double, 3, Eigen::Affine> > TransformMatrixGenerator::generateMap(const QStringList &lines, const Device& defaultDevice, const bool requireDeviceDefinition, const bool singleDevice)
{
    State state;

    state.requireDeviceDefinition = requireDeviceDefinition;
    state.singleDevice = singleDevice;
    state.currentDevice = defaultDevice;

    int lineNumber = 0;

    QByteArray subString;
    int firstCol = -1;

    while (lineNumber < lines.count())
    {
        QString line = lines[lineNumber];

        int i = 0;

//        int endCol = -1;

        while (i < line.length())
        {
            char character = line.at(i).toLatin1();

            if (!character)
            {
                Issue error;
                error.text = "Only Latin-1 (\"8-bit ascii\") characters allowed in non-comment sections.";
                error.item.lineNumber = lineNumber;
                error.item.firstCol = i;
                error.item.lastCol = i;
                throw error;
            }
            else if ((line.length() >= (i + 2)) && (character == '/') && (line.at(i+1).toLatin1() == '/'))
            {
                // Rest of the line is comment -> Skip it
                break;
            }
            else if ((character == ' ') || (character == '\t'))
            {
                // Command/argument separator
                if (!subString.isEmpty())
                {
                    Item newItem;
                    newItem.text = subString;
                    newItem.lineNumber = lineNumber;
                    newItem.firstCol = firstCol;
                    newItem.lastCol = i - 1;
                    state.command.push_back(newItem);
                    subString.clear();
                }
            }
            else if (character == ';')
            {
                if (!subString.isEmpty())
                {
                    Item newItem;
                    newItem.text = subString;
                    newItem.lineNumber = lineNumber;
                    newItem.firstCol = firstCol;
                    newItem.lastCol = i - 1;
                    state.command.push_back(newItem);
                    subString.clear();
                }

                if (state.command.size() == 0)
                {
                    break;
                }

                if (state.requireDeviceDefinition && (!state.deviceDefined))
                {
                    Issue error;
                    error.text = "Device not defined before command \"" + state.command.at(0).text + "\".";
                    error.item = state.command.at(0);
                    throw error;
                }

                state.subMatrices.push_back(processCommand(state));
                state.command.clear();
            }
            else if (character == ':')
            {
                if (!subString.isEmpty())
                {
                    Item newItem;
                    newItem.text = subString;
                    newItem.lineNumber = lineNumber;
                    newItem.firstCol = firstCol;
                    newItem.lastCol = i - 1;
                    state.command.push_back(newItem);
                    subString.clear();
                }

                if (state.command.size() == 0)
                {
                    Issue error;
                    error.text = "Block type identifier \"" + QString(character) + "\" without definition.";
                    error.item.text = ":";
                    error.item.lineNumber = lineNumber;
                    error.item.firstCol = firstCol;
                    error.item.lastCol = i - 1;
                    throw error;
                }

                processBlockHeader(state);
                state.command.clear();
            }
            else
            {
                // Non-control character -> add it to the buffer

                if (subString.length() == 0)
                {
                    firstCol = i;
                }
                subString += character;
            }

            i++;
        }

        if (!subString.isEmpty())
        {
            Item newItem;
            newItem.text = subString;
            newItem.lineNumber = lineNumber;
            newItem.firstCol = firstCol;
            newItem.lastCol = i - 1;
            state.command.push_back(newItem);
            subString.clear();
        }

        lineNumber++;
    }

    if (!state.command.isEmpty())
    {
        Issue error;
        error.text = "Unterminated command in the end.";
        error.item = state.command.at(0);
        throw error;
    }

    Eigen::Transform<double, 3, Eigen::Affine> matrix;

    auto iter = state.deviceMatrices.find(state.currentDevice);

    if (iter != state.deviceMatrices.end())
    {
        matrix = iter.value();
    }
    else
    {
        matrix = matrix.Identity();
    }

    for (int i = state.subMatrices.size() - 1; i >= 0; i--)
    {
        matrix = matrix * state.subMatrices.at(i);
    }

    state.deviceMatrices.insert(state.currentDevice, matrix);

#ifdef TRANSFORMMATRIXGENERATORDEBUGOUTPUT
    qDebug() << "Final transform matrix generated:";
    qDebug() << matrix;
#endif

    return state.deviceMatrices;
}

Eigen::Transform<double, 3, Eigen::Affine> TransformMatrixGenerator::processCommand(State& state)
{
    QByteArray cmd0 = state.command.at(0).text.toLower();

    Eigen::Transform<double, 3, Eigen::Affine> matrix = matrix.Identity();

    if (cmd0 == "rotate")
    {
        matrix = cmd_Rotate(state.command);
    }
    else if (cmd0 == "translate")
    {
        matrix = cmd_Translate(state.command);
    }
    else if (cmd0 == "multiply")
    {
        matrix = cmd_Multiply(state.command);
    }
    else
    {
        Issue error;
        error.text = "Unknown command \"" + state.command.at(0).text + "\"";
        error.item = state.command.at(0);
        throw error;
    }

#ifdef TRANSFORMMATRIXGENERATORDEBUGOUTPUT
    qDebug() << "Transform matrix generated by command \"" << command.at(0).text << "\" at line " +
                QString::number(command.at(0).lineNumber);
    qDebug() << matrix;
#endif

    return matrix;
}

QVector<double> TransformMatrixGenerator::convertItemsToDoubles(const QVector<Item>& command, const unsigned int startItem, const unsigned int numOfItems)
{
// Tautologous (and keep other code that way!)    checkArgumentCount(command, startItem + numOfItems - 1);

    QVector<double> retVals;

    for (unsigned int i = 0; i < numOfItems; i++)
    {
        bool convOk;
        Item arg = command.at(startItem + i);
        double val = arg.text.toDouble(&convOk);
        if (!convOk)
        {
            Issue error;
            error.text = "Can not convert argument to double (real).";
            error.item = arg;
            throw error;
        }

        retVals.push_back(val);
    }

    return retVals;
}

double TransformMatrixGenerator::getAngleMultiplier(const Item& item)
{
    if (item.text.toLower() == "rad")
    {
        return 1;
    }
    else if (item.text.toLower() == "deg")
    {
        return 2 * M_PI / 360;
    }
    else
    {
        Issue error;
        error.text = "Invalid angle unit.";
        error.item = item;
        throw error;
    }
}

void TransformMatrixGenerator::checkArgumentCount(const QVector<Item>& command, const int argsNeeded)
{
    int argNum = command.count() - 1;

    if (argNum < argsNeeded)
    {
        Issue error;
        error.text = "Not enough arguments. Required " + QString::number(argsNeeded) + ", got " + QString::number(argNum);
        error.item = command.at(0);
        throw error;
    }
    else if (argNum > argsNeeded)
    {
        Issue error;
        error.text = "Too many arguments. Required " + QString::number(argsNeeded) + ", got " + QString::number(argNum);
        error.item = command.at(argsNeeded + 1);
        throw error;
    }
}

Eigen::Transform<double, 3, Eigen::Affine> TransformMatrixGenerator::cmd_Rotate(const QVector<Item>& command)
{
    Eigen::Transform<double, 3, Eigen::Affine> matrix = matrix.Identity();

    checkArgumentCount(command, 3);

    Eigen::Vector3d rotationAxis;

    QByteArray axisName = command.at(1).text.toLower();
    if ((axisName == "n") || (axisName == "x") || (axisName == "roll"))
    {
        rotationAxis = Eigen::Vector3d::UnitX();
    }
    else if ((axisName == "e") || (axisName == "y") || (axisName == "pitch"))
    {
        rotationAxis = Eigen::Vector3d::UnitY();
    }
    else if ((axisName == "d") || (axisName == "z") || (axisName == "yaw") || (axisName == "heading"))
    {
        rotationAxis = Eigen::Vector3d::UnitZ();
    }
    else
    {
        Issue error;
        error.text = "Unknown axis \"" + command.at(1).text + "\"";
        error.item = command.at(1);
        throw error;
    }

    QVector<double> angle = convertItemsToDoubles(command, 2, 1);

    double angleMultiplier = getAngleMultiplier(command.at(3));

    matrix.rotate(Eigen::AngleAxisd(angle.at(0) * angleMultiplier, rotationAxis));

    return matrix;
}

Eigen::Transform<double, 3, Eigen::Affine> TransformMatrixGenerator::cmd_Translate(const QVector<Item>& command)
{
    Eigen::Transform<double, 3, Eigen::Affine> matrix = matrix.Identity();

    checkArgumentCount(command, 3);

    QVector<double> translationValues = convertItemsToDoubles(command, 1, 3);

    Eigen::Vector3d translation(translationValues.at(0), translationValues.at(1), translationValues.at(2));

    matrix.translate(translation);

    return matrix;
}

Eigen::Transform<double, 3, Eigen::Affine> TransformMatrixGenerator::cmd_Multiply(const QVector<Item>& command)
{
    Eigen::Transform<double, 3, Eigen::Affine> matrix = matrix.Identity();

    checkArgumentCount(command, 12);

    QVector<double> translationValues = convertItemsToDoubles(command, 1, 12);

    matrix.matrix() << translationValues.at(0), translationValues.at(1), translationValues.at(2), translationValues.at(3),
            translationValues.at(4), translationValues.at(5), translationValues.at(6), translationValues.at(7),
            translationValues.at(8), translationValues.at(9), translationValues.at(10), translationValues.at(11),
            0, 0, 0, 1;

    return matrix;
}

void TransformMatrixGenerator::processBlockHeader(State& state)
{
    QByteArray cmd = state.command.at(0).text.toLower();
    if (cmd == "device")
    {
        if (state.singleDevice)
        {
            Issue error;
            error.text = "Device definition not supported in this context.";
            error.item = state.command.at(0);
            throw error;
        }

        if (state.command.size() < 2)
        {
            Issue error;
            error.text = "Device type not defined.";
            error.item = state.command.at(0);
            throw error;
        }

        QByteArray deviceType = state.command.at(1).text.toLower();

        QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> >::iterator iter;

//        Device device;

        if (deviceType == "rplidar")
        {
            if (state.command.size() > 2)
            {
                Issue error;
                error.text = "No extra parameters allowed for device type \"" + state.command.at(1).text + "\" (only one active RPLidar device supported currently).";
                error.item = state.command.at(1);
                throw error;
            }

            state.currentDevice.type = Device::DT_RPLIDAR;
            state.currentDevice.data = 0;
            //device.data.clear();    // QVariant-version

            iter = state.deviceMatrices.find(state.currentDevice);
        }
        else if (deviceType == "mid360")
        {
            if (state.command.size() < 3)
            {
                Issue error;
                error.text = "No IP address defined for device type \"" + state.command.at(1).text + "\".";
                error.item = state.command.at(1);
                throw error;
            }
            if (state.command.size() > 3)
            {
                Issue error;
                error.text = "too many parameters for device type \"" + state.command.at(1).text + "\". Only numeric IPv4 address allowed.";
                error.item = state.command.at(3);
                throw error;
            }

            QHostAddress ipAddressNotValidated;

            if (!ipAddressNotValidated.setAddress(QString(state.command.at(2).text)))
            {
                Issue error;
                error.text = "Can't convert parameter \"" + QString(state.command.at(2).text) + "\" to IPv4 address. ";
                error.item = state.command.at(2);
                throw error;
            }

            bool convOk = false;
            quint32 hostAddress = ipAddressNotValidated.toIPv4Address(&convOk);

            if (!convOk)
            {
                Issue error;
                error.text = "Parameter \"" + QString(state.command.at(2).text) + "\" is not a valid IPv4 address. ";
                error.item = state.command.at(2);
                throw error;
            }

            state.currentDevice.type = Device::DT_LIVOX_MID360;
            state.currentDevice.data = hostAddress;

            iter = state.deviceMatrices.find(state.currentDevice);
        }
        else
        {
            Issue error;
            error.text = "Unknown device type \"" + state.command.at(1).text + "\".";
            error.item = state.command.at(1);
            throw error;
        }

        if ((state.deviceDefined) || (!state.requireDeviceDefinition))
        {
            Eigen::Transform<double, 3, Eigen::Affine> matrix;

            if (iter != state.deviceMatrices.end())
            {
                matrix = iter.value();
            }
            else
            {
                matrix = matrix.Identity();
            }

            for (int i = state.subMatrices.size() - 1; i >= 0; i--)
            {
                matrix = matrix * state.subMatrices.at(i);
            }

            state.deviceMatrices.insert(state.currentDevice, matrix);
        }

        state.subMatrices.clear();
        state.deviceDefined = true;
    }
    else
    {
        Issue error;
        error.text = "Unknown block type \"" + state.command.at(0).text + "\"";
        error.item = state.command.at(0);
        throw error;
    }
}

