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

#include "expressionfiltergenerator.h"
#include "expressionfilter_mid360.h"
#include "expressionfilter_rplidar.h"

namespace PointFilter
{


ExpressionFilterGenerator::ExpressionFilterGenerator()
{
}

QMap<ExpressionFilterGenerator::Device, ExpressionFilterGenerator::FilterPair> ExpressionFilterGenerator::generateMap(const QStringList& lines, const QVector<ExpressionFilter_Base::ConvexHullFilter>& convexHullFilters)
{
    QMap<Device, std::pair<std::shared_ptr<ExpressionFilter_Base>, std::shared_ptr<ExpressionFilter_Base> > > filters;

    State state;

    state.currentDevice.type = Device::DT_UNDEFINED;

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



















    std::shared_ptr<ExpressionFilter_Mid360> filtsu;
    std::shared_ptr<ExpressionFilter_Mid360> laatu;

    Device dev;

//    filters.insert(dev, dynamic_cast<std::unique_ptr<ExpressionFilter_Base> > (std::move(filtsu)));
//    filters.insert(dev, std::unique_ptr<ExpressionFilter_Base>(std::move(filtsu)));
    filters.insert(dev, std::pair<std::shared_ptr<ExpressionFilter_Base>, std::shared_ptr<ExpressionFilter_Base> > (filtsu, laatu));
    filters.insert(dev, FilterPair(filtsu, laatu));

    return filters;
}

void ExpressionFilterGenerator::processBlockHeader(State& state)
{
    QByteArray cmd = state.command.at(0).text.toLower();
    if (cmd == "device")
    {
        if (state.command.size() < 2)
        {
            Issue error;
            error.text = "Device type not defined.";
            error.item = state.command.at(0);
            throw error;
        }

        QByteArray deviceType = state.command.at(1).text.toLower();

//        QMap<Device, Eigen::Transform<double, 3, Eigen::Affine> >::iterator iter;

//        Device prevDevice = state.currentDevice;

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

//            iter = state.deviceMatrices.find(state.currentDevice);
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

//            iter = state.deviceMatrices.find(state.currentDevice);
        }
        else
        {
            Issue error;
            error.text = "Unknown device type \"" + state.command.at(1).text + "\".";
            error.item = state.command.at(1);
            throw error;
        }
/*
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

            state.deviceMatrices.insert(prevDevice, matrix);
        }

        state.subMatrices.clear();
*/

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

// Skips a comment starting from the defined position (lineNum, column), if any
// Doesn't skip consecutive comments in one call
// lineNum / column will point to the next character after the comment if there was a comment, otherwise they are unchanged
// @return true if comment was skipped.
bool ExpressionFilterGenerator::skipComments(const QStringList& lines, int& lineNum, int& column)
{
    if (column >= lines.at(lineNum).length())
    {
        return false;
//        lineNum++;
//        column = 0;
    }

    if (lineNum >= lines.count())
    {
        return false;
    }

    if (lines[lineNum].length() == 0)
    {
        // Empty line
        return false;
    }

    QString line = lines.at(lineNum);

    if ((line.length() >= (column + 2)) && (line.at(column).toLatin1() == '/') && (line.at(column+1).toLatin1() == '/'))
    {
        // Rest of the line is comment -> Skip it
        column = 0;
        lineNum++;
        return true;
    }
    else if ((line.length() >= (column + 2)) && (line.at(column).toLatin1() == '/') && (line.at(column+1).toLatin1() == '*'))
    {
        // Block comment start.
        int startLine = lineNum;
        int startColumn = column;

        column += 2;

        while (lineNum < lines.count())
        {
            while (column < line.length())
            {
                if ((line.length() >= (column + 2)) && (line.at(column).toLatin1() == '*') && (line.at(column+1).toLatin1() == '/'))
                {
                    // Comment end
                    column += 2;
                    if (column >= line.length())
                    {
                        column = 0;
                        lineNum++;
                    }
                    return true;
                }
                column++;
            }
            lineNum++;
        }

        Issue error;
        error.text = "Unterminated block comment.";
        error.item.lineNumber = startLine;
        error.item.firstCol = startColumn;
        error.item.lastCol = startColumn + 2;
        error.item.text = "/*";

        throw error;
    }

    return false;
}

// Skips whitespaces (including newlines) starting from the defined position (lineNum, column), if any
// lineNum / column will point to the next character after the comment if there were whitespace(s), otherwise they are unchanged
// @return true if something was skipped.
bool ExpressionFilterGenerator::skipWhitespaces(const QStringList& lines, int& lineNum, int& column)
{
    if (column >= lines.at(lineNum).length())
    {
        lineNum++;
        column = 0;
    }

    bool retval = false;

    while (lineNum < lines.count())
    {
        // Skip empty lines

        if (lines[lineNum].length() == 0)
        {
            column = 0;
            lineNum++;
            retval = true;
            continue;
        }

        char character = lines[lineNum].at(column).toLatin1();

        if ((character != ' ') && (character != '\t'))
        {
            retval = true;
            break;
        }

        column++;

        if (column >= lines[lineNum].length())
        {
            column = 0;
            lineNum++;
        }
    }

    return retval;
}

bool ExpressionFilterGenerator::skipWhitespacesAndComments(const QStringList& lines, int& lineNum, int& column)
{
    bool skipped;

    do
    {
        skipped = skipWhitespaces(lines, lineNum, column);
        skipped |= skipComments(lines, lineNum, column);
    } while(skipped);

    return skipped;
}

QString ExpressionFilterGenerator::getExpressionString(const QStringList& lines, int& lineNum, int& column, QMap<int, std::pair<int, int> >& charMap)
{
    charMap.clear();

    if (column >= lines.at(lineNum).length())
    {
        lineNum++;
        column = 0;
    }

    if (lineNum >= lines.count())
    {
        return "";
    }

    int startLine = lineNum;
    int startColumn = column;

    skipWhitespacesAndComments(lines, lineNum, column);

    if (lineNum >= lines.count())
    {
        Issue error;
        error.text = "No opening curly brace found for expression (filter or quality).";
        error.item.lineNumber = startLine;
        error.item.firstCol = startColumn;
        error.item.lastCol = startColumn;

        throw error;
    }

    if (lines[lineNum].at(column) != '{')
    {
        Issue error;
        error.text = "Only whitespaces or comments allowed before opening curly brace for the expression (filtering or quality).";
        error.item.lineNumber = startLine;
        error.item.firstCol = startColumn;
        error.item.lastCol = startColumn;

        throw error;
    }

    startLine = lineNum;
    startColumn = column;

    QByteArray retval;

    column++;

    while (lineNum < lines.count())
    {
        while (column < lines[lineNum].length())
        {
            if (skipComments(lines, lineNum, column))
            {
                continue;
            }

            char character = lines[lineNum].at(column).toLatin1();

            if (!character)
            {
                // Comment sections may contain non-latin chars, so replace them with spaces
                character = ' ';
            }

            if (character == '}')
            {
                return retval;
            }

            retval += character;
            charMap[retval.length() - 1] = std::pair<int, int>(lineNum, column);

            column++;
        }

        retval += 10;   // Not sure how tinyexpr handles/counts newlines, so add a single linefeed
        charMap[retval.length() - 1] = std::pair<int, int>(lineNum, column);

        column = 0;
        lineNum++;
    }

    Issue error;
    error.text = "Unmatched opening curly brace.";
    error.item.lineNumber = startLine;
    error.item.firstCol = startColumn;
    error.item.lastCol = startColumn;

    throw error;
}

}; // namespace PointFilter




























