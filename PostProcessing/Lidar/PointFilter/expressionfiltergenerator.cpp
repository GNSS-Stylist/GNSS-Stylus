/*
    expressionfiltergenerator.cpp (part of GNSS-Stylus)
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
#include "Util/textblockparser.h"

namespace PointFilter
{

QMap<ExpressionFilterGenerator::Device, std::shared_ptr<ExpressionFilter_Base>> ExpressionFilterGenerator::generateMap(const QString& plainText, const QVector<ExpressionFilter_Base::ConvexHullFilter>& convexHullFilters)
{
    QMap<Device, std::shared_ptr<ExpressionFilter_Base> > filters;

    int charIndex = 0;

    QByteArray subString;

    int plainTextLength = plainText.length();

    while (charIndex < plainTextLength)
    {
        TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

        if (charIndex >= plainTextLength)
        {
            break;
        }

        int deviceStringStartIndex = charIndex;
        QString deviceTypeString = TextBlockParser::getSubString(plainText, charIndex, " \t\n{").toLower();
        int deviceDefinitionEndIndex = charIndex;
        QString deviceStringForErrors = deviceTypeString;

        Device device;
        std::shared_ptr<ExpressionFilter_Base> newFilterPair;

        if (deviceTypeString == "rplidar")
        {
            device.type = Device::DT_RPLIDAR;
            newFilterPair = std::make_shared<ExpressionFilter_RPLidar>();
        }
        else if (deviceTypeString == "mid360")
        {
            device.type = Device::DT_LIVOX_MID360;
            newFilterPair = std::make_shared<ExpressionFilter_Mid360>();

            TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

            if ((plainText.at(charIndex) == '{') || charIndex >= plainText.length())
            {
                Issue error;
                error.beginChar = deviceStringStartIndex;
                error.endChar = deviceDefinitionEndIndex;
                error.text = "IP address needed for device type mid360.";
                throw error;
            }

            int ipStringStartIndex = charIndex;
            QString ipString = TextBlockParser::getSubString(plainText, charIndex, " \t\n{").toLower();
            deviceDefinitionEndIndex = charIndex;

            QHostAddress ipAddressNotValidated;

            if (!ipAddressNotValidated.setAddress(QString(ipString)))
            {
                Issue error;
                error.beginChar = ipStringStartIndex;
                error.endChar = charIndex;
                error.text = "Can't convert parameter \"" + ipString + "\" to IPv4 address.";
                throw error;
            }

            bool convOk = false;
            quint32 hostAddress = ipAddressNotValidated.toIPv4Address(&convOk);

            if (!convOk)
            {
                Issue error;
                error.beginChar = ipStringStartIndex;
                error.endChar = charIndex;
                error.text = "Parameter \"" + ipString + "\" is not a valid IPv4 address.";
                throw error;
            }

            deviceStringForErrors += " " + ipString;

            device.data = hostAddress;
        }
        else
        {
            Issue error;
            error.beginChar = deviceStringStartIndex;
            error.endChar = charIndex;
            error.text = "Unknown device type: \"" + deviceStringForErrors + "\".";
            throw error;
        }

        if (filters.contains(device))
        {
            Issue error;
            error.beginChar = deviceStringStartIndex;
            error.endChar = charIndex;
            error.text = "Duplicate device: \"" + deviceStringForErrors + "\".";

            throw error;
        }

        newFilterPair->setConvexHullFilters(convexHullFilters);

        try
        {
            TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

            if (charIndex >= plainText.length())
            {
                Issue error;
                error.beginChar = deviceStringStartIndex;
                error.endChar = deviceDefinitionEndIndex;
                error.text = "Expression definitions missing for device \"" + deviceStringForErrors + "\".";
                throw error;
            }

            if (plainText.at(charIndex) != '{')
            {
                Issue error;
                error.beginChar = charIndex;
                error.endChar = charIndex + 1;
                error.text = "Only comments and whitespaces allowed between device definition and opening curly brace for filter expression.";
                throw error;
            }

            int expressionsStartIndex = charIndex;
            QString filterExpression = TextBlockParser::getTextBlockAsString(plainText, charIndex);
            int filterExpressionEndIndex= charIndex;

            try
            {
                newFilterPair->setExpression_Filter(filterExpression);
            }
            catch (ExpressionFilter_Base::Issue& issue)
            {
                Issue error;
                error.beginChar = issue.beginChar + expressionsStartIndex + 1;    // + 1 from starting '{'
                error.endChar = issue.endChar + expressionsStartIndex + 1;
                error.text = "Error compiling filter expression: " + issue.text;

                throw error;
            }

            TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

            if (charIndex >= plainText.length())
            {
                Issue error;
                error.beginChar = deviceStringStartIndex;
                error.endChar = filterExpressionEndIndex;
                error.text = "Quality expression definition missing for device \"" + deviceStringForErrors + "\".";
                throw error;
            }

            if (plainText.at(charIndex) != '{')
            {
                Issue error;
                error.beginChar = charIndex;
                error.endChar = charIndex +1;
                error.text = "Only comments and whitespaces allowed between closing and opening curly braces for filter and quality expression.";
                throw error;
            }

            expressionsStartIndex = charIndex;
            filterExpression = TextBlockParser::getTextBlockAsString(plainText, charIndex);

            try
            {
                newFilterPair->setExpression_Quality(filterExpression);
            }
            catch (ExpressionFilter_Base::Issue& issue)
            {
                Issue error;
                error.beginChar = issue.beginChar + expressionsStartIndex + 1;    // + 1 from starting '{'
                error.endChar = issue.endChar + expressionsStartIndex + 1;
                error.text = "Error compiling quality expression: " + issue.text;

                throw error;
            }
        }
        catch (TextBlockParser::Issue& issue)
        {
            Issue error;
            error.beginChar = issue.beginChar;
            error.endChar = issue.endChar;
            error.text = issue.text;

            throw error;
        }

        filters.insert(device, newFilterPair);
    }

    return filters;
}

}; // namespace PointFilter




























