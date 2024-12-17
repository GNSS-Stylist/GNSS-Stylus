/*
    convexhullgenerator.cpp (part of GNSS-Stylus)
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

#include "convexhullgenerator.h"
#include "Util/textblockparser.h"
#include "PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h"

QMap<QString, ConvexHull> ConvexHullGenerator::generateMap(const QString& plainText)
{
    QMap<QString, ConvexHull> hulls;

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

        int hullNameStartIndex = charIndex;
        QString hullNameString = TextBlockParser::getSubString(plainText, charIndex, " \t\n{");
        QString hullNameStringLowerCase = hullNameString.toLower();
        int hullNameEndIndex = charIndex;

        if (hulls.contains(hullNameStringLowerCase))
        {
            Issue error;
            error.beginChar = hullNameStartIndex;
            error.endChar = charIndex;
            error.text = "Duplicate hull: \"" + hullNameString + "\".";
            throw error;
        }

        for (int i = 0; i < hullNameStringLowerCase.length(); i++)
        {
            QChar character = hullNameStringLowerCase.at(i);

            if (!(((character >= 'a') && (character <= 'z')) || ((character >= '0') && (character <= '9'))))
            {
                Issue error;
                error.beginChar = hullNameStartIndex + i;
                error.endChar = error.beginChar + 1;
                error.text = "Only alphanumeric \"ASCII\" (a...z, 0...9) allowed in convex hull name.";
                throw error;
            }
        }

        try
        {
            TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

            if (charIndex >= plainText.length())
            {
                Issue error;
                error.beginChar = hullNameStartIndex;
                error.endChar = hullNameEndIndex;
                error.text = "Point definitions missing for hull \"" + hullNameString + "\".";
                throw error;
            }

            if (plainText.at(charIndex) != '{')
            {
                Issue error;
                error.beginChar = charIndex;
                error.endChar = charIndex + 1;
                error.text = "Only comments and whitespaces allowed between hull name and opening curly brace for coordinate definition block.";
                throw error;
            }

            int coordinateBlockStartIndex = charIndex;
            QString coordinateBlockContents = TextBlockParser::getTextBlockAsString(plainText, charIndex, true);
            int coordinateBlockEndIndex = charIndex;

            ConvexHull newHull;
            addHullPointsFromBlock(coordinateBlockContents, coordinateBlockStartIndex + 1, newHull);

            if (newHull.getNumOfUniquePoints() < 4)
            {
                Issue error;
                error.beginChar = coordinateBlockStartIndex;
                error.endChar = coordinateBlockEndIndex;
                error.text = "At least 4 unique points needed to define a convex hull.";
                throw error;
            }

            if (!(newHull.isHullvalid()))
            {
                Issue error;
                error.beginChar = coordinateBlockStartIndex;
                error.endChar = coordinateBlockEndIndex;
                error.text = "Hull is invalid (convhull_3d failed to construct it).";
                throw error;
            }

            hulls.insert(hullNameStringLowerCase, newHull);
        }
        catch (TextBlockParser::Issue& issue)
        {
            Issue error;
            error.beginChar = issue.beginChar;
            error.endChar = issue.endChar;
            error.text = issue.text;
            throw error;
        }
    }

    return hulls;
}

void ConvexHullGenerator::addHullPointsFromBlock(const QString &blockString, const int blockStartCharIndex, ConvexHull& hull)
{
    int charIndex = 0;
    int blockStringLength = blockString.length();

    while (charIndex < blockStringLength)
    {
        TextBlockParser::skipWhitespacesAndComments(blockString, charIndex);

        if (charIndex >= blockStringLength)
        {
            break;
        }

        if (blockString.at(charIndex) != '{')
        {
            Issue error;
            error.beginChar = charIndex + blockStartCharIndex;
            error.endChar = charIndex + blockStartCharIndex+ 1;
            error.text = "Only comments and whitespaces allowed before first coordinate definition block.";
            throw error;
        }

        int coordinateBlockStartIndex = charIndex + blockStartCharIndex;
        QString coordinateBlockContents = TextBlockParser::getTextBlockAsString(blockString, charIndex, true);

        Eigen::Vector3d newPoint = extractCoordinatesFromBlock(coordinateBlockContents, coordinateBlockStartIndex + 1); // +1 for '{'

        if (!hull.addPoint(newPoint))
        {
            Issue error;
            error.beginChar = coordinateBlockStartIndex;
            error.endChar = charIndex + blockStartCharIndex;
            error.text = "Duplicate point. Only unique points allowed when defining a convex hull.";
            throw error;
        }
    }
}

Eigen::Vector3d ConvexHullGenerator::extractCoordinatesFromBlock(const QString& blockString, const int blockStartCharIndex)
{
    double coords[3];
    int charIndex = 0;
    int blockStringLength = blockString.length();

    for (int i = 0; i < 3; i++)
    {
        TextBlockParser::skipWhitespacesAndComments(blockString, charIndex);

        if (charIndex >= blockStringLength)
        {
            Issue error;
            error.beginChar = blockStartCharIndex;
            error.endChar = blockStringLength + blockStartCharIndex;
            error.text = "Not enough coordinate blocks (3 for xyz) in point definition block.";
            throw error;
        }

        if (blockString.at(charIndex) != '{')
        {
            Issue error;
            error.beginChar = charIndex + blockStartCharIndex;
            error.endChar = charIndex + blockStartCharIndex+ 1;
            error.text = "Only comments and whitespaces allowed in point definition block outside the coordinate blocks.";
            throw error;
        }

        int coordinateBlockStartIndex = charIndex + blockStartCharIndex;
        QString coordinateBlockContents = TextBlockParser::getTextBlockAsString(blockString, charIndex);

        coords[i] = evaluateBlockContents(coordinateBlockContents, coordinateBlockStartIndex + 1); // + 1 for '{'
    }

    TextBlockParser::skipWhitespacesAndComments(blockString, charIndex);

    if ((charIndex < blockStringLength) && (blockString.at(charIndex) == '{'))
    {
        Issue error;
        error.beginChar = charIndex + blockStartCharIndex;
        error.endChar = charIndex + blockStartCharIndex + 1;
        error.text = "Opening curly brace after point definitions (only 3 spatial dimensions in use in the known universe).";
        throw error;
    }

    if (charIndex < blockStringLength)
    {
        Issue error;
        error.beginChar = charIndex + blockStartCharIndex;
        error.endChar = charIndex + blockStartCharIndex + 1;
        error.text = "Only comments and whitespaces allowed in point definition block after the coordinate blocks.";
        throw error;
    }

    return Eigen::Vector3d(coords[0], coords[1], coords[2]);
}

double ConvexHullGenerator::evaluateBlockContents(const QString& blockString, const int blockStartCharIndex)
{
    QByteArray expression_8bit;
    TextBlockParser::CommentState cState;

    for (int i = 0; i < blockString.length(); i++)
    {
        bool inComment = TextBlockParser::isInComment(blockString, i, cState);

        char character = blockString.at(i).toLatin1();

        if (character == 0)
        {
            if (inComment)
            {
                character = '?';
            }
            else
            {
                Issue error;
                error.text = "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.";
                error.beginChar = i + blockStartCharIndex;
                error.endChar = i + blockStartCharIndex + 1;
                throw error;
            }
        }

        expression_8bit += character;
    }

    char* prevLocale = std::setlocale(LC_NUMERIC, "C");

    te_parser parser;
    double evalResult = parser.evaluate(expression_8bit.constData());

    if (prevLocale)
    {
        setlocale(LC_NUMERIC, prevLocale);
    }

    if (std::isnan(evalResult))
    {
        Issue error;

        QString qstrErrorMessage = QString::fromStdString(parser.get_last_error_message());

        if (qstrErrorMessage.isEmpty())
        {
            error.text = "Error evaluating expression. TinyExpr error: (empty).";
        }
        else
        {
            error.text = "Error evaluating expression. TinyExpr error: " + qstrErrorMessage;
        }

        int errorPos = parser.get_last_error_position();

        if (errorPos == te_parser::npos)
        {
            errorPos = 0;
        }

        error.beginChar = errorPos + blockStartCharIndex;
        error.endChar = error.beginChar;
        throw error;
    }

    if (!std::isfinite(evalResult))
    {
        // Not absolutely sure if TinyExpr++ ever returns infinite
        // (could not find a way by quickly checking).
        // Doesn't cost much to check, so why not...
        Issue error;
        error.text = "Expression returns an infinite value (plus or minus).";
        error.beginChar = blockStartCharIndex;
        error.endChar = blockString.length() + blockStartCharIndex;
        throw error;
    }

    return evalResult;
}
