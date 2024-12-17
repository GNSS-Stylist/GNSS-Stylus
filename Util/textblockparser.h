/*
    textblockparser.h (part of GNSS-Stylus)
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

#ifndef TEXTBLOCKPARSER_H
#define TEXTBLOCKPARSER_H

#include <QString>

class TextBlockParser
{
public:
    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    class CommentState
    {
    public:
        bool inEOLComment = false;
        bool inBlockComment = false;
        int commentStartIndex = -1;
        void init(void) { inEOLComment = false; inBlockComment = false; commentStartIndex = -1; };
    };

//    TextBlockParser();
    static bool isInSet(const QChar& character, const QString& charSet);
    static bool skipCharacters(const QString& plainText, int& charIndex, const QString& charsToSkip, const bool skipComments = true);
    static QString getSubString(const QString& plainText, int& charIndex, const QString& endingChars);
    static bool isNewline(const QChar character) { return isInSet(character, "\n\r" ); };
    static bool isInComment(const QString& plainText, int& charIndex, CommentState& commentState);
    static bool skipSingleComment(const QString& plainText, int& charIndex);
    static bool skipComments(const QString& plainText, int& charIndex);
    static bool skipWhitespacesAndComments(const QString& plainText, int& charIndex);
    static QByteArray getTextBlockAsByteArray(const QString& plainText, int& charIndex, const bool allowRecursiveCurlyBraces = false);
    static QString getTextBlockAsString(const QString& plainText, int& charIndex, const bool allowRecursiveCurlyBraces = false);
};

#endif // TEXTBLOCKPARSER_H
