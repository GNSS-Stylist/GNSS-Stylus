/*
    textblockparser.cpp (part of GNSS-Stylus)
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

#include "textblockparser.h"

bool TextBlockParser::isInSet(const QChar& character, const QString& charSet)
{
    for (int i = 0; i < charSet.length(); i++)
    {
        if (character == charSet.at(i))
        {
            return true;
        }
    }

    return false;
}

bool TextBlockParser::skipCharacters(const QString& plainText, int& charIndex, const QString& charsToSkip, const bool skipComments)
{
    bool skipped = false;

    while (charIndex < plainText.length())
    {
        bool skippedComments = false;

        if (skipComments)
        {
            skippedComments = TextBlockParser::skipComments(plainText, charIndex);
        }
        if (charIndex >= plainText.length())
        {
            break;
        }

        bool skip = false;

        if (isInSet(plainText.at(charIndex), charsToSkip))
        {
            skip = true;
            skipped = true;
            charIndex++;
        }

        if (!skip && !skippedComments)
        {
            break;
        }
    }

    return skipped;
}

QString TextBlockParser::getSubString(const QString& plainText, int& charIndex, const QString& endingChars)
{
    QString subString;

    CommentState cState;

    while (charIndex < plainText.length())
    {
        if ((isInSet(plainText.at(charIndex), endingChars)) || (isInComment(plainText, charIndex, cState)))
        {
            // Comment breaks substring
            break;
        }

        subString += plainText.at(charIndex);
        charIndex++;
    }

    return subString;
}

class CommentState
{
public:
    bool inEOLComment = false;
    bool inBlockComment = false;
    int commentStartIndex = -1;
    void init(void) { inEOLComment = false; inBlockComment = false; commentStartIndex = -1; };
};

bool TextBlockParser::isInComment(const QString& plainText, int& charIndex, CommentState& commentState)
{
    if (charIndex >= plainText.length())
    {
        return false;
    }

    if (commentState.inEOLComment)
    {
        if ((charIndex > 1) && (isNewline(plainText.at(charIndex - 1))))
        {
            // End EOL comment _after_ newline
            // so that the "cursor" is left in the start of the next line

            commentState.init();
        }
        else
        {
            return true;
        }
    }
    else if (commentState.inBlockComment)
    {
        if ((charIndex >= 3) && ((charIndex - commentState.commentStartIndex) >= 3) &&
            (plainText.at(charIndex - 2) == '*') && (plainText.at(charIndex - 1) == '/'))
        {
            // End block comment _after_ "*/"

            commentState.init();
        }
        else
        {
            return true;
        }
    }

    if (charIndex >= plainText.length() - 2)
    {
        // No space left for comment start sequence
        return false;
    }

    if ((plainText.at(charIndex) == '/') && (plainText.at(charIndex + 1) == '*'))
    {
        commentState.commentStartIndex = charIndex;
        commentState.inBlockComment = true;
        commentState.inEOLComment = false;
        return true;
    }

    if ((plainText.at(charIndex) == '/') && (plainText.at(charIndex + 1) == '/'))
    {
        commentState.commentStartIndex = charIndex;
        commentState.inBlockComment = false;
        commentState.inEOLComment = true;
        return true;
    }

    return false;
}

bool TextBlockParser::skipSingleComment(const QString& plainText, int& charIndex)
{
    CommentState cState;

    bool skipped = false;

    while (charIndex < plainText.length())
    {
        if (!isInComment(plainText, charIndex, cState))
        {
            break;
        }

        skipped = true;
        charIndex++;
    }

    return skipped;
}

bool TextBlockParser::skipComments(const QString& plainText, int& charIndex)
{
    bool skipped = false;

    while (skipSingleComment(plainText, charIndex))
    {
        skipped = true;
    }

    return skipped;
}

bool TextBlockParser::skipWhitespacesAndComments(const QString& plainText, int& charIndex)
{
    bool skippedEver = false;

    while (charIndex < plainText.length())
    {
        bool skipped = false;

        if (skipSingleComment(plainText, charIndex))
        {
            skipped = true;
        }
        if (skipCharacters(plainText, charIndex, " \t\n\r"))
        {
            skipped = true;
        }

        skippedEver |= skipped;

        if (!skipped)
        {
            break;
        }
    }

    return skippedEver;
}

QByteArray TextBlockParser::getTextBlockAsByteArray(const QString& plainText, int& charIndex)
{
    QByteArray retval;

    if (charIndex >= plainText.length())
    {
        Issue error;
        error.beginChar = charIndex;
        error.endChar = charIndex;
        error.text = "Block doesn't exist.";
        throw error;
    }

    if (plainText.at(charIndex) != '{')
    {
        Issue error;
        error.beginChar = charIndex;
        error.endChar = charIndex + 1;
        error.text = "Block not starting with \"{\"-character.";
        throw error;
    }

    int blockStartIndex = charIndex;

    charIndex++;

    CommentState cState;

    while (charIndex < plainText.length())
    {
        bool isCommentChar = isInComment(plainText, charIndex, cState);

        char character = plainText.at(charIndex).toLatin1();

        if (!isCommentChar)
        {
            if (character == 0)
            {
                Issue error;
                error.beginChar = charIndex;
                error.endChar = charIndex + 1;
                error.text = "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment blocks.";
                throw error;
            }

            if (character == '}')
            {
                charIndex++;
                return retval;
            }
        }

        if (character == 0)
        {
            // Substitute "non-ascii"-characters as question marks in comment sections
            character = '?';
        }

        retval += character;
        charIndex++;
    }

    // No '}' in non-comment section encountered before the end of source string

    Issue error;
    error.beginChar = blockStartIndex;
    error.endChar = blockStartIndex + 1;
    error.text = "Unterminated block (matching \"}\"-character missing).";
    throw error;
}

QString TextBlockParser::getTextBlockAsString(const QString& plainText, int& charIndex)
{
    QString retval;

    if (charIndex >= plainText.length())
    {
        Issue error;
        error.beginChar = charIndex;
        error.endChar = charIndex;
        error.text = "Block doesn't exist.";
        throw error;
    }

    if (plainText.at(charIndex) != '{')
    {
        Issue error;
        error.beginChar = charIndex;
        error.endChar = charIndex + 1;
        error.text = "Block not starting with \"{\"-character.";
        throw error;
    }

    int blockStartIndex = charIndex;

    charIndex++;

    CommentState cState;

    while (charIndex < plainText.length())
    {
        bool isCommentChar = isInComment(plainText, charIndex, cState);

        QChar character = plainText.at(charIndex);

        if (!isCommentChar)
        {
            if (character == '}')
            {
                charIndex++;
                return retval;
            }
        }

        retval += character;
        charIndex++;
    }

    // No '}' in non-comment section encountered before the end of source string

    Issue error;
    error.beginChar = blockStartIndex;
    error.endChar = blockStartIndex + 1;
    error.text = "Unterminated block (matching \"}\"-character missing).";
    throw error;
}






























