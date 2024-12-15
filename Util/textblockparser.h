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
    static QByteArray getTextBlockAsByteArray(const QString& plainText, int& charIndex);
    static QString getTextBlockAsString(const QString& plainText, int& charIndex);
};

#endif // TEXTBLOCKPARSER_H
