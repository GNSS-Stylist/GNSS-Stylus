/*
    tst_textblockparser.cpp (part of GNSS-Stylus)
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

#include "tst_textblockparser.h"
#include "../Util/textblockparser.h"

TestTextBlockParser::TestTextBlockParser()
{
}

TestTextBlockParser::~TestTextBlockParser()
{
}

void TestTextBlockParser::initTestCase()
{
}

void TestTextBlockParser::cleanupTestCase()
{
}

void TestTextBlockParser::plainText()
{
    QString plainText = "This is just a plain text, without any comments or blocks.\nBut newline anyway.";
    QString plainTextBackup = plainText;

    for (int index = 0; index < plainText.length(); index++)
    {
        int feedIndex = index;

        TextBlockParser::CommentState cState;

        QCOMPARE(TextBlockParser::isInSet(plainText.at(feedIndex), " ,\n"), ((plainText.at(index) == ' ') || (plainText.at(index) == ',') || (plainText.at(index) == '\n')));
        QCOMPARE(feedIndex, index);

        QCOMPARE(TextBlockParser::isNewline(plainText.at(feedIndex)), (plainText.at(index) == '\n'));
        QCOMPARE(feedIndex, index);

        QVERIFY(!TextBlockParser::isInComment(plainText, feedIndex, cState));
        QCOMPARE(feedIndex, index);

        QCOMPARE(cState.commentStartIndex, -1);
        QCOMPARE(cState.inBlockComment, false);
        QCOMPARE(cState.inEOLComment, false);
    }

    QCOMPARE(plainText, plainTextBackup);
}

void TestTextBlockParser::skipCharacters()
{
    QString plainText = "\tThis is just a plain text, without any comments or blocks.\nBut newline anyway and multiple    spaces.   ";
    QString plainText_Stripped = "ThisisjustaplaintextwithoutanycommentsorblocksButnewlineanywayandmultiplespaces";
    QString plainTextBackup = plainText;
    QString destText;

    int index = 0;

    while (index < plainText.length())
    {
        int prevIndex = index;

        bool shouldSkip = ((plainText.at(index) == ' ') || (plainText.at(index) == ',') || (plainText.at(index) == '\t') || (plainText.at(index) == '.') || (plainText.at(index) == '\n'));
        QCOMPARE(TextBlockParser::skipCharacters(plainText, index, " ,.\n\t"), shouldSkip);

        if (index < plainText.length())
        {
            destText += plainText.at(index);
        }

        if (shouldSkip)
        {
            QVERIFY(prevIndex != index);
        }
        else
        {
            QCOMPARE(prevIndex, index);
        }

        index++;
    }

    QCOMPARE(plainText, plainTextBackup);
    QCOMPARE(destText, plainText_Stripped);
}

void TestTextBlockParser::getSubString()
{
    QString plainText = "\tThis is just a plain text, /*with*/ too /*many*//*comments*/.\nAnd newline and multiple    spaces.//EOL-comment too\nMore text. Com/*foobar*/ment, comm//\nent/* comment*/";
    QString plainText_Stripped = "Thisisjustaplaintext,too.Andnewlineandmultiplespaces.Moretext.Comment,comment";
    QString expectedSubStrings[] = { "This", "is", "just", "a", "plain", "text,", "too", ".", "And", "newline", "and", "multiple", "spaces.", "More", "text.", "Com", "ment,", "comm", "ent"};

    QString plainTextBackup = plainText;
    QString destText;

    unsigned int subStringIndex = 0;

    int index = 0;

    while (index < plainText.length())
    {
        TextBlockParser::skipWhitespacesAndComments(plainText, index);

        if (index < plainText.length())
        {
            QString subString = TextBlockParser::getSubString(plainText, index, " \n\r\t");

            QVERIFY(subStringIndex < sizeof(expectedSubStrings) / sizeof(expectedSubStrings[0]));
            QCOMPARE(subString, expectedSubStrings[subStringIndex]);

            destText += subString;
            subStringIndex++;
        }
    }

    QCOMPARE(plainText, plainTextBackup);
    QCOMPARE(destText, plainText_Stripped);
}

void TestTextBlockParser::blockHandling()
{
    QString plainText_AllInOneBlock = "{\tThis is just a plain text, /*with*/ too /*many*//*comments*/.\nAnd newline and multiple    spaces.//EOL-comment too\nMore text. Com/*foobar*/ment, comm//\nent/* comment*/}";
    QByteArray plainText_AllInOneBlock_ExpectedOut = "\tThis is just a plain text, /*with*/ too /*many*//*comments*/.\nAnd newline and multiple    spaces.//EOL-comment too\nMore text. Com/*foobar*/ment, comm//\nent/* comment*/";
    QString plainTextBackup_AllInOneBlock = plainText_AllInOneBlock;
    QByteArray destText;

    int charIndex = 0;

    destText = TextBlockParser::getTextBlockAsByteArray(plainText_AllInOneBlock, charIndex);

    QCOMPARE(plainText_AllInOneBlock, plainTextBackup_AllInOneBlock);
    QCOMPARE(destText, plainText_AllInOneBlock_ExpectedOut);

    QString plainText_SeveralBlocks = "0123456789{9876543210}     {test}";
    QString plainTextBackup_SeveralBlocks = plainText_SeveralBlocks;

    charIndex = 10;

    destText = TextBlockParser::getTextBlockAsByteArray(plainText_SeveralBlocks, charIndex);
    QCOMPARE(plainText_SeveralBlocks, plainTextBackup_SeveralBlocks);
    QCOMPARE(destText, "9876543210");
    QCOMPARE(charIndex, 10 + 10 + 2);

    charIndex += 5;

    destText = TextBlockParser::getTextBlockAsByteArray(plainText_SeveralBlocks, charIndex);
    QCOMPARE(plainText_SeveralBlocks, plainTextBackup_SeveralBlocks);
    QCOMPARE(destText, "test");
    QCOMPARE(charIndex, 10 + 10 + 2 + 5 + 4 + 2);

    QString chineseHelloWorldInComment = QString::fromUtf8("{Hello world in chinese: /* \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd */}");
    QString chineseHelloWorldInComment_Backup = chineseHelloWorldInComment;

    try
    {
        charIndex = 0;
        destText = TextBlockParser::getTextBlockAsByteArray(chineseHelloWorldInComment, charIndex);

        // Hello world in chinese (世界您好) (<- Does this survive gitHub etc. btw?) is 4 characters long, substituted with spaces here
        QCOMPARE(destText, QString::fromUtf8("Hello world in chinese: /* ???? */"));
    }
    catch (TextBlockParser::Issue& issue)
    {
        QFAIL("Should not throw an exception");
    }

    QCOMPARE(chineseHelloWorldInComment, chineseHelloWorldInComment_Backup);

    QString chineseHelloWorldInBlock = QString::fromUtf8("{Hello world in chinese: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd}");
    QString chineseHelloWorldInBlock_ExpectedOut = QString::fromUtf8("Hello world in chinese: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd");
    QString chineseHelloWorldInBlock_Backup = chineseHelloWorldInBlock;

    QString destTextQString;

    try
    {
        charIndex = 0;
        destTextQString = TextBlockParser::getTextBlockAsString(chineseHelloWorldInBlock, charIndex);

        QCOMPARE(destTextQString, chineseHelloWorldInBlock_ExpectedOut);
    }
    catch (TextBlockParser::Issue& issue)
    {
        QFAIL("Should not throw an exception");
    }

    QCOMPARE(chineseHelloWorldInBlock, chineseHelloWorldInBlock_Backup);
}

void TestTextBlockParser::Errors()
{
    QString plainText_IndexingTests = "0123456789{9876543210}";
    QString plainTextBackup_IndexingTests = plainText_IndexingTests;
    QByteArray destText_ByteArray;
    QString destText_QString;

    int charIndex = plainText_IndexingTests.length();

    try
    {
        destText_ByteArray = TextBlockParser::getTextBlockAsByteArray(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, charIndex);
        QCOMPARE(issue.endChar, charIndex);
        QCOMPARE(issue.text, "Block doesn't exist.");
    }

    try
    {
        destText_QString = TextBlockParser::getTextBlockAsString(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, charIndex);
        QCOMPARE(issue.endChar, charIndex);
        QCOMPARE(issue.text, "Block doesn't exist.");
    }


    try
    {
        charIndex = 0;

        destText_ByteArray = TextBlockParser::getTextBlockAsByteArray(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 0);
        QCOMPARE(issue.endChar, 1);
        QCOMPARE(issue.text, "Block not starting with \"{\"-character.");
    }

    try
    {
        charIndex = 0;

        destText_QString = TextBlockParser::getTextBlockAsString(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 0);
        QCOMPARE(issue.endChar, 1);
        QCOMPARE(issue.text, "Block not starting with \"{\"-character.");
    }


    try
    {
        charIndex = 5;

        destText_ByteArray = TextBlockParser::getTextBlockAsByteArray(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 5);
        QCOMPARE(issue.endChar, 5 + 1);
        QCOMPARE(issue.text, "Block not starting with \"{\"-character.");
    }

    try
    {
        charIndex = 5;

        destText_QString = TextBlockParser::getTextBlockAsString(plainText_IndexingTests, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 5);
        QCOMPARE(issue.endChar, 5 + 1);
        QCOMPARE(issue.text, "Block not starting with \"{\"-character.");
    }

    QCOMPARE(plainText_IndexingTests, plainTextBackup_IndexingTests);

    QString chineseHelloWorldInBlock = QString::fromUtf8("{Hello world in chinese: \xe4\xb8\x96\xe7\x95\x8c\xe6\x82\xa8\xe5\xa5\xbd}");
    QString chineseHelloWorldInBlock_Backup = chineseHelloWorldInBlock;

    try
    {
        charIndex = 0;
        destText_ByteArray = TextBlockParser::getTextBlockAsByteArray(chineseHelloWorldInBlock, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 25);
        QCOMPARE(issue.endChar, 25 + 1);
        QCOMPARE(issue.text, "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment blocks.");
    }

    QCOMPARE(chineseHelloWorldInBlock, chineseHelloWorldInBlock_Backup);

    QString unterminatedBlock = QString::fromUtf8("01234{This block doesn't end");
    QString unterminatedBlock_Backup = unterminatedBlock;

    try
    {
        charIndex = 5;
        destText_ByteArray = TextBlockParser::getTextBlockAsByteArray(unterminatedBlock, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 5);
        QCOMPARE(issue.endChar, 5 + 1);
        QCOMPARE(issue.text, "Unterminated block (matching \"}\"-character missing).");
    }

    try
    {
        charIndex = 5;
        destText_QString = TextBlockParser::getTextBlockAsString(unterminatedBlock, charIndex);

        QFAIL("Should throw an exception");
    }
    catch (TextBlockParser::Issue& issue)
    {
        QCOMPARE(issue.beginChar, 5);
        QCOMPARE(issue.endChar, 5 + 1);
        QCOMPARE(issue.text, "Unterminated block (matching \"}\"-character missing).");
    }

    QCOMPARE(unterminatedBlock, unterminatedBlock_Backup);
}




