/*
    rastercameragenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include <QDebug>

#include "rastercameragenerator.h"
#include "EasyEXIF/exif.h"

RasterCameraGenerator::RasterCameraGenerator()
{
    init();
}

void RasterCameraGenerator::init(void)
{
    rasterItemOutputFormatString.clear();

    baseDir.setPath("");
    relativeDir.setPath("");

    timeShift = 0;

    referenceImageDateTime.setTime(QTime());    // Invalidate
    referenceImageUptime = -1;
    referenceImageITOW = -1;

    outString.clear();
}

RasterCameraGenerator::Item::Item(const QString& text, const int lineNumber, const int firstCol, const int lastCol)
{
    this->text = text;
    this->lineNumber = lineNumber;
    this->firstCol = firstCol;
    this->lastCol = lastCol;
};

QString RasterCameraGenerator::generate(const Params& params)
{
    init();

    int lineNumber = 0;

    QVector<Item> command;

    QString subString;
    int firstCol = -1;

    while (lineNumber < params.lines->count())
    {
        QString line = params.lines->at(lineNumber);

        int i = 0;

//        int endCol = -1;

        bool expectingEscapeCharacter = false;
        bool quoting = false;
        int quoteStart = 0;

        while (i < line.length())
        {
            QChar character = line.at(i);
            char latin1Character = character.toLatin1();

            if ((line.length() >= (i + 2)) && (latin1Character == '/') && (line.at(i+1).toLatin1() == '/') && (!quoting))
            {
                if (expectingEscapeCharacter)
                {
                    Issue error;
                    error.text = "Comment section not allowed right after escape character (\"\\\").";
                    error.item.lineNumber = lineNumber;
                    error.item.firstCol = i;
                    error.item.lastCol = i;
                    throw error;
                }

                // Rest of the line is comment -> Skip it
                break;
            }
            else if (!expectingEscapeCharacter && latin1Character == '\\')
            {
                expectingEscapeCharacter = true;
            }
            else if (expectingEscapeCharacter)
            {
                switch (latin1Character)
                {
                case '\\':
                    subString += '\\';
                    break;
                case '\"':
                    subString += '\"';
                    break;
                case 'n':
                    subString += '\n';
                    break;
                case '\'':
                    subString += '\'';
                    break;
                case 't':
                    subString += '\t';
                    break;
                default:
                    Issue error;
                    error.text = "Unsupported escape character.";
                    error.item.lineNumber = lineNumber;
                    error.item.firstCol = i;
                    error.item.lastCol = i;
                    throw error;
                    break;
                }

                expectingEscapeCharacter = false;
            }
            else if (latin1Character == '\"')
            {
                if (!quoting)
                {
                    quoteStart = i;
                }

                quoting = !quoting;
            }
            else if (((latin1Character == ' ') || (latin1Character == '\t')) && (!quoting))
            {
                // Command/argument separator
                if (!subString.isEmpty())
                {
                    Item newItem;
                    newItem.text = subString;
                    newItem.lineNumber = lineNumber;
                    newItem.firstCol = firstCol;
                    newItem.lastCol = i - 1;
                    command.push_back(newItem);
                    subString.clear();
                }
            }
            else if ((latin1Character == ';') && (!quoting))
            {
                if (!subString.isEmpty())
                {
                    Item newItem;
                    newItem.text = subString;
                    newItem.lineNumber = lineNumber;
                    newItem.firstCol = firstCol;
                    newItem.lastCol = i - 1;
                    command.push_back(newItem);
                    subString.clear();
                }

                if (command.size() == 0)
                {
                    break;
                }

                processCommand(command, params);
                command.clear();
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
        if (expectingEscapeCharacter)
        {
            Issue error;
            error.text = "Unterminated escape sequence in the end of line.";
            error.item.lineNumber = lineNumber;
            error.item.firstCol = i - 2;
            error.item.lastCol = i - 1;
            throw error;
        }
        else if (quoting)
        {
            Issue error;
            error.text = "Unterminated quote in the end of line.";
            error.item.lineNumber = lineNumber;
            error.item.firstCol = quoteStart;
            error.item.lastCol = i - 1;
            throw error;
        }
        else if (!subString.isEmpty())
        {
            Item newItem;
            newItem.text = subString;
            newItem.lineNumber = lineNumber;
            newItem.firstCol = firstCol;
            newItem.lastCol = i - 1;
            command.push_back(newItem);
            subString.clear();
        }

        lineNumber++;
    }

    if (!command.isEmpty())
    {
        Issue error;
        error.text = "Unterminated command in the end.";
        error.item = command.at(0);
        throw error;
    }

    return outString;
}

void RasterCameraGenerator::processCommand(const QVector<Item>& command, const Params& params)
{
    QString cmd0 = command.at(0).text.toLower();

    if (cmd0 == "writeoutputstring")
    {
        cmd_WriteOutputString(command);
    }
    else if (cmd0 == "rasteritemoutputformatstring")
    {
        cmd_RasterItemOutputFormatString(command);
    }
    else if (cmd0 == "basepath")
    {
        cmd_BasePath(command);
    }
    else if (cmd0 == "relativepath")
    {
        cmd_RelativePath(command);
    }
    else if (cmd0 == "referencetimeimage")
    {
        cmd_ReferenceTimeImage(command);
    }
    else if (cmd0 == "processstills")
    {
        cmd_ProcessStills(command, params);
    }
    else if (cmd0 == "timeshift")
    {
        cmd_TimeShift(command);
    }
    else
    {
        Issue error;
        error.text = "Unknown command \"" + command.at(0).text + "\"";
        error.item = command.at(0);
        throw error;
    }
}

void RasterCameraGenerator::cmd_WriteOutputString(const QVector<Item>& command)
{
    checkArgumentCount(command, 1);
    outString += command.at(1).text;
}

void RasterCameraGenerator::cmd_RasterItemOutputFormatString(const QVector<Item>& command)
{
    genericStringCmd(command, rasterItemOutputFormatString);
}

QDateTime RasterCameraGenerator::readEXIFDateTimeFromString(QString dateTimeString)
{
    QDateTime dateTime;

    if (dateTimeString.length() != 19)
    {
        return dateTime;
    }
    else
    {
        // Convert from EXIF's "YYYY:MM:DD hh:mm:ss" to Qt:ISOData ("yyyy-MM-ddTHH:mm:ss")
        dateTimeString.replace(4,1,"-");
        dateTimeString.replace(7,1,"-");
        dateTimeString.replace(10,1,"T");

        return QDateTime::fromString(dateTimeString, Qt::ISODate);
    }
}

QDateTime RasterCameraGenerator::readEXIFDateTimeFromFile(const QString fileName)
{
    QFile file(fileName);
    file.open(QIODevice::ReadOnly | QIODevice::ExistingOnly);

    QByteArray fileData = file.readAll();

    easyexif::EXIFInfo exifInfo;
    exifInfo.parseFrom((unsigned char*)fileData.data(), fileData.size());

    QDateTime dateTime;

    // Prefer "Original" date&time
    // (There was some info somewhere that this may not be available, so fall back to others if necessary)
    dateTime = readEXIFDateTimeFromString(exifInfo.DateTimeOriginal.c_str());
    if (dateTime.isValid())
    {
        return dateTime;
    }

    dateTime = readEXIFDateTimeFromString(exifInfo.DateTimeDigitized.c_str());
    if (dateTime.isValid())
    {
        return dateTime;
    }

    dateTime = readEXIFDateTimeFromString(exifInfo.DateTime.c_str());
    return dateTime;
}

void RasterCameraGenerator::genericStringCmd(const QVector<Item>& command, QString& string)
{
    checkArgumentCount(command, 1, 2);

    QString subCommand = command.at(1).text.toLower();
    if (subCommand == "clear")
    {
        checkArgumentCount(command, 1);
        string.clear();
    }
    else if (subCommand == "set")
    {
        checkArgumentCount(command, 2);
        string = command.at(2).text;
    }
    else if (subCommand == "append")
    {
        checkArgumentCount(command, 2);
        string += command.at(2).text;
    }
    else
    {
        Issue error;
        error.text = "Unknown subcommand for " + command.at(0).text +  "\"" + command.at(1).text + "\"";
        error.item = command.at(1);
        throw error;
    }
}

void RasterCameraGenerator::cmd_BasePath(const QVector<Item> &command)
{
    genericPathCmd(command, baseDir, "Base path");

    emit infoMessage("Full path is now \"" + fullPath() + "\".");
}

void RasterCameraGenerator::cmd_RelativePath(const QVector<Item> &command)
{
    genericPathCmd(command, relativeDir, "Relative path");

    emit infoMessage("Full path is now \"" + fullPath() + "\".");
}

void RasterCameraGenerator::cmd_ReferenceTimeImage(const QVector<Item>& command)
{
    checkArgumentCount(command, 2, 3);

    QString fileName = command.at(1).text;

    fileName.replace("%{BASEPATH}", baseDir.path());
    fileName.replace("%{RELATIVEPATH}", relativeDir.path());
    fileName.replace("%{FULLPATH}", fullPath());

    emit infoMessage("Opening and parsing time reference image file \"" + fileName + "\"...");

    referenceImageDateTime = readEXIFDateTimeFromFile(fileName);

    if (!referenceImageDateTime.isValid())
    {
        Issue error;
        error.text = "Can't parse valid date/time for time reference image \"" + fileName + "\".";
        error.item = command.at(1);
        throw error;
    }

    referenceImageITOW = -1;
    referenceImageUptime = -1;

    for (int i = 2; i < command.size(); i++)
    {
        QStringList time = command.at(i).text.split(":");

        if (time.size() != 2)
        {
            Issue error;
            error.text = "Malformed time identifier (allowed itow:value or uptime:value).";
            error.item = command.at(i);
            throw error;
        }

        bool convOk = false;
        qint64 timeValue = time.at(1).toLongLong(&convOk);

        if (!convOk)
        {
            Issue error;
            error.text = "Can not convert time value to integer.";
            error.item = command.at(i);
            throw error;
        }

        if (time.at(0).toLower() == "itow")
        {
            if (referenceImageITOW != -1)
            {
                Issue error;
                error.text = "ITOW time already defined.";
                error.item = command.at(i);
                throw error;
            }

            referenceImageITOW = timeValue;
        }
        else if (time.at(0).toLower() == "uptime")
        {
            if (referenceImageUptime != -1)
            {
                Issue error;
                error.text = "Uptime already defined.";
                error.item = command.at(i);
                throw error;
            }

            referenceImageUptime = timeValue;
        }
        else
        {
            Issue error;
            error.text = "Unidentified time identifier (allowed itow or uptime).";
            error.item = command.at(i);
            throw error;
        }
    }

    if ((referenceImageITOW < 0) && (referenceImageUptime < 0))
    {
        Issue error;
        error.text = "Either itow or uptime must be defined as positive value.";
        error.item = command.at(0);
        throw error;
    }

    emit infoMessage("EXIF time " + referenceImageDateTime.toString() +
                     " set to correspond ITOW " + QString::number(referenceImageITOW) +
                     " and uptime " + QString::number(referenceImageUptime) +
                     " (-1: value not defined)");
}

void RasterCameraGenerator::cmd_ProcessStills(const QVector<Item>& command, const Params& params)
{
    checkArgumentCount(command, 1, 2);

    QString filter = command.at(1).text;
    QStringList filterList(filter);

    bool useUptimes =false;

    Eigen::Transform<double, 3, Eigen::Affine> transform_XYZToNED_NoTranslation;
    transform_XYZToNED_NoTranslation = (*params.transform_NEDToXYZ).linear().transpose();

    if (command.size() < 3)
    {
        emit infoMessage("No time stamps (ITOW/uptime) for syncing defined. Defaulting to ITOW.");
    }
    else
    {
        QString argument = command.at(2).text.toLower();

        if (argument == "itows")
        {
            emit infoMessage("Using itows for syncing.");
            useUptimes = false;
        }
        else if (argument == "uptimes")
        {
            emit infoMessage("Using uptime for syncing.");
            useUptimes = true;
        }
    }

    if ((useUptimes) && (referenceImageUptime == -1))
    {
        Issue error;
        error.text = "Reference uptime not valid.";
        error.item = command.at(0);
        throw error;
    }
    else if ((!useUptimes) && (referenceImageITOW == -1))
    {
        Issue error;
        error.text = "Reference ITOW not valid.";
        error.item = command.at(0);
        throw error;
    }

    QDir dir = QDir(fullPath());

    emit infoMessage("Processing files in directory \"" + dir.path() + "\"...");

    if (!dir.exists())
    {
        Issue error;
        error.text = "Directory doesn't exist.";
        error.item = command.at(0);
        throw error;
    }

    dir.setNameFilters(filterList);
    dir.setSorting(QDir::Name);
    dir.setFilter(QDir::Files);

    QStringList fileList = dir.entryList();

    // Map where uptimes for all equal ITOWs are the same.
    // This makes processing later easier
    // Uptimes here are calculated as averages from rover values (for each ITOW)
    QMap<qint64, UBXMessage_RELPOSNED::ITOW> averagedSync;

    emit infoMessage("Generating equalized rover uptime timestamps...");
    PostProcessingForm::generateAveragedRoverUptimeSync(params.rovers, averagedSync);
    emit infoMessage("Equalized rover uptime timestamps created. Number of items: " + QString::number(averagedSync.size()));

    for (int i = 0; i < fileList.size(); i++)
    {
        QString fileName = fullPath(fileList[i]);

        emit infoMessage("Processing file \"" + fileName + "\"...");
        QDateTime imageDateTime = readEXIFDateTimeFromFile(fileName);

        if (!imageDateTime.isValid())
        {
            Issue error;
            error.text = "Can't parse valid date/time for image \"" + fileName + "\".";
            error.item = command.at(1);
            throw error;
        }

        Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

        qint64 timeOffset = referenceImageDateTime.msecsTo(imageDateTime);
        double timeOffset_s = timeOffset * 0.001;

        if (useUptimes)
        {
            qint64 imageUptime = referenceImageUptime + timeOffset + timeShift;

            emit infoMessage("EXIF time " + imageDateTime.toString() +
                             " with time offset " + QString::number(timeOffset_s) + " s to reference image" +
                             " set to correspond uptime " + QString::number(imageUptime));

            try
            {
                params.loInterpolator->getInterpolatedLocationOrientationTransformMatrix_Uptime(imageUptime, averagedSync, transform_LoSolver);
            }
            catch (QString& stringThrown)
            {
                Issue error;
                error.text = "Failed to solve location/orientation for camera. Error: " + stringThrown;
                error.item = command.at(1);
                throw error;
            }
        }
        else
        {
            UBXMessage_RELPOSNED::ITOW imageITOW = referenceImageITOW + timeOffset + timeShift;

            emit infoMessage("EXIF time " + imageDateTime.toString() +
                             " with time offset " + QString::number(timeOffset_s) + " s to reference image" +
                             " set to correspond ITOW " + QString::number(imageITOW));

            try
            {
                params.loInterpolator->getInterpolatedLocationOrientationTransformMatrix_ITOW(imageITOW, transform_LoSolver);
            }
            catch (QString& stringThrown)
            {
                Issue error;
                error.text = "Failed to solve location/orientation for camera. Error: " + stringThrown;
                error.item = command.at(1);
                throw error;
            }
        }

        Eigen::Transform<double, 3, Eigen::Affine> finalMatrix = *params.transform_NEDToXYZ * transform_LoSolver * *params.transform_Generated * transform_XYZToNED_NoTranslation;

        QString fileString = rasterItemOutputFormatString;

        const int originDecimals = 4;
        const int unitVectorDecimals = 6;

        fileString.replace("%{TRANSLATION_X}", QString::number(finalMatrix(0,3), 'f', originDecimals));
        fileString.replace("%{TRANSLATION_Y}", QString::number(finalMatrix(1,3), 'f', originDecimals));
        fileString.replace("%{TRANSLATION_Z}", QString::number(finalMatrix(2,3), 'f', originDecimals));

        fileString.replace("%{LINEAR_11}", QString::number(finalMatrix(0,0), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_12}", QString::number(finalMatrix(1,0), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_13}", QString::number(finalMatrix(2,0), 'f', unitVectorDecimals));

        fileString.replace("%{LINEAR_21}", QString::number(finalMatrix(0,1), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_22}", QString::number(finalMatrix(1,1), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_23}", QString::number(finalMatrix(2,1), 'f', unitVectorDecimals));

        fileString.replace("%{LINEAR_31}", QString::number(finalMatrix(0,2), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_32}", QString::number(finalMatrix(1,2), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_33}", QString::number(finalMatrix(2,2), 'f', unitVectorDecimals));

        fileString.replace("%{TRANSLATION_NEGATED_X}", QString::number(-finalMatrix(0,3), 'f', originDecimals));
        fileString.replace("%{TRANSLATION_NEGATED_Y}", QString::number(-finalMatrix(1,3), 'f', originDecimals));
        fileString.replace("%{TRANSLATION_NEGATED_Z}", QString::number(-finalMatrix(2,3), 'f', originDecimals));

        fileString.replace("%{LINEAR_NEGATED_11}", QString::number(-finalMatrix(0,0), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_12}", QString::number(-finalMatrix(1,0), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_13}", QString::number(-finalMatrix(2,0), 'f', unitVectorDecimals));

        fileString.replace("%{LINEAR_NEGATED_21}", QString::number(-finalMatrix(0,1), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_22}", QString::number(-finalMatrix(1,1), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_23}", QString::number(-finalMatrix(2,1), 'f', unitVectorDecimals));

        fileString.replace("%{LINEAR_NEGATED_31}", QString::number(-finalMatrix(0,2), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_32}", QString::number(-finalMatrix(1,2), 'f', unitVectorDecimals));
        fileString.replace("%{LINEAR_NEGATED_33}", QString::number(-finalMatrix(2,2), 'f', unitVectorDecimals));

        fileString.replace("%{BASEPATH}", baseDir.path());
        fileString.replace("%{RELATIVEPATH}", relativeDir.path());
        fileString.replace("%{FULLPATH}", fullPath());
        fileString.replace("%{FULLFILEPATH}", fileName);
        fileString.replace("%{FILENAME}", fileList[i]);

        outString += fileString;
    }
}

void RasterCameraGenerator::cmd_TimeShift(const QVector<Item>& command)
{
    checkArgumentCount(command, 1, 1);

    bool convOk = false;
    timeShift = command.at(1).text.toLong(&convOk);

    if (!convOk)
    {
        Issue error;
        error.text = "Unable to convert timeshift value to integer.";
        error.item = command.at(1);
        throw error;
    }
}

void RasterCameraGenerator::genericPathCmd(const QVector<Item>& command, QDir& dir, QString pathTitle)
{
    checkArgumentCount(command, 1, 2);

    QString subCommand = command.at(1).text.toLower();
    if (subCommand == "clear")
    {
        checkArgumentCount(command, 1);
        dir.setPath("");
        emit infoMessage(pathTitle + " cleared.");
    }
    else if (subCommand == "home")
    {
        checkArgumentCount(command, 1);
        dir = QDir::home();
        emit infoMessage(pathTitle + " set to home path (\"" + dir.path() + "\").");
    }
    else if (subCommand == "root")
    {
        checkArgumentCount(command, 1);
        dir = QDir::root();
        emit infoMessage(pathTitle + " set to root path (\"" + dir.path() + "\").");
    }
    else if (subCommand == "set")
    {
        checkArgumentCount(command, 2);
        dir.setPath(command.at(2).text);
        emit infoMessage(pathTitle + " changed to \"" + dir.path() + "\".");
    }
    else if (subCommand == "append")
    {
        checkArgumentCount(command, 2);
        dir.setPath(dir.path() + command.at(2).text);
        emit infoMessage(pathTitle + " changed to \"" + dir.path() + "\".");
    }
    else if (subCommand == "cd")
    {
        checkArgumentCount(command, 2);
        dir.cd(command.at(2).text);
        emit infoMessage(pathTitle + " changed to \"" + dir.path() + "\".");
    }
    else
    {
        Issue error;
        error.text = "Unknown subcommand for " + command.at(0).text + "\"" + command.at(1).text + "\".";
        error.item = command.at(1);
        throw error;
    }
}

QString RasterCameraGenerator::fullPath(QString fileName)
{
    QString fullPath;

    if (baseDir.path().length() > 0)
    {
        fullPath = baseDir.path() + "/" + relativeDir.path();
    }
    else
    {
        fullPath = relativeDir.path();
    }

    if (fileName.length())
    {
        fullPath += "/" + fileName;
    }

    return fullPath;

    /* QDir->cd doesn't work when directory doesn't exist...
    if (basePath.path().length() > 0)
    {
        QDir fullPath = basePath;
        fullPath.cd(relativePath.path());
        return fullPath.path();
    }
    else
    {
        return relativePath.path();
    }
    */
}

void RasterCameraGenerator::checkArgumentCount(const QVector<Item>& command, const int argsNumMin, int argsNumMax)
{
    if (argsNumMax == -1)
    {
        argsNumMax = argsNumMin;
    }

    int argNum = command.count() - 1;

    if (argNum < argsNumMin)
    {
        Issue error;
        error.text = "Not enough arguments. Required " + QString::number(argsNumMin) + ", got " + QString::number(argNum);
        error.item = command.at(0);
        throw error;
    }
    else if (argNum > argsNumMax)
    {
        Issue error;
        error.text = "Too many arguments. Allowed " + QString::number(argsNumMax) + ", got " + QString::number(argNum);
        error.item = command.at(argsNumMax + 1);
        throw error;
    }
}

