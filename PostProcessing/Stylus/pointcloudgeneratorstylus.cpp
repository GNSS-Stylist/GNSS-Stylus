/*
    pointcloudgeneratorstylus.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "pointcloudgeneratorstylus.h"

namespace Stylus
{

void PointCloudGenerator::generatePointClouds(const Params& params)
{
    // This function is identical to the one used in Lidar's PointCloudGenerator.
    // I actually first wrote a base class so that this function was implemented there
    // and only "specialized" generatePointCloudPointSet-function was implemented in
    // Lidar's and Stylus' subclasses. This, however felt like an overkill and obfuscated
    // the structure instead of making it clearer.
    // Maybe duplicating 200+ lines of code can be seen as a bad practise, but whatever...

    if (!params.directory.exists())
    {
        emit errorMessage("Directory \"" + params.directory.path() + "\" doesn't exist. Point cloud files not created.");
        return;
    }

    emit infoMessage("Processing...");

    bool objectActive = false;

    qint64 beginningUptime = -1;
    int pointsWritten = 0;

    bool ignoreBeginningAndEndingTags = false;

    QFile* outFile = nullptr;
    QTextStream* outStream = nullptr;

    QString objectName;
    QString baseFileName;
    int fileIndex = 0;

    qint64 uptime = -1;
    PostProcessingForm::Tag beginningTag;

    while (params.tags->upperBound(uptime) != params.tags->end())
    {
        uptime = params.tags->upperBound(uptime).key();

        QList<PostProcessingForm::Tag> tagItems = params.tags->values(uptime);

        // Since "The items that share the same key are available from most recently to least recently inserted."
        // (taken from QMultiMap's doc), iterate in "reverse order" here

        for (int i = tagItems.size() - 1; i >= 0; i--)
        {
            const PostProcessingForm::Tag& currentTag = tagItems[i];

            if (!(currentTag.ident.compare(params.tagIdent_BeginNewObject)))
            {
                // Tag type: new object

                if (objectActive)
                {
                    // Object already active -> Close existing stream and file

                    if (outStream)
                    {
                        delete outStream;
                        outStream = nullptr;
                    }
                    if (outFile)
                    {
                        emit infoMessage("Closing file \"" + outFile->fileName() + "\".");
                        outFile->close();
                        delete outFile;
                        outFile = nullptr;
                    }

                    emit infoMessage("Object \"" + objectName + "\": Total points written: " + QString::number(pointsWritten));

                    objectActive = false;
                }

                objectName = currentTag.text;

                if (currentTag.text.length() == 0)
                {
                    // Empty name for the new object not allowed

                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": New object without a name. Ending previous object, but not beginning new nor creating a new file. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    continue;
                }

                baseFileName = QDir::cleanPath(params.directory.path() + "/" + currentTag.text);

                QString fileName = baseFileName + ".xyz";

                if (!params.separateFilesForSubScans)
                {
                    outFile = createNewOutFile(fileName, currentTag, uptime);

                    if (!outFile)
                    {
                        ignoreBeginningAndEndingTags = true;
                        continue;
                    }
                    else
                    {
                        outStream = new QTextStream(outFile);
                        ignoreBeginningAndEndingTags = false;
                    }
                }
                else
                {
                    emit infoMessage("Starting new object \"" +  currentTag.text + "\".");
                    ignoreBeginningAndEndingTags = false;
                }

                objectActive = true;
                beginningUptime = -1;
                pointsWritten = 0;

                fileIndex = 0;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: Begin points

                if (!objectActive)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": Beginning tag outside object. Skipped.");
                    continue;
                }

                if (beginningUptime != -1)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": Duplicate beginning tag. Skipped.");
                    continue;
                }

                // Just store the beginning uptime-value and tag. Writing of the points is done in ending tag-branch
                beginningUptime = uptime;
                beginningTag = currentTag;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: end points

                if (!objectActive)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": End tag outside object. Skipped.");
                    continue;
                }

                if (beginningUptime == -1)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": End tag without beginning tag. Skipped.");
                    continue;
                }

                const PostProcessingForm::Tag& endingTag = currentTag;

                if (endingTag.sourceFile != beginningTag.sourceFile)
                {
                    emit warningMessage("Starting and ending tags belong to different files. Starting tag file \"" +
                               beginningTag.sourceFile + "\", line " +
                               QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                               endingTag.sourceFile + "\", line " +
                               QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                    continue;
                }

                if (params.separateFilesForSubScans)
                {
                    fileIndex++;

                    QString fileIndexString = QString::number(fileIndex);

                    while (fileIndexString.length() < 4)
                    {
                        fileIndexString.prepend("0");
                    }

                    QString fileName = QDir::cleanPath(baseFileName + "_" + fileIndexString + ".xyz");

                    outFile = createNewOutFile(fileName, currentTag, uptime);

                    if (!outFile)
                    {
                        ignoreBeginningAndEndingTags = true;
                        continue;
                    }
                    else
                    {
                        outStream = new QTextStream(outFile);
                        objectActive = true;
                        ignoreBeginningAndEndingTags = false;
                    }
                }

                bool generatingOk = false;
                int prevPointsWritten = pointsWritten;

                generatingOk = generatePointCloudPointSet(params, beginningTag, endingTag, beginningUptime, uptime, outStream, pointsWritten);

                if (generatingOk)
                {
                    int pointsBetweenTags = pointsWritten - prevPointsWritten;

                    if (pointsBetweenTags == 0)
                    {
                        emit warningMessage("File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                                   QString::number(beginningTag.sourceFileLine) +
                                   ", uptime " + QString::number(beginningUptime) +
                                   ", iTOW " + QString::number(beginningTag.iTOW) + ", ending tag line " +
                                   QString::number(endingTag.sourceFileLine) +
                                   ", uptime " + QString::number(endingTag.iTOW) +
                                   ", iTOW " + QString::number(endingTag.iTOW) +
                                   ", File \"" + endingTag.sourceFile + "\""
                                   " No points between tags.");
                    }
                }

                if (params.separateFilesForSubScans)
                {
                    if (outStream)
                    {
                        delete outStream;
                        outStream = nullptr;
                    }
                    if (outFile)
                    {
                        int pointsBetweenTags = pointsWritten - prevPointsWritten;
                        emit infoMessage("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsBetweenTags));
                        outFile->close();
                        delete outFile;
                        outFile = nullptr;
                    }
                }

                beginningUptime = -1;
            }
        }
    }

    if (beginningUptime != -1)
    {
        emit warningMessage("File \"" + beginningTag.sourceFile + "\", line " +
                   QString::number(beginningTag.sourceFileLine) +
                   ", iTOW " + QString::number(beginningUptime) +
                   ", iTOW " + QString::number(beginningTag.iTOW) +
                   " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
    }

    if (outStream)
    {
        delete outStream;
    }
    if (outFile)
    {
        emit infoMessage("Closing file \"" + outFile->fileName() + "\".");
        outFile->close();
        delete outFile;
    }

    if (objectActive)
    {
        emit infoMessage("Object \"" + objectName + "\": Total points written: " + QString::number(pointsWritten));
    }

    emit infoMessage("Point cloud files generated.");
}


bool PointCloudGenerator::generatePointCloudPointSet(const Params& params,
                                                     const PostProcessingForm::Tag& beginningTag,
                                                     const PostProcessingForm::Tag& endingTag,
                                                     const qint64 beginningUptime, const qint64 endingUptime,
                                                     QTextStream* outStream,
                                                     int& pointsWritten)
{
    double stylusTipDistanceFromRoverA = params.initialStylusTipDistanceFromRoverA;
    bool includeNormals = params.includeNormals;

    bool constDistancesOnly = true;

    QMap<qint64, PostProcessingForm::DistanceItem>::const_iterator distIter = params.distances->upperBound(beginningUptime);

    while ((distIter != params.distances->end()) && (distIter.key() < endingUptime))
    {
        if (distIter.value().type == PostProcessingForm::DistanceItem::Type::MEASURED)
        {
            constDistancesOnly = false;
            break;
        }

        distIter++;
    }

    int pointsBetweenTags = 0;

    if (constDistancesOnly)
    {
        distIter = params.distances->upperBound(beginningUptime);

        if (distIter != params.distances->end())
        {
            if ((distIter == params.distances->begin()) ||
                    ((--distIter).value().type != PostProcessingForm::DistanceItem::Type::CONSTANT))
            {
                emit warningMessage("File \"" + endingTag.sourceFile + "\", line " +
                           QString::number(endingTag.sourceFileLine)+
                           ", uptime " + QString::number(endingUptime) +
                           ", iTOW " + QString::number(endingTag.iTOW) +
                           ": Points between tags having only constant distances without preceeding constant distance. Skipped.");

                return false;
            }

            stylusTipDistanceFromRoverA = distIter.value().distance;
        }

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = params.rovers[0].relposnedMessages.upperBound(beginningTag.iTOW);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = params.rovers[1].relposnedMessages.upperBound(beginningTag.iTOW);

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = params.rovers[0].relposnedMessages.upperBound(endingTag.iTOW);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = params.rovers[1].relposnedMessages.upperBound(endingTag.iTOW);

        while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
            (relposIterator_RoverB != relposIterator_RoverB_EndTag))
        {
            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                   (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
            {
                // Skip all rover A RELPOSNEDs that have lower iTOW than the next rover B RELPOSNED (sync)
                relposIterator_RoverA++;
            }

            while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                   (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                   (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
            {
                // Skip all rover B RELPOSNEDs that have lower iTOW than the next rover A RELPOSNED (sync)
                relposIterator_RoverB++;
            }

            if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                (relposIterator_RoverB != relposIterator_RoverB_EndTag))
            {
                Eigen::Vector3d roverAPosNED(
                        relposIterator_RoverA.value().relPosN,
                        relposIterator_RoverA.value().relPosE,
                        relposIterator_RoverA.value().relPosD);

                Eigen::Vector3d roverBPosNED(
                        relposIterator_RoverB.value().relPosN,
                        relposIterator_RoverB.value().relPosE,
                        relposIterator_RoverB.value().relPosD);

                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                // Convert to XYZ-coordinates
                Eigen::Vector3d roverAPosXYZ = *params.transform_NEDToXYZ * roverAPosNED;
                Eigen::Vector3d roverBPosXYZ = *params.transform_NEDToXYZ * roverBPosNED;
                Eigen::Vector3d stylusTipPosXYZ = *params.transform_NEDToXYZ * stylusTipPosNED;
                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                QString lineOut;

                if (includeNormals)
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                }
                else
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                }

                outStream->operator<<(lineOut + "\n");

                pointsWritten++;

                pointsBetweenTags++;
                relposIterator_RoverA++;
                relposIterator_RoverB++;
            }
        }
    }
    else
    {
        // Distances found, sync point creation to them.
        // Rover coordinates are interpolated according to distance timestamps.

        distIter = params.distances->upperBound(beginningUptime);

        while ((distIter != params.distances->end()) && (distIter.key() < endingUptime))
        {
            if (distIter.value().type == PostProcessingForm::DistanceItem::Type::CONSTANT)
            {
                emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                           QString::number(distIter.value().sourceFileLine)+
                           ", uptime " + QString::number(distIter.key()) +
                           ": Constant distance between measured ones. Skipped.");
                distIter++;
                continue;
            }
            else if (distIter.value().type == PostProcessingForm::DistanceItem::Type::MEASURED)
            {
                // Try to find next and previous rover coordinates
                // for this uptime

                qint64 distanceUptime = distIter.key();
                // TODO: Add/subtract fine tune sync value here if needed

                UBXMessage_RELPOSNED interpolated_Rovers[2];
                bool fail = false;

                for (int i = 0; i < 2; i++)
                {
                    QMap<qint64, PostProcessingForm::RoverSyncItem>::const_iterator roverUptimeIter = params.rovers[i].roverSyncData.lowerBound(distanceUptime);

                    if (roverUptimeIter != params.rovers[i].roverSyncData.end())
                    {
                        const PostProcessingForm::RoverSyncItem upperSyncItem = roverUptimeIter.value();
                        PostProcessingForm::RoverSyncItem lowerSyncItem;

                        if (roverUptimeIter != params.rovers[i].roverSyncData.begin())
                        {
                            lowerSyncItem = (--roverUptimeIter).value();
                        }
                        else
                        {
                            emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + PostProcessingForm::getRoverIdentString(i) + " sync data (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        if (params.rovers[i].relposnedMessages.find(upperSyncItem.iTOW) == params.rovers[i].relposnedMessages.end())
                        {
                            emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + PostProcessingForm::getRoverIdentString(i) + " iTOW (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        if (params.rovers[i].relposnedMessages.find(lowerSyncItem.iTOW) == params.rovers[i].relposnedMessages.end())
                        {
                            emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                       QString::number(distIter.value().sourceFileLine)+
                                       ", uptime " + QString::number(distIter.key()) +
                                       ": Can not find corresponding rover" + PostProcessingForm::getRoverIdentString(i) + " iTOW (higher limit). Skipped.");
                            distIter++;
                            fail = true;
                            break;
                        }

                        qint64 timeDiff = distanceUptime - roverUptimeIter.key();

                        interpolated_Rovers[i] = UBXMessage_RELPOSNED::interpolateCoordinates(params.rovers[i].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                params.rovers[i].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                    }
                    else
                    {
                        emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                   QString::number(distIter.value().sourceFileLine)+
                                   ", uptime " + QString::number(distIter.key()) +
                                   ": Can not find corresponding rover" + PostProcessingForm::getRoverIdentString(i) + " sync data (upper limit). Skipped.");
                        distIter++;
                        fail = true;
                        break;
                    }
                }

                if (fail)
                {
                    continue;
                }

                stylusTipDistanceFromRoverA = distIter.value().distance;

                // TODO: Refine this quick hack or at least make the distance configurable!
                // Skip distances that are too far away
                // (measurement module seems to emit "outliers" now and then)
                if (stylusTipDistanceFromRoverA > 2)
                {
                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                               QString::number(distIter.value().sourceFileLine)+
                               ", uptime " + QString::number(distIter.key()) +
                               ": Distance between RoverA and tip too high (" +
                               QString::number(stylusTipDistanceFromRoverA) +
                               " m). Skipped.");
                    distIter++;
                    continue;
                }

                Eigen::Vector3d roverAPosNED(
                        interpolated_Rovers[0].relPosN,
                        interpolated_Rovers[0].relPosE,
                        interpolated_Rovers[0].relPosD);

                Eigen::Vector3d roverBPosNED(
                        interpolated_Rovers[1].relPosN,
                        interpolated_Rovers[1].relPosE,
                        interpolated_Rovers[1].relPosD);

                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                // Convert to XYZ-coordinates
                Eigen::Vector3d roverAPosXYZ = *params.transform_NEDToXYZ * roverAPosNED;
                Eigen::Vector3d roverBPosXYZ = *params.transform_NEDToXYZ * roverBPosNED;
                Eigen::Vector3d stylusTipPosXYZ = *params.transform_NEDToXYZ * stylusTipPosNED;
                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                QString lineOut;

                if (includeNormals)
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                }
                else
                {
                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                }

                outStream->operator<<(lineOut + "\n");
                pointsWritten++;
                pointsBetweenTags++;
            }
            else
            {
                emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                           QString::number(distIter.value().sourceFileLine)+
                           ", uptime " + QString::number(distIter.key()) +
                           ": Unknown distance type between measured ones. Skipped.");
                distIter++;
                continue;
            }

            distIter++;
        }
    }

    return true;
}

QFile *PointCloudGenerator::createNewOutFile(const QString fileName, const PostProcessingForm::Tag &currentTag, const qint64 uptime)
{
    QFile* outFile = new QFile(fileName);

    if (outFile->exists())
    {
        // File already exists -> Not allowed

        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                   QString::number(currentTag.sourceFileLine)+
                   ", uptime " + QString::number(uptime) +
                   ", iTOW " + QString::number(currentTag.iTOW) +
                   ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

        delete outFile;
        outFile = nullptr;
        return outFile;
    }

    emit infoMessage("Creating file \"" + fileName + "\"...");

    if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        // Creating the file failed

        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                   QString::number(currentTag.sourceFileLine)+
                   ", uptime " + QString::number(uptime) +
                   ", iTOW " + QString::number(currentTag.iTOW) +
                   ": File \"" + fileName + "\" can't be created. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

        delete outFile;
        outFile = nullptr;
        return outFile;
    }

    return outFile;
}

}; // namespace Stylus
