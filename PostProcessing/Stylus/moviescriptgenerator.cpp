/*
    moviescriptgenerator.cpp (part of GNSS-Stylus)
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

/**
 * @file moviescriptgenerator.cpp
 * @brief Definition for a class generating a "movie script" that can be used with Processing-sketch (for example)
 */

#include <QObject>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QPushButton>
#include <QTextStream>

#include "moviescriptgenerator.h"

namespace Stylus
{

void MovieScriptGenerator::GenerateMovieScript(const Params& params)
{
    Q_ASSERT(params.transform);
    Q_ASSERT(params.tags);
    Q_ASSERT(params.distances);
    Q_ASSERT(params.rovers);

    double stylusTipDistanceFromRoverA = params.initialStylusTipDistanceFromRoverA;

    Eigen::Matrix3d transform_NoTranslation = params.transform->linear();

    QFile movieScriptFile;

    movieScriptFile.setFileName(params.fileName);

    if (movieScriptFile.exists())
    {
        QMessageBox msgBox;
        msgBox.setText("File already exists.");
        msgBox.setInformativeText("How to proceed?");

        QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
        QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

        msgBox.setDefaultButton(cancelButton);

        msgBox.exec();

        if (msgBox.clickedButton() != overwriteButton)
        {
            emit infoMessage("Movie script not created.");
            return;
        }
    }

    if (!movieScriptFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open movie script file.");
        return;
    }

    QTextStream textStream(&movieScriptFile);

    textStream << "// Lines\tiTOW\tX\tY\tZ\taccX\tAccY\tAccZ\tObject\n";

    emit infoMessage("Processing line sets...");

//        QMap<qint64, Tag_New>::const_iterator currentTagIterator;

    bool objectActive = false;

    qint64 beginningUptime = -1;
    int pointsWritten = 0;

    bool ignoreBeginningAndEndingTags = false;

    QString objectName = "N/A";

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
                objectActive = false;

                if (currentTag.text.length() == 0)
                {
                    emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", uptime " + QString::number(uptime) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ": New object without a name. Ending previous object, but not beginning new nor creating a new line. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;
                    objectName = "N/A";

                    continue;
                }

                emit infoMessage("Object \"" + currentTag.text + "\"...");

                objectActive = true;
                objectName = currentTag.text;
                ignoreBeginningAndEndingTags = false;
                beginningUptime = -1;
                pointsWritten = 0;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
            {
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

                beginningUptime = uptime;
                beginningTag = currentTag;
            }
            else if ((!(currentTag.ident.compare(params.tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
            {
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
//                const Tag_New beginningTag = tags_New[beginningUptime];

                if (endingTag.sourceFile != beginningTag.sourceFile)
                {
                    emit warningMessage("Starting and ending tags belong to different files. Starting tag file \"" +
                               beginningTag.sourceFile + "\", line " +
                               QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                               endingTag.sourceFile + "\", line " +
                               QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                    continue;
                }

                bool constDistancesOnly = true;

                QMap<qint64, PostProcessingForm::DistanceItem>::const_iterator distIter = params.distances->upperBound(beginningUptime);

                while ((distIter != params.distances->end()) && (distIter.key() < uptime))
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
                            emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                       QString::number(currentTag.sourceFileLine)+
                                       ", uptime " + QString::number(uptime) +
                                       ", iTOW " + QString::number(currentTag.iTOW) +
                                       ": Points between tags having only constant distances without preceeding constant distance. Skipped.");
                            continue;
                        }

                        stylusTipDistanceFromRoverA = distIter.value().distance;
                    }

                    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = params.rovers[0].relposnedMessages.upperBound(params.rovers[0].relposnedMessages.upperBound(beginningUptime).value().iTOW);
                    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = params.rovers[1].relposnedMessages.upperBound(params.rovers[1].relposnedMessages.upperBound(beginningUptime).value().iTOW);

                    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = params.rovers[0].relposnedMessages.upperBound(currentTag.iTOW);
                    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = params.rovers[1].relposnedMessages.upperBound(currentTag.iTOW);

                    while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                        (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                    {
                        while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                               (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                               (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
                        {
                            relposIterator_RoverA++;
                        }

                        while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                               (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                               (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
                        {
                            relposIterator_RoverB++;
                        }

                        if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                            (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                        {
                            if ((relposIterator_RoverA.key() >= params.iTOWRange_Lines_Min) &&
                            (relposIterator_RoverA.key() <= params.iTOWRange_Lines_Max))
                            {
                                Eigen::Vector3d roverAPosNED(
                                        relposIterator_RoverA.value().relPosN,
                                        relposIterator_RoverA.value().relPosE,
                                        relposIterator_RoverA.value().relPosD);

                                Eigen::Vector3d roverBPosNED(
                                        relposIterator_RoverB.value().relPosN,
                                        relposIterator_RoverB.value().relPosE,
                                        relposIterator_RoverB.value().relPosD);

                                Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                                Eigen::Vector3d stylusTipPosXYZ = *params.transform * stylusTipPosNED;

                                Eigen::Vector3d stylusTipAccNED(
                                            relposIterator_RoverA.value().accN,
                                            relposIterator_RoverA.value().accE,
                                            relposIterator_RoverA.value().accD);

                                // Use accuracies of rover A (used for stylus tip accuracy)
                                // Could calculate some kind of "worst case" scenario using both rovers,
                                // but probably errors are mostly common to both of them.
                                Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

                                QString lineOut;

                                if (pointsBetweenTags == 0)
                                {
                                    lineOut = "LStart";

                                }
                                else
                                {
                                    lineOut = "LCont";
                                }

                                lineOut +=
                                        "\t" + QString::number(relposIterator_RoverA.key()) +
                                        "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                        "\t" + objectName;

                                textStream << (lineOut + "\n");

                                pointsWritten++;

                                pointsBetweenTags++;
                            }
                            relposIterator_RoverA++;
                            relposIterator_RoverB++;
                        }
                    }
                } // if (constDistancesOnly)
                else
                {
                    // Distances found, sync point creation to them.
                    // Rover coordinates are interpolated according to distance timestamps.

                    distIter = params.distances->upperBound(beginningUptime);

                    while ((distIter != params.distances->end()) && (distIter.key() < uptime))
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

                            QMap<qint64, PostProcessingForm::RoverSyncItem>::const_iterator roverAUptimeIter = params.rovers[0].roverSyncData.lowerBound(distanceUptime);
                            UBXMessage_RELPOSNED interpolated_RoverA;

                            if (roverAUptimeIter != params.rovers[0].roverSyncData.end())
                            {
                                const PostProcessingForm::RoverSyncItem upperSyncItem = roverAUptimeIter.value();
                                PostProcessingForm::RoverSyncItem lowerSyncItem;
                                if (roverAUptimeIter != params.rovers[0].roverSyncData.begin())
                                {
                                    lowerSyncItem = (--roverAUptimeIter).value();
                                }
                                else
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A sync data (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                if (params.rovers[0].relposnedMessages.find(upperSyncItem.iTOW) == params.rovers[0].relposnedMessages.end())
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                if (params.rovers[0].relposnedMessages.find(lowerSyncItem.iTOW) == params.rovers[0].relposnedMessages.end())
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                interpolated_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(params.rovers[0].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                        params.rovers[0].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                            }
                            else
                            {
                                emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Can not find corresponding rover A sync data (upper limit). Skipped.");
                                distIter++;
                                continue;
                            }

                            QMap<qint64, PostProcessingForm::RoverSyncItem>::const_iterator roverBUptimeIter = params.rovers[1].roverSyncData.lowerBound(distanceUptime);
                            UBXMessage_RELPOSNED interpolated_RoverB;

                            if (roverBUptimeIter != params.rovers[1].roverSyncData.end())
                            {
                                const PostProcessingForm::RoverSyncItem upperSyncItem = roverBUptimeIter.value();
                                PostProcessingForm::RoverSyncItem lowerSyncItem;
                                if (roverBUptimeIter != params.rovers[1].roverSyncData.begin())
                                {
                                    lowerSyncItem = (--roverBUptimeIter).value();
                                }
                                else
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B sync data (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                if (params.rovers[1].relposnedMessages.find(upperSyncItem.iTOW) == params.rovers[1].relposnedMessages.end())
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                if (params.rovers[1].relposnedMessages.find(lowerSyncItem.iTOW) == params.rovers[1].relposnedMessages.end())
                                {
                                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                interpolated_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(params.rovers[1].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                        params.rovers[1].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                            }
                            else
                            {
                                emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Can not find corresponding rover B sync data (upper limit). Skipped.");
                                distIter++;
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
                                    interpolated_RoverA.relPosN,
                                    interpolated_RoverA.relPosE,
                                    interpolated_RoverA.relPosD);

                            Eigen::Vector3d roverBPosNED(
                                    interpolated_RoverB.relPosN,
                                    interpolated_RoverB.relPosE,
                                    interpolated_RoverB.relPosD);

                            Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                            Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                            Eigen::Vector3d stylusTipPosXYZ = *params.transform * stylusTipPosNED;

                            Eigen::Vector3d stylusTipAccNED(
                                        interpolated_RoverA.accN,
                                        interpolated_RoverA.accE,
                                        interpolated_RoverA.accD);

                            // Use accuracies of rover A (used for stylus tip accuracy)
                            // Could calculate some kind of "worst case" scenario using both rovers,
                            // but probably errors are mostly common to both of them.
                            Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

                            QString lineOut;

                            if (pointsBetweenTags == 0)
                            {
                                lineOut = "LStart";
                            }
                            else
                            {
                                lineOut = "LCont";
                            }

                            lineOut +=
                                    "\t" + QString::number(interpolated_RoverA.iTOW) +
                                    "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                    "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                    "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                    "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                    "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                    "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                    "\t" + objectName;

                            textStream << (lineOut + "\n");

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


                if (pointsBetweenTags == 0)
                {
                    emit warningMessage("File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                               QString::number(beginningTag.sourceFileLine) +
                               ", iTOW " + QString::number(params.rovers[0].roverSyncData.upperBound(beginningUptime).value().iTOW) + ", ending tag line " +
                               QString::number(endingTag.sourceFileLine) +
                               ", iTOW " + QString::number(currentTag.iTOW) +
                               ", File \"" + endingTag.sourceFile + "\""
                               " No points between tags.");
                }

                beginningTag.iTOW = -1;
                beginningUptime = -1;
            }
        }
    }

    if (beginningTag.iTOW != -1)
    {
        emit warningMessage("File \"" + beginningTag.sourceFile + "\", line " +
                   QString::number(beginningTag.sourceFileLine) +
                   ", iTOW " + QString::number(beginningTag.iTOW) +
                   " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
    }

    stylusTipDistanceFromRoverA = params.initialStylusTipDistanceFromRoverA;

    emit infoMessage("Processing script...");
    textStream << "// Frame type\tiTOW"
                  "\tTip_X\tTip_Y\tTip_Z"
                  "\tRoverA_X\tRoverA_Y\tRoverA_Z"
                  "\tRoverB_X\tRoverB_Y\tRoverB_Z"
                  "\tTip_acc_X\tTip_Acc_Y\tTip_Acc_Z"
                  "\tRoverA_acc_X\tRoverA_Acc_Y\tRoverA_Acc_Z"
                  "\tRoverB_acc_X\tRoverB_Acc_Y\tRoverB_Acc_Z"
                  "\tCamera_X\tCamera_Y\tCamera_Z"
                  "\tLookAt_X\tLookAt_Y\tLookAt_X\tTipPositionValidity\n";

    UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = params.iTOWRange_Script_Min;
    UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = params.iTOWRange_Script_Max;

    iTOWRange_Script_Min -= iTOWRange_Script_Min % params.expectedITOWAlignment; // Round to previous aligned ITOW

    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = params.rovers[0].relposnedMessages.lowerBound(iTOWRange_Script_Min);
    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = params.rovers[1].relposnedMessages.lowerBound(iTOWRange_Script_Min);

    UBXMessage_RELPOSNED::ITOW startingITOW = (relposIterator_RoverA.value().iTOW > relposIterator_RoverB.value().iTOW) ?
                relposIterator_RoverA.value().iTOW : relposIterator_RoverB.value().iTOW;

    startingITOW -= startingITOW % params.expectedITOWAlignment; // Should not be needed, but just to be sure...

    int frameCounter = 0;

    UBXMessage_RELPOSNED::ITOW iTOW = (frameCounter * 1000) / params.fps + startingITOW;

    UBXMessage_RELPOSNED::ITOW lastRoverANagITOW = -1;
    UBXMessage_RELPOSNED::ITOW lastRoverBNagITOW = -1;

    while ((iTOW <= iTOWRange_Script_Max) &&
           (params.rovers[0].relposnedMessages.upperBound(iTOW) != params.rovers[0].relposnedMessages.end()) &&
           (params.rovers[1].relposnedMessages.upperBound(iTOW) != params.rovers[1].relposnedMessages.end()))
    {
        QString frameType;

        UBXMessage_RELPOSNED relposned_RoverA;
        UBXMessage_RELPOSNED relposned_RoverB;

        qint64 uptime = -1;

        if ((params.rovers[0].relposnedMessages.find(iTOW) != params.rovers[0].relposnedMessages.end()) &&
                (params.rovers[1].relposnedMessages.find(iTOW) != params.rovers[1].relposnedMessages.end()))
        {
            frameType = "F_Key";
            relposned_RoverA = params.rovers[0].relposnedMessages.find(iTOW).value();
            relposned_RoverB = params.rovers[1].relposnedMessages.find(iTOW).value();

            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA = params.rovers[0].reverseSync.find(iTOW);
            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB = params.rovers[1].reverseSync.find(iTOW);

            if ((reverseIter_RoverA == params.rovers[0].reverseSync.end()) &&
                    (lastRoverANagITOW != iTOW))
            {
                emit warningMessage("Uptime for rover A iTOW \"" + QString::number(iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverANagITOW = iTOW;
            }

            if ((reverseIter_RoverB == params.rovers[1].reverseSync.end()) &&
                    (lastRoverBNagITOW != iTOW))
            {
                emit warningMessage("Uptime for rover B iTOW \"" + QString::number(iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverBNagITOW = iTOW;
            }

            if ((reverseIter_RoverA != params.rovers[0].reverseSync.end()) &&
                    (reverseIter_RoverB != params.rovers[1].reverseSync.end()))
            {
                uptime = (reverseIter_RoverA.value() + reverseIter_RoverB.value()) / 2;
            }
        }
        else
        {
            frameType = "F_Interp";

            UBXMessage_RELPOSNED interpAStart = (params.rovers[0].relposnedMessages.upperBound(iTOW) - 1).value();
            UBXMessage_RELPOSNED interpAEnd = params.rovers[0].relposnedMessages.upperBound(iTOW).value();

            UBXMessage_RELPOSNED interpBStart = (params.rovers[1].relposnedMessages.upperBound(iTOW) - 1).value();
            UBXMessage_RELPOSNED interpBEnd = params.rovers[1].relposnedMessages.upperBound(iTOW).value();

            relposned_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(interpAStart, interpAEnd, iTOW);
            relposned_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(interpBStart, interpBEnd, iTOW);

            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_Start = params.rovers[0].reverseSync.find(interpAStart.iTOW);
            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_End = params.rovers[0].reverseSync.find(interpAEnd.iTOW);

            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_Start = params.rovers[1].reverseSync.find(interpBStart.iTOW);
            QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_End = params.rovers[1].reverseSync.find(interpBEnd.iTOW);

            if ((reverseIter_RoverA_Start == params.rovers[0].reverseSync.end()) &&
                    (lastRoverANagITOW != interpAStart.iTOW))
            {
                emit warningMessage("Uptime for rover A iTOW \"" + QString::number(interpAStart.iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverANagITOW = interpAStart.iTOW;
            }

            if ((reverseIter_RoverA_End == params.rovers[0].reverseSync.end()) &&
                    (lastRoverANagITOW != interpAEnd.iTOW))
            {
                emit warningMessage("Uptime for rover A iTOW \"" + QString::number(interpAEnd.iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverANagITOW = interpAEnd.iTOW;
            }

            if ((reverseIter_RoverB_Start == params.rovers[1].reverseSync.end()) &&
                    (lastRoverBNagITOW != interpBStart.iTOW))
            {
                emit warningMessage("Uptime for rover B iTOW \"" + QString::number(interpBStart.iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverBNagITOW = interpBStart.iTOW;
            }

            if ((reverseIter_RoverB_End == params.rovers[1].reverseSync.end()) &&
                    (lastRoverBNagITOW != interpBEnd.iTOW))
            {
                emit warningMessage("Uptime for rover B iTOW \"" + QString::number(interpBEnd.iTOW) +
                           " not found in sync data. Distance can not be synced.");

                lastRoverBNagITOW = interpBEnd.iTOW;
            }

            if ((reverseIter_RoverA_Start != params.rovers[0].reverseSync.end()) &&
                    (reverseIter_RoverA_End != params.rovers[0].reverseSync.end()) &&
                    (reverseIter_RoverB_Start != params.rovers[1].reverseSync.end()) &&
                    (reverseIter_RoverB_End != params.rovers[1].reverseSync.end()))
            {
                double startITOWAvg = (interpAStart.iTOW + interpBStart.iTOW) / 2;
                double endITOWAvg = (interpAEnd.iTOW + interpBEnd.iTOW) / 2;

                double fraction = static_cast<double>((iTOW - startITOWAvg)) /
                        (endITOWAvg - startITOWAvg);

                uptime = (reverseIter_RoverA_Start.value() + reverseIter_RoverB_Start.value()) / 2 +
                        fraction *
                        (((reverseIter_RoverA_End.value() - reverseIter_RoverA_Start.value()) +
                         (reverseIter_RoverB_End.value() - reverseIter_RoverB_Start.value())) / 2);
            }
        }

        bool distanceValid = false;

        QMap<qint64, PostProcessingForm::DistanceItem>::const_iterator distIter = params.distances->upperBound(uptime);

        if ((distIter != params.distances->cend()) && (uptime != -1))
        {
            QMap<qint64, PostProcessingForm::DistanceItem>::const_iterator nextDistIter = distIter;

            if (distIter != params.distances->cbegin())
            {
                distIter--;

                QMap<qint64, PostProcessingForm::DistanceItem>::const_iterator prevDistIter = distIter;

                if (prevDistIter.value().type == PostProcessingForm::DistanceItem::Type::CONSTANT)
                {
                    distanceValid = true;
                    stylusTipDistanceFromRoverA = prevDistIter.value().distance;
                }
                else if ((prevDistIter.value().type == PostProcessingForm::DistanceItem::Type::MEASURED) &&
                         (nextDistIter.value().type == PostProcessingForm::DistanceItem::Type::MEASURED))
                {
                    // Interpolate between measured distances

                    double fraction = static_cast<double>((uptime - prevDistIter.key())) /
                            (nextDistIter.key() - prevDistIter.key());

                    stylusTipDistanceFromRoverA = prevDistIter.value().distance +
                            fraction * (nextDistIter.value().distance - prevDistIter.value().distance);

                    if ((uptime - prevDistIter.key()) < 500)
                    {
                        distanceValid = true;
                    }
                }
                else if (prevDistIter.value().type == PostProcessingForm::DistanceItem::Type::MEASURED)
                {
                    // No more measured distances -> Use the last one

                    stylusTipDistanceFromRoverA = prevDistIter.value().distance;

                    if ((uptime - prevDistIter.key()) < 500)
                    {
                        distanceValid = true;
                    }
                }
                else
                {
                    emit warningMessage("File \"" + distIter.value().sourceFile + "\", line " +
                               QString::number(distIter.value().sourceFileLine)+
                               ", uptime " + QString::number(distIter.key()) +
                               ": Unknown distance type between measured ones. Skipped.");
                }
            }
        }

        Eigen::Vector3d roverAPosNED(
                relposned_RoverA.relPosN,
                relposned_RoverA.relPosE,
                relposned_RoverA.relPosD);

        Eigen::Vector3d roverBPosNED(
                relposned_RoverB.relPosN,
                relposned_RoverB.relPosE,
                relposned_RoverB.relPosD);

        Eigen::Vector3d roverBToAVecNormalizedNED = (roverAPosNED - roverBPosNED).normalized();

        Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalizedNED * stylusTipDistanceFromRoverA;

        Eigen::Vector3d roverAAccNED(
                    relposned_RoverA.accN,
                    relposned_RoverA.accE,
                    relposned_RoverA.accD);

        Eigen::Vector3d roverBAccNED(
                    relposned_RoverB.accN,
                    relposned_RoverB.accE,
                    relposned_RoverB.accD);

        Eigen::Vector3d roverAPosXYZ = *params.transform * roverAPosNED;
        Eigen::Vector3d roverBPosXYZ = *params.transform * roverBPosNED;
        Eigen::Vector3d stylusTipPosXYZ = *params.transform * stylusTipPosNED;

        Eigen::Vector3d roverAAccXYZ = transform_NoTranslation * roverAAccNED;
        Eigen::Vector3d roverBAccXYZ = transform_NoTranslation * roverBAccNED;

        // Use accuracies of rover A (used for stylus tip accuracy)
        // Could calculate some kind of "worst case" scenario using both rovers,
        // but probably errors are mostly common to both of them.
        Eigen::Vector3d stylusTipAccXYZ = roverAAccXYZ;

        Eigen::Vector3d downVecNED(0,0,1);
        Eigen::Vector3d stylusForwardAxis = roverBToAVecNormalizedNED;
        Eigen::Vector3d stylusRightAxis = -(roverBToAVecNormalizedNED.cross(downVecNED).normalized());
        Eigen::Vector3d stylusDownAxis = roverBToAVecNormalizedNED.cross(stylusRightAxis).normalized();

        Eigen::Vector3d cameraPosNED = roverAPosNED +
                stylusForwardAxis * params.cameraNShift +
                stylusRightAxis * params.cameraEShift +
                stylusDownAxis * params.cameraDShift;

        Eigen::Vector3d cameraPosXYZ = *params.transform * cameraPosNED;

        Eigen::Vector3d lookAtPosNED = roverAPosNED +
                stylusForwardAxis * params.lookAtNShift +
                stylusRightAxis * params.lookAtEShift +
                stylusDownAxis * params.lookAtDShift;

        Eigen::Vector3d lookAtPosXYZ = *params.transform * lookAtPosNED;

        QString *stylusTipLocationValidityString = new QString;

        if (distanceValid)
        {
            *stylusTipLocationValidityString = "Valid";
        }
        else
        {
            *stylusTipLocationValidityString = "Invalid";
        }

        QString lineOut =
                frameType +
                "\t" + QString::number(iTOW) +
                "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                "\t" + QString::number(roverAPosXYZ(0), 'f', 4) +
                "\t" + QString::number(roverAPosXYZ(1), 'f', 4) +
                "\t" + QString::number(roverAPosXYZ(2), 'f', 4) +
                "\t" + QString::number(roverBPosXYZ(0), 'f', 4) +
                "\t" + QString::number(roverBPosXYZ(1), 'f', 4) +
                "\t" + QString::number(roverBPosXYZ(2), 'f', 4) +
                "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                "\t" + QString::number(roverAAccXYZ(0), 'f', 4) +
                "\t" + QString::number(roverAAccXYZ(1), 'f', 4) +
                "\t" + QString::number(roverAAccXYZ(2), 'f', 4) +
                "\t" + QString::number(roverBAccXYZ(0), 'f', 4) +
                "\t" + QString::number(roverBAccXYZ(1), 'f', 4) +
                "\t" + QString::number(roverBAccXYZ(2), 'f', 4) +
                "\t" + QString::number(cameraPosXYZ(0), 'f', 4) +
                "\t" + QString::number(cameraPosXYZ(1), 'f', 4) +
                "\t" + QString::number(cameraPosXYZ(2), 'f', 4) +
                "\t" + QString::number(lookAtPosXYZ(0), 'f', 4) +
                "\t" + QString::number(lookAtPosXYZ(1), 'f', 4) +
                "\t" + QString::number(lookAtPosXYZ(2), 'f', 4) +
                "\t" + *stylusTipLocationValidityString;

        textStream << (lineOut + "\n");

        frameCounter++;
        iTOW = (frameCounter * 1000) / params.fps + startingITOW;
    }
    emit infoMessage("Movie script generated.");
}

}; // namespace Stylus
