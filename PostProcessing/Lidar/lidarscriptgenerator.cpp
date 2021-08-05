/*
    lidarscriptgenerator.cpp (part of GNSS-Stylus)
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

#include <QMessageBox>
#include <QPushButton>
#include "lidarscriptgenerator.h"

namespace Lidar
{

void LidarScriptGenerator::generateLidarScript(const Params& params)
{
    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;
    filteredItems.reserve(10000);

    RPLidarPlausibilityFilter plausibilityFilter;
    plausibilityFilter.setSettings(*params.lidarFilteringSettings);

    QFile lidarScriptFile;

    lidarScriptFile.setFileName(params.fileName);

    if (lidarScriptFile.exists())
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
            emit infoMessage("Generating lidar script cancelled.");
            return;
        }
    }

    if (!lidarScriptFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open lidar script file.");
        return;
    }

    QTextStream textStream(&lidarScriptFile);

    emit infoMessage("Processing lidar script...");

    // Add some metadata to make possible changes in the future easier
    textStream << "META\tHEADER\tGNSS-Stylus lidar script\n";
    textStream << "META\tVERSION\t1.0.0\n";
    textStream << "META\tFORMAT\tASCII\n";
    textStream << "META\tCONTENT\tDEFAULT\n";
    textStream << "META\tEND\n";

    textStream << "Uptime\tType\tDescr/subtype\t"
                  "RotAngle\t"
                  "Origin_X\tOrigin_Y\tOrigin_Z\t"
                  "Hit_X\tHit_Y\tHit_Z\n";

    QMap<qint64, PostProcessingForm::LidarRound>::const_iterator lidarIter = params.lidarRounds->upperBound(params.uptime_Min);
    QMultiMap<qint64, PostProcessingForm::Tag>::const_iterator tagIter = params.tags->begin();

    QString objectName;
    bool objectActive = false;
    bool scanningActive = false;
    bool ignoreBeginningAndEndingTags = false;
    qint64 beginningUptime = -1;
    PostProcessingForm::Tag beginningTag;

    unsigned int pointsWritten = 0;

    while ((lidarIter.key() <= params.uptime_Max) && (lidarIter != params.lidarRounds->end()))
    {
        QString previousObjectName = objectName;
        bool previousObjectActive = objectActive;
        bool previousScanningActive = scanningActive;

        while ((tagIter.key() < lidarIter.value().startTime) && tagIter != params.tags->end())
        {
            // Roll tags to the current uptime to keep track of scanning state and object name

            QList<PostProcessingForm::Tag> tagItems = params.tags->values(tagIter.key());

            PostProcessingForm::Tag currentTag = tagIter.value();

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const PostProcessingForm::Tag& currentTag = tagItems[i];

                qint64 tagUptime = tagIter.key();

                if (!(currentTag.ident.compare(params.tagIdent_BeginNewObject)))
                {
                    // Tag type: new object

                    if (currentTag.text.length() == 0)
                    {
                        // Empty name for the new object not allowed

                        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": New object without a name. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        continue;
                    }

                    objectName = currentTag.text;
                    objectActive = true;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                }
                else if ((!(currentTag.ident.compare(params.tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: Begin points

                    if (!objectActive)
                    {
                        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Duplicate beginning tag. Skipped.");
                        continue;
                    }

                    scanningActive = true;
                    beginningUptime = tagUptime;
                    beginningTag = currentTag;
                }
                else if ((!(currentTag.ident.compare(params.tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: end points

                    if (!objectActive)
                    {
                        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        emit warningMessage("File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(tagUptime) +
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

                    beginningUptime = -1;
                    scanningActive = false;
                }
            }

            if (previousObjectName != objectName)
            {
                // Note: params.timeShift used here so that LOScript and this use the same timing

                textStream << QString::number(tagIter.key() + params.timeShift) +  "\tOBJECTNAME\t" + objectName + "\n";
            }

            if (!previousObjectActive && objectActive)
            {
                textStream << QString::number(tagIter.key() + params.timeShift) + "\tSTARTOBJECT\n";
            }

            if (previousObjectActive && !objectActive)
            {
                textStream << QString::number(tagIter.key() + params.timeShift) + "\tENDOBJECT\n";
            }

            if (!previousScanningActive && scanningActive)
            {
                textStream << QString::number(tagIter.key() + params.timeShift) + "\tSTARTSCAN\n";
            }

            if (previousScanningActive && !scanningActive)
            {
                textStream << QString::number(tagIter.key() + params.timeShift) + "\tENDSCAN\n";
            }

            tagIter++;
        }

        const PostProcessingForm::LidarRound& round = lidarIter.value();

        plausibilityFilter.filter(round.distanceItems, filteredItems);

        for (int i = 0; i < filteredItems.count(); i++)
        {
            const RPLidarPlausibilityFilter::FilteredItem& currentItem = filteredItems[i];

            // Rover coordinates interpolated according to distance timestamps.

            qint64 itemUptime = round.startTime + (round.endTime - round.startTime) * i / lidarIter.value().distanceItems.count();
            UBXMessage_RELPOSNED interpolated_Rovers[3];

            qint64 roverUptime = itemUptime + params.timeShift;

            Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

            try
            {
                params.loInterpolator->getInterpolatedLocationOrientationTransformMatrix_Uptime(roverUptime, transform_LoSolver);
            }
            catch (QString& stringThrown)
            {
                emit warningMessage("File \"" + lidarIter.value().fileName + "\", chunk index " +
                           QString::number(lidarIter.value().chunkIndex)+
                           ", uptime " + QString::number(lidarIter.key()) +
                           ": " + stringThrown + " Lidar script generating terminated.");
                return;
            }

            Eigen::Transform<double, 3, Eigen::Affine> transform_LaserRotation;
            transform_LaserRotation = Eigen::AngleAxisd(currentItem.item.angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

            // Lot of parentheses here to keep all calculations as matrix * vector
            // This is _much_ faster, in quick tests time was dropped from 510 s to 295 s when using parentheses in the whole lidarscript-creation)
            Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = *params.transform_NEDToXYZ * (transform_LoSolver * (*params.transform_AfterRotation * (transform_LaserRotation * (*params.transform_BeforeRotation * Eigen::Vector3d::Zero()))));

            // Lot of parentheses here to keep all calculations as matrix * vector
            // This is _much_ faster, in quick tests time was dropped from 510 s to 295 s when using parentheses in the whole lidarscript-creation)
            Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (*params.transform_AfterRotation * (transform_LaserRotation * (*params.transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX()))));

            Eigen::Vector3d laserHitPosAfterLOSolverTransformXYZ = *params.transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;

            QString descr;

            if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
            {
                if (objectActive)
                {
                    if (scanningActive)
                    {
                        if ((laserHitPosAfterLOSolverTransform - *params.boundingSphere_Center).norm() <= params.boundingSphere_Radius)
                        {
                            descr = "H";
                        }
                        else
                        {
                            descr = "M";
                        }
                    }
                    else
                    {
                        descr = "NS";
                    }
                }
                else
                {
                    descr = "NO";
                }
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_ANGLE)
            {
                descr = "FA";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_QUALITY_PRE)
            {
                descr = "FQ1";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_QUALITY_POST)
            {
                descr = "FQ2";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_NEAR)
            {
                descr = "FDN";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_FAR)
            {
                descr = "FDF";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_DISTANCE_DELTA)
            {
                descr = "FDD";
            }
            else if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_SLOPE)
            {
                descr = "FS";
            }
            else
            {
                descr = "F?";
            }

            // Note: roverUptime used here so that LOScript and this use the same timing

            textStream << QString::number(roverUptime) + "\tL\t" + descr +
                          "\t" + QString::number(currentItem.item.angle, 'f', 2) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(0), 'f', 4) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(1), 'f', 4) +
                          "\t" + QString::number(laserOriginAfterLOSolverTransformXYZ(2), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                          "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4) + "\n";

            pointsWritten++;
        }

        lidarIter++;
    }

    emit infoMessage("Lidar script generated. Number of points: " + QString::number(pointsWritten));
}

}; // namespace Lidar

