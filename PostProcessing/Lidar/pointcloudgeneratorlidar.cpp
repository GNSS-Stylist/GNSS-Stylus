/*
    pointcloudgeneratorlidar.cpp (part of GNSS-Stylus)
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

#include "pointcloudgeneratorlidar.h"

namespace Lidar
{

void PointCloudGenerator::generatePointClouds(const Params& params)
{
    // This function is identical to the one used in Stylus' PointCloudGenerator.
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
    QMap<qint64, PostProcessingForm::LidarRound>::const_iterator lidarIter = params.lidarRounds->upperBound(beginningUptime);

    // As lidar rounds are "mapped" according to their arriving (=end) timestamps,
    // roll here to the first one with a bigger starting timestamp
    // to prevent taking "past" measurements into account

    while ((lidarIter != params.lidarRounds->end()) && (lidarIter.value().startTime < beginningUptime))
    {
        lidarIter++;
    }

    int pointsBetweenTags = 0;

    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;
    filteredItems.reserve(10000);

    RPLidarPlausibilityFilter plausibilityFilter;

    plausibilityFilter.setSettings(*params.lidarFilteringSettings);

    // Map where uptimes for all equal ITOWs are the same.
    // This makes processing later easier
    // Uptimes here are calculated as averages from rover values (for each ITOW)
    QMap<qint64, UBXMessage_RELPOSNED::ITOW> averagedSync;

    emit infoMessage("Generating equalized rover uptime timestamps...");
    PostProcessingForm::generateAveragedRoverUptimeSync(params.rovers, averagedSync);
    emit infoMessage("Equalized rover uptime timestamps created. Number of items: " + QString::number(averagedSync.size()));

    while ((lidarIter != params.lidarRounds->end()) && (lidarIter.value().startTime < endingUptime))
    {
        plausibilityFilter.filter(lidarIter.value().distanceItems, filteredItems);

        // Q_ASSERT(lidarIter.value().distanceItems.count() == filteredItems.count());

        const PostProcessingForm::LidarRound& round = lidarIter.value();

        for (int i = 0; i < filteredItems.count(); i++)
        {
            const RPLidarPlausibilityFilter::FilteredItem& currentItem = filteredItems[i];

            if (currentItem.type == RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED)
            {
                // Rover coordinates interpolated according to distance timestamps.

                qint64 itemUptime = round.startTime + (round.endTime - round.startTime) * i / lidarIter.value().distanceItems.count();
                UBXMessage_RELPOSNED interpolated_Rovers[3];

                qint64 roverUptime = itemUptime + params.timeShift;

                Eigen::Transform<double, 3, Eigen::Affine> transform_LoSolver;

                try
                {
                    params.loInterpolator->getInterpolatedLocationOrientationTransformMatrix_Uptime(roverUptime, averagedSync, transform_LoSolver);
                }
                catch (QString& stringThrown)
                {
                    emit warningMessage("File \"" + lidarIter.value().fileName + "\", chunk index " +
                               QString::number(lidarIter.value().chunkIndex)+
                               ", uptime " + QString::number(lidarIter.key()) +
                               ": " + stringThrown + " Skipped the rest of this set of points " +
                               "between tags in lines " + QString::number(beginningTag.sourceFileLine) + " and " +
                               QString::number(endingTag.sourceFileLine) +
                               " in file \"" + beginningTag.sourceFile + "\".");
                    return(false);
                }


                Eigen::Transform<double, 3, Eigen::Affine> transform_LaserRotation;
                transform_LaserRotation = Eigen::AngleAxisd(currentItem.item.angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = *params.transform_NEDToXYZ * (transform_LoSolver * (*params.transform_AfterRotation * (transform_LaserRotation * (*params.transform_BeforeRotation * Eigen::Vector3d::Zero()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserOriginBeforeRotation = transform_BeforeRotation * Eigen::Vector3d::Zero();
                Eigen::Vector3d laserOriginAfterRotation = transform_LaserRotation * laserOriginBeforeRotation;
                Eigen::Vector3d laserOriginAfterPostRotationTransform = transform_AfterRotation * laserOriginAfterRotation;
                Eigen::Vector3d laserOriginAfterLOSolverTransform = transform_LoSolver * laserOriginAfterPostRotationTransform;
                Eigen::Vector3d laserOriginAfterLOSolverTransformXYZ = transform_NEDToXYZ * laserOriginAfterLOSolverTransform;
                */

                // Lot of parentheses here to keep all calculations as matrix * vector
                // This is _much_ faster, in quick tests time was dropped from 44 s to 24 s when using parentheses in the whole pointcloud-creation)
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * (*params.transform_AfterRotation * (transform_LaserRotation * (*params.transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX()))));

                /* "Step by step"-versions of the calculations above for possible debugging/tuning in the future:
                Eigen::Vector3d laserVectorBeforeRotation = transform_BeforeRotation * (currentItem.item.distance * Eigen::Vector3d::UnitX());
                Eigen::Vector3d laserVectorAfterRotation = transform_LaserRotation * laserVectorBeforeRotation;
                Eigen::Vector3d laserVectorAfterPostRotationTransform = transform_AfterRotation * laserVectorAfterRotation;
                Eigen::Vector3d laserHitPosAfterLOSolverTransform = transform_LoSolver * laserVectorAfterPostRotationTransform;
                */

                if ((laserHitPosAfterLOSolverTransform - *params.boundingSphere_Center).norm() <= params.boundingSphere_Radius)
                {
                    Eigen::Vector3d laserHitPosAfterLOSolverTransformXYZ = *params.transform_NEDToXYZ * laserHitPosAfterLOSolverTransform;

                    Eigen::Vector3d normal = (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).normalized();

                    if (params.normalLengthsAsQuality)
                    {
                        normal = (1. / (laserOriginAfterLOSolverTransformXYZ - laserHitPosAfterLOSolverTransformXYZ).norm()) * normal;
                    }

                    QString lineOut;
                    if (params.includeNormals)
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4) +
                                "\t" + QString::number(normal(0), 'f', 4) +
                                "\t" + QString::number(normal(1), 'f', 4) +
                                "\t" + QString::number(normal(2), 'f', 4);
                    }
                    else
                    {
                        lineOut = QString::number(laserHitPosAfterLOSolverTransformXYZ(0), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(1), 'f', 4) +
                                "\t" + QString::number(laserHitPosAfterLOSolverTransformXYZ(2), 'f', 4);
                    }

                    outStream->operator<<(lineOut + "\n");
                    pointsWritten++;
                    pointsBetweenTags++;
                }
            }
        }

        lidarIter++;
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


}; // namespace Lidar
