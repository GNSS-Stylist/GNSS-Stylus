/*
    lidarscriptgenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

// Barn scan:
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"
#include "nanoflann/nanoflann.hpp"
#include "nanoflann/KDTreeVectorOfVectorsAdaptor.hpp"

namespace Lidar
{

void LidarScriptGenerator::generateLidarScript(const Params& params)
{
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



    // Barn scan: Add maps for "constructive geometry"

    QMultiMap<qint64, Eigen::Vector3d> geometryHitPoints;
    QMultiMap<qint64, Eigen::Vector3d> geometryShotPoints;




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
                params.loInterpolator->getInterpolatedLocationOrientationTransformMatrix_Uptime(roverUptime, averagedSync, transform_LoSolver);
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

                            // Barn scan: Add point
                            // Note: roverUptime used here so that LOScript and this use the same timing

                            geometryHitPoints.insert(roverUptime, laserHitPosAfterLOSolverTransformXYZ);
                            geometryShotPoints.insert(roverUptime, laserOriginAfterLOSolverTransformXYZ);
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
                          "\t" + QString::number(currentItem.item.angle, 'f', 4) +
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























    // Barn scan: Create file for "constructive geometry":

    emit infoMessage("Constructing \"constructive geometry\" for barn (let's see if this works...)...");

    std::string inputfile = "D:\\GNSSStylusData\\Temp\\ConstructiveObject\\Barn_Translated_480_129_631_dm.obj";
    QString outputFileBase = "D:/GNSSStylusData/Temp/ConstructiveObject/Barn_Translated_480_129_631_dm";

    tinyobj::ObjReaderConfig reader_config;
    reader_config.mtl_search_path = "./"; // Path to material files

    tinyobj::ObjReader reader;

    if (!reader.ParseFromFile(inputfile, reader_config))
    {
        if (!reader.Error().empty())
        {
            emit errorMessage(QString("TinyObjReader: ") + reader.Error().c_str());
        }
        return;
    }

    if (!reader.Warning().empty())
    {
        emit warningMessage(QString("TinyObjReader: ") + reader.Warning().c_str());
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
//    auto& materials = reader.GetMaterials();

    if (shapes.size() > 1)
    {
        emit errorMessage(QString("Only one shape allowed in obj file. Shapes: ") + QString::number(shapes.size()));
        return;
    }

    QList<qint64> uptimes = geometryHitPoints.uniqueKeys();

    QMultiMap<qint64, unsigned int> faceAddUptimes;   // <uptime, faceIndex>

    QString lastProcessString;















    // Some preparations for nanoflann:
    // TODO: Should be made prettier. Now this is super ugly.
    // First tried to use Eigen::Vector3ds here, but failed. Using std::vector<std::vector<double>> instead (for now?)...

//    QVector<Eigen::Vector3d> hitPoints;
//    std::vector<std::vector<double>> hitPoints;
    std::vector<Eigen::Vector3d> lidarHitPoints;
    std::vector<Eigen::Vector3d> lidarShotPoints;
    std::vector<std::vector<double>> lidarStdHitPoints;
    QVector<int> uptimeIndexes;
    QVector<int> hitPointIndexes;

    for (int uptimeIndex = 0; uptimeIndex < uptimes.size(); uptimeIndex++)
    {
        qint64 uptime = uptimes[uptimeIndex];

        QList<Eigen::Vector3d> uptimeHitPoints = geometryHitPoints.values(uptime);
        QList<Eigen::Vector3d> uptimeShotPoints = geometryShotPoints.values(uptime);

        for (int hitPointIndex = 0; hitPointIndex < uptimeHitPoints.size(); hitPointIndex++)
        {
            Eigen::Vector3d hitPoint = uptimeHitPoints[hitPointIndex];
// Tried to use Eigen::vector3d here:            hitPoints.push_back(hitPoint);

            std::vector<double> hitPointStdVec(3);
            hitPointStdVec[0] = hitPoint(0);
            hitPointStdVec[1] = hitPoint(1);
            hitPointStdVec[2] = hitPoint(2);

            lidarStdHitPoints.push_back(hitPointStdVec);

            lidarHitPoints.push_back(uptimeHitPoints[hitPointIndex]);
            lidarShotPoints.push_back(uptimeShotPoints[hitPointIndex]);

            uptimeIndexes.push_back(uptimeIndex);
            hitPointIndexes.push_back(hitPointIndex);
        }
    }

// Read todo above...:    typedef QVector<Eigen::Vector3d> my_vector_of_vectors_t;
    typedef std::vector<std::vector<double>> my_vector_of_vectors_t;
    typedef KDTreeVectorOfVectorsAdaptor<my_vector_of_vectors_t, double>  my_kd_tree_t;

    my_kd_tree_t mat_index(3, lidarStdHitPoints, 10 /* max leaf */);

    mat_index.index->buildIndex();

    // Init a knn search
    const size_t num_results = 1000;
    std::vector<size_t> ret_indexes(num_results);
    std::vector<double> out_dists_sqr(num_results);

    nanoflann::KNNResultSet<double> resultSet(num_results);

    resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
















    // Only one shape allowed here
    const int shapeIndex = 0;

    for (size_t faceIndex = 0; faceIndex < shapes[0].mesh.indices.size() / 3; faceIndex++)
    {
        double percentProgress = (double(faceIndex) / (shapes[0].mesh.indices.size() / 3)) * 100;
        QString progressString = "Processing, " + QString::number(percentProgress, 'f', 0) + "% done.";

        if (progressString != lastProcessString)
        {
            emit infoMessage(progressString);
            lastProcessString = progressString;
        }

        size_t numOfFaceVertices = size_t(shapes[shapeIndex].mesh.num_face_vertices[faceIndex]);

        if (numOfFaceVertices != 3)
        {
            emit errorMessage("Only triangulated mesh allowed here. Vertices in face: " + QString::number(numOfFaceVertices));
            return;
        }

        Eigen::Vector3d faceVertices[3];

        // Loop over vertices in the face.
        for (size_t vertexIndex = 0; vertexIndex < numOfFaceVertices; vertexIndex++)
        {
            // access to vertex
            tinyobj::index_t idx = shapes[shapeIndex].mesh.indices[faceIndex * 3 + vertexIndex];

            faceVertices[vertexIndex] = Eigen::Vector3d(attrib.vertices[3*size_t(idx.vertex_index)+0], attrib.vertices[3*size_t(idx.vertex_index)+1], attrib.vertices[3*size_t(idx.vertex_index)+2]);
        }

        Eigen::Vector3d faceCentroid = (1./3.) * (faceVertices[0] + faceVertices[1] + faceVertices[2]);

        Eigen::Vector3d faceNormal = (faceVertices[1] - faceVertices[0]).cross(faceVertices[2] - faceVertices[0]).normalized();

        // TODO: Is this needed on every round?
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);

 // Eigen-version:       mat_index.index->findNeighbors(resultSet, faceCentroid, nanoflann::SearchParams(10));

        std::vector<double> query_pt(3);

        query_pt[0] = faceCentroid[0];
        query_pt[1] = faceCentroid[1];
        query_pt[2] = faceCentroid[2];

        mat_index.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));

        // "Scoring system" for faces used. This prioritizes shots shot from "right side"
        // and also tries to order them better by distance etc.
        double bestScore = -1e15;
        int bestScroreIndex = 0;

        for (int i = 0; i < (int)num_results; i++)
        {
            size_t index = ret_indexes[i];

            double score = 1.0;

            Eigen::Vector3d lidarShotVector = lidarShotPoints[index] - lidarHitPoints[index];

            if ((faceNormal.dot(lidarShotVector)) > 0.0)
            {
                // Found a "ray" that is shot from the right side
                // This is the most important thing in the score so give it a high priority boost
                score *= 1.0e6;
//                score *= 1.0;   // For ground the results are better if this is not prioritized

                // So why not use that (positive) dot-product directly as a multiplier also?:
                score *= 100.0 * faceNormal.dot(lidarShotVector.normalized());
            }

            score *= 1.0 / lidarShotVector.norm();
            score *= 1.0 / sqrt(out_dists_sqr[i]);

            if (score > bestScore)
            {
                bestScore = score;
                bestScroreIndex = i;
            }
        }

        faceAddUptimes.insert(uptimes[uptimeIndexes[ret_indexes[bestScroreIndex]]], faceIndex);
    }



















#if 0

    std::unique_ptr<bool[]> addedFaces(new bool[shapes[0].mesh.num_face_vertices.size()]);
    const double maxDistanceSquared = 0.1 * 0.1;

    for (int uptimeIndex = 0; uptimeIndex < uptimes.size(); uptimeIndex++)
//    for (int uptimeIndex = 0; uptimeIndex < 0; uptimeIndex++)
    {
        double percentProgress = (double(uptimeIndex) / uptimes.size()) * 100;
        QString progressString = "Processing, " + QString::number(percentProgress, 'f', 1) + "% done.";

        if (progressString != lastProcessString)
        {
            emit infoMessage(progressString);
            lastProcessString = progressString;
        }

        qint64 uptime = uptimes[uptimeIndex];

        QList<Eigen::Vector3d> uptimeHitPoints = geometryHitPoints.values(uptime);

        for (int hitPointIndex = 0; hitPointIndex < uptimeHitPoints.size(); hitPointIndex++)
        {
            Eigen::Vector3d hitPoint = uptimeHitPoints[hitPointIndex];

            // Loop over faces(polygon)
            for (size_t faceIndex = 0; faceIndex < shapes[0].mesh.indices.size() / 3; faceIndex++)
            {
                if (addedFaces[faceIndex])
                {
                    continue;
                }

                size_t numOfFaceVertices = size_t(shapes[shapeIndex].mesh.num_face_vertices[faceIndex]);

                if (numOfFaceVertices != 3)
                {
                    emit errorMessage("Only triangulated mesh allowed here. Vertices in face: " + QString::number(numOfFaceVertices));
                    return;
                }

                Eigen::Vector3d difference = hitPoint - faceCentroids[faceIndex];

                if (difference.squaredNorm() < maxDistanceSquared)
                {
                    faceAddUptimes.insert(uptime, faceIndex);
                    addedFaces[faceIndex] = true;
                }


                // per-face material
//                    shapes[s].mesh.material_ids[f];
            }
        }
    }

#endif

    QFile vertexFile;

    vertexFile.setFileName(outputFileBase + ".vertices");

    if (!vertexFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open vertex file \"" + vertexFile.fileName() + "\"");
        return;
    }

    QDataStream vertexStream(&vertexFile);
    vertexStream.setByteOrder(QDataStream::LittleEndian);
    vertexStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    for (unsigned int i = 0; i < attrib.vertices.size(); i++)
    {
        vertexStream << attrib.vertices[i];
    }


    QFile normalFile;

    normalFile.setFileName(outputFileBase + ".normals");

    if (!normalFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open normal file \"" + normalFile.fileName() + "\"");
        return;
    }

    QDataStream normalStream(&normalFile);
    normalStream.setByteOrder(QDataStream::LittleEndian);
    normalStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    for (unsigned int i = 0; i < attrib.normals.size(); i++)
    {
        normalStream << attrib.normals[i];
    }



    QFile vertexIndexFile;

    vertexIndexFile.setFileName(outputFileBase + ".vertexindexes");

    if (!vertexIndexFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open vertex index file \"" + vertexIndexFile.fileName() + "\"");
        return;
    }

    QDataStream vertexIndexStream(&vertexIndexFile);
    vertexIndexStream.setByteOrder(QDataStream::LittleEndian);

    for (unsigned int i = 0; i < shapes[0].mesh.indices.size() / 3; i++)
    {
        // Order must be changed for Godot to get correct sides shown
        vertexIndexStream << shapes[0].mesh.indices[i * 3 + 2].vertex_index;
        vertexIndexStream << shapes[0].mesh.indices[i * 3 + 1].vertex_index;
        vertexIndexStream << shapes[0].mesh.indices[i * 3 + 0].vertex_index;
    }




// Texture coords may not be needed:
#if true

    QFile texCoordsFile;

    texCoordsFile.setFileName(outputFileBase + ".texcoords");

    if (!texCoordsFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open normal file \"" + texCoordsFile.fileName() + "\"");
        return;
    }

    QDataStream texCoordsStream(&texCoordsFile);
    texCoordsStream.setByteOrder(QDataStream::LittleEndian);
    texCoordsStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

    for (unsigned int i = 0; i < attrib.texcoords.size(); i++)
    {
        texCoordsStream << attrib.texcoords[i];
    }

    QFile texCoordIndexFile;

    texCoordIndexFile.setFileName(outputFileBase + ".texcoordindexes");

    if (!texCoordIndexFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open texture coordinate index file \"" + texCoordIndexFile.fileName() + "\"");
        return;
    }

    QDataStream texCoordIndexStream(&texCoordIndexFile);
    texCoordIndexStream.setByteOrder(QDataStream::LittleEndian);

    for (unsigned int i = 0; i < shapes[0].mesh.indices.size() / 3; i++)
    {
        // Order must be changed for Godot to get correct sides shown
        texCoordIndexStream << shapes[0].mesh.indices[i * 3 + 2].texcoord_index;
        texCoordIndexStream << shapes[0].mesh.indices[i * 3 + 1].texcoord_index;
        texCoordIndexStream << shapes[0].mesh.indices[i * 3 + 0].texcoord_index;
    }
#endif



    QFile faceSyncFile;

    faceSyncFile.setFileName(outputFileBase + ".facesync");

    if (!faceSyncFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open facesync file \"" + faceSyncFile.fileName() + "\"");
        return;
    }

    QDataStream faceSyncStream(&faceSyncFile);
    faceSyncStream.setByteOrder(QDataStream::LittleEndian);

    uptimes = faceAddUptimes.uniqueKeys();

    for (int uptimeIndex = 0; uptimeIndex < uptimes.size(); uptimeIndex++)
    {
//        qint64 uptime = uptimes[uptimeIndex];
        // 32 bit is enough here:
        int uptime = uptimes[uptimeIndex];

        QList<unsigned int> uptimeFaceIndexes = faceAddUptimes.values(uptime);

        for (int i = 0; i < uptimeFaceIndexes.size(); i++)
        {
            faceSyncStream << uptime;
            faceSyncStream << uptimeFaceIndexes[i];
        }
    }

    emit infoMessage("Constructive geometry constructed.");


}

}; // namespace Lidar

