/*
    postprocessform.h (part of GNSS-Stylus)
    Copyright (C) 2019-2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file postprocessform.h
 * @brief Declaration for a form that allows post-processing based on the logged data.
 */

#ifndef POSTPROCESSFORM_H
#define POSTPROCESSFORM_H

#include <QTime>
#include <QWidget>
#include <QMap>
#include <QFileDialog>
#include <QTimer>
#include <QElapsedTimer>
#include <QVector3D>
#include <QTextStream>

#include "gnssmessage.h"
#include "ubloxdatastreamprocessor.h"
#include "Eigen/Geometry"
#include "losolver.h"
#include "Lidar/rplidarthread.h"
#include "Lidar/rplidarplausibilityfilter.h"

namespace Ui {
class PostProcessingForm;
}

/**
 * @brief Declaration for a form that allows post-processing based on the logged data.
 *
 * This class operates entirely on the rovers' RELPOSNED-data and tags.
 * These are read to relposnedMessages_RoverA, relposnedMessages_RoverA and tags-maps.
 *
 * Interpreting of the rovers' RELPOSNED-data is done by UBloxDataStreamProcessor.
 */
class PostProcessingForm : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Tag-class stores the individual tag ("tag" here means items in tag-file)
     */
    class Tag
    {
    public:
        int iTOW = -1;
        QString sourceFile;     //!< Filename of the source file
        int sourceFileLine = 0; //!< Line of the source file where this tag was read from
        QString ident;          //!< Identifier
        QString text;           //!< Additional textual info (for example a name for a new object)
    };

    class DistanceItem
    {
    public:
        double distance = 0;

        enum Type
        {
            UNKNOWN = 0,
            CONSTANT,
            MEASURED,
        } type = UNKNOWN;

        QString sourceFile;     //!< Filename of the source file
        int sourceFileLine = 0;     //!< Line of the source file where this tag was read from
        int frameDuration = 0;
    };

    explicit PostProcessingForm(QWidget *parent = nullptr); //!< Constructor
    ~PostProcessingForm();

protected:
    void showEvent(QShowEvent* event);  //!< Initializes some things that can't be initialized in constructor

private slots:

    void on_replayTimerTimeout();

    void on_pushButton_ClearRELPOSNEDData_RoverA_clicked();

    void on_pushButton_ClearRELPOSNEDData_RoverB_clicked();

    void on_pushButton_ClearTagData_clicked();

    void on_pushButton_AddRELPOSNEDData_RoverA_clicked();

    void on_pushButton_ClearAll_clicked();

    void on_pushButton_AddRELPOSNEDData_RoverB_clicked();

    void on_pushButton_AddTagData_clicked();

    void on_pushButton_Stylus_GeneratePointClouds_clicked();

    void on_pushButton_StartReplay_clicked();

    void on_pushButton_StopReplay_clicked();

    void on_pushButton_ContinueReplay_clicked();

    void on_pushButton_Stylus_Movie_GenerateScript_clicked();

    // Slots for UBloxDataStreamProcessor
    void ubloxProcessor_nmeaSentenceReceived(const NMEAMessage& nmeaSentence);
    void ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage);
    void ubloxProcessor_rtcmMessageReceived(const RTCMMessage& rtcmMessage);
    void ubloxProcessor_ubxParseError(const QString& errorString);
    void ubloxProcessor_nmeaParseError(const QString& errorString);
    void ubloxProcessor_unidentifiedDataReceived(const QByteArray& data);

    void on_pushButton_ClearDistanceData_clicked();

    void on_pushButton_AddDistanceData_clicked();

    void on_pushButton_ClearSyncData_clicked();

    void on_pushButton_AddSyncData_clicked();

    void on_pushButton_GenerateSyncDataBasedOnITOWS_clicked();

    void on_pushButton_ClearAllFileData_clicked();

    void on_pushButton_AddAll_clicked();

    void on_pushButton_LoadTransformation_clicked();

    void on_pushButton_SaveTransformation_clicked();

    void on_pushButton_AddAllIncludingTransform_clicked();

    void on_pushButton_Preset_clicked();

    void on_pushButton_ClearRELPOSNEDData_RoverC_clicked();

    void on_pushButton_AddRELPOSNEDData_RoverC_clicked();

    void on_pushButton_LoadAntennaLocations_clicked();

    void on_pushButton_SaveAntennaLocations_clicked();

    void on_pushButton_ValidateAntennaLocations_clicked();

    void on_pushButton_LOSolver_GenerateScript_clicked();

    void on_pushButton_AddLidarData_clicked();

    void on_pushButton_ClearLidarData_clicked();

    void on_pushButton_Lidar_GeneratePointClouds_clicked();

    void on_pushButton_Lidar_GenerateScript_clicked();

private:
    /**
     * @brief RELPOSNEDReadingData-class is used to make it easier to handle processing if RELPOSNED-data
     * (RELPOSNED-data is read using UBloxDataStreamProcessor that emits the data.)
     */
    class RELPOSNEDReadingData
    {
    public:
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>* relposnedMessages;  //!< Pointer to RELPOSNED-data where new items should be added

        int messageCount_UBX;                       //!< Total number of UBX-messages
        int messageCount_NMEA;                      //!< Total number of NMEA-messages
        int messageCount_RTCM;                      //!< Total numner of RTCM-messages
        int messageCount_UBX_RELPOSNED_Total;       //!< Total number of RELPOSNED-messages
        int messageCount_UBX_RELPOSNED_UniqueITOWs; //!< Number of RELPOSNED-messages with unique iTOW

        int lastReadITOW;                   //!< iTOW of the last RELPOSNED-message
        int firstDuplicateITOW;             //!< First duplicate iTOW in this "batch" of duplicate iTOWS
        int firstDuplicateITOWByteIndex;    //!< Byte index (in file) of the first duplicate RELPOSNED iTOW
        int duplicateITOWCounter;           //!< Number of RELPOSNED-messages with duplicate iTOWS in this "batch"

        int currentFileByteIndex;           //!< Byte index (in file)
        int lastHandledDataByteIndex;       //!< Byte index of the last processed byte
        int discardedBytesCount;            //!< Total number of bytes that could not be interpreted as any of the supported message types

        void init();
    };

    RELPOSNEDReadingData currentRELPOSNEDReadingData;   // For UBloxDataProcessor callbacks

    Ui::PostProcessingForm *ui;

    QMultiMap<qint64, Tag> tags;     //!< Tags read from a file

    QMap<qint64, DistanceItem> distances;

    class RoverSyncItem
    {
    public:
        typedef enum
        {
            MSGTYPE_UNKNOWN = 0,
            MSGTYPE_UBX_RELPOSNED,
        } MessageType;

        QString sourceFile;     //!< Filename of the source file
        int sourceFileLine = 0;     //!< Line of the source file where this tag was read from
        MessageType messageType = MSGTYPE_UNKNOWN;
        UBXMessage_RELPOSNED::ITOW iTOW = -1;
        qint64 frameTime = 0;
    };

    class Rover
    {
    public:
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED> relposnedMessages;
        QMap<qint64, RoverSyncItem> roverSyncData;
        QMap<UBXMessage_RELPOSNED::ITOW, qint64> reverseSync;
    };

    Rover rovers[3];

    class LidarRound
    {
    public:
        QString fileName;
        int chunkIndex = -1;
        qint64 startTime = -1;
        qint64 endTime = -1;
        QVector<RPLidarThread::DistanceItem> distanceItems;
    };

    class LOInterpolator
    {
    public:
        LOInterpolator(PostProcessingForm* owner);
        void getInterpolatedLocationOrientationTransformMatrix(const qint64 uptime, Eigen::Transform<double, 3, Eigen::Affine>& transform);

        LOSolver loSolver;  // This must be initialized by user of this class before using the interpolation function!

    private:
        PostProcessingForm* owner = nullptr;
        qint64 roverUptimeLimits[3][2];
        UBXMessage_RELPOSNED roverRELPOSNEDS_Lower[3];
        UBXMessage_RELPOSNED roverRELPOSNEDS_Upper[3];
    };

    QMap<qint64, LidarRound> lidarRounds;

    bool onShowInitializationsDone = false;
    QFileDialog fileDialog_UBX;
    QFileDialog fileDialog_Tags;
    QFileDialog fileDialog_Distances;
    QFileDialog fileDialog_Sync;
    QFileDialog fileDialog_Lidar;
    QFileDialog fileDialog_All;

    QFileDialog fileDialog_Transformation_Load;
    QFileDialog fileDialog_Transformation_Save;

    QFileDialog fileDialog_AntennaLocations_Load;
    QFileDialog fileDialog_AntennaLocations_Save;

    QFileDialog fileDialog_PointCloud;
    QFileDialog fileDialog_Stylus_MovieScript;
    QFileDialog fileDialog_LOSolver_Script;
    QFileDialog fileDialog_Lidar_Script;

    // Replay:
    qint64 firstUptimeToReplay = 0;
    qint64 lastUptimeToReplay = std::numeric_limits < qint64 >::max();
    qint64 lastReplayedUptime_ms = 0;
    QElapsedTimer replayTimeElapsedTimer;
    qint64 cumulativeRequestedWaitTime_ns;
    bool stopReplayRequest = false;

    void addLogLine(const QString& line);
    void addRELPOSNEDData_Rover(const unsigned int roverId);
    void addRELPOSNEDData_Rover(const QStringList fileNames, const unsigned int roverId);
    void addRELPOSNEDFileData(const QStringList& fileNames);
    void handleReplay(bool firstRound);

    void addTagData(const QStringList& fileNames);
    void addDistanceData(const QStringList& fileNames);
    void addSyncData(const QStringList& fileNames);
    void addLidarData(const QStringList& fileNames);
    void addAllData(const bool includeTransformation);

    void loadTransformation(const QString fileName);

    QStringList getAppendedFileNames(const QStringList& fileNames, const QString appendix);

    qint64 getNextUptime(const qint64 uptime);
    qint64 getFirstUptime();
    qint64 getLastUptime();

    bool generateTransformationMatrix(Eigen::Transform<double, 3, Eigen::Affine>& outputMatrix);

    QString getRoverIdentString(const unsigned int roverId);

    void loadAntennaLocations(const QString fileName);

    bool updateLOSolverReferencePointLocations(LOSolver& loSolver);

    void syncLogFileDialogDirectories(const QString dir, const bool savesetting);

    bool generatePointCloudPointSet_Stylus(const Tag& beginningTag, const Tag& endingTag,
                                           const qint64 beginningUptime, const qint64 endingUptime,
                                           QTextStream* outStream,
                                           const Eigen::Transform<double, 3, Eigen::Affine>& transform,
                                           int& pointsWritten);

    bool generatePointCloudPointSet_Lidar(const Tag& beginningTag, const Tag& endingTag,
                                           const qint64 beginningUptime, const qint64 endingUptime,
                                           QTextStream* outStream,
                                           const Eigen::Transform<double, 3, Eigen::Affine>& transform_NEDToXYZ,
                                           const Eigen::Transform<double, 3, Eigen::Affine>& transform_BeforeRotation,
                                           const Eigen::Transform<double, 3, Eigen::Affine>& transform_AfterRotation,
                                           LOInterpolator& loInterpolator, const RPLidarPlausibilityFilter::Settings& filteringSettings,
                                           int& pointsWritten);

    typedef enum
    {
        SOURCE_LASERDISTANCEMETER_OR_CONSTANT = 0, //!< Laser distance distance meter or constant as a fallback
        SOURCE_LIDAR                            //!< Lidar
    } PointCloudDistanceSource;

    void generatePointClouds(const PointCloudDistanceSource source);

    void getLidarFilteringSettings(RPLidarPlausibilityFilter::Settings& lidarFilteringSettings);

    bool generateLidarTransformMatrices(Eigen::Transform<double, 3, Eigen::Affine>& transform_Lidar_Generated_BeforeRotation,
                                        Eigen::Transform<double, 3, Eigen::Affine>& transform_LidarGenerated_AfterRotation);

signals:
    void replayData_Rover(const UBXMessage&, const unsigned int roverId);  //!< New data for rover

    // QT's signals and slots need to be defined exactly the same way, therefore PostProcessingForm::Tag
    void replayData_Tag(const qint64, const PostProcessingForm::Tag&); //!< New tag

    void replayData_Distance(const qint64, const PostProcessingForm::DistanceItem&); //!< New distance

    void replayData_Lidar(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64);
};

#endif // POSTPROCESSFORM_H
