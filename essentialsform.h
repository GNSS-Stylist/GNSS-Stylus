/*
    essentialsform.h (part of GNSS-Stylus)
    Copyright (C) 2019 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file essentialsform.h
 * @brief Declaration for a form that allows logging of signals/tags and has some "essential" things for realtime operation.
 */

#ifndef ESSENTIALSFORM_H
#define ESSENTIALSFORM_H

#include <QWidget>
#include <QFile>
#include <QTextStream>
#include <QSoundEffect>
#include <QTreeWidgetItem>
#include <cmath>

#include "serialthread.h"
#include "ntripthread.h"
#include "ubloxdatastreamprocessor.h"
#include "gnssmessage.h"
#include "postprocessform.h"
#include "laserrangefinder20hzv2serialthread.h"

namespace Ui {
class EssentialsForm;
}

/**
 * @brief Form that allows logging of signals/tags and has some "essential" things for realtime operation.
 */
class EssentialsForm : public QWidget
{
    Q_OBJECT

public:
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

        qint64 frameStartTime = 0;
        qint64 frameEndTime = 0;
    };

    explicit EssentialsForm(QWidget *parent = nullptr); //!< Constructor
    ~EssentialsForm();

    void connectSerialThreadSlots_Base(SerialThread* serThread); //!< Connects signals from SerialThread (base)
    void disconnectSerialThreadSlots_Base(SerialThread* serThread); //!< Disconnects signals from SerialThread (base)
    void connectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor (base). Works for serial and NTRIP
    void disconnectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor (base). Works for serial and NTRIP

    void connectNTRIPThreadSlots_Base(NTRIPThread* serThread); //!< Connects signals from NTRIPThread
    void disconnectNTRIPThreadSlots_Base(NTRIPThread* serThread); //!< Disconnects signals from NTRIPThread

    void connectSerialThreadSlots_Rover(SerialThread* serThread, const unsigned int roverId); //!< Connects signals from SerialThread
    void disconnectSerialThreadSlots_Rover(SerialThread* serThread, const unsigned int roverId); //!< Disconnects signals from SerialThread
    void connectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor, const unsigned int roverId); //!< Connects signals from UBloxDataStreamProcessor
    void disconnectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor, const unsigned int roverId); //!< Disconnects signals from UBloxDataStreamProcessor

    void connectPostProcessingSlots(PostProcessingForm* postProcessingForm); //!< Connects signals from PostProcessingForm
    void disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm); //!< Disconnects signals from PostProcessingForm

    void connectLaserRangeFinder20HzV2SerialThreadSlots(LaserRangeFinder20HzV2SerialThread* distanceThread); //!< Connects signals from "Laser range finder 20 Hz V2"-thread
    void disconnectLaserRangeFinder20HzV2SerialThreadSlots(LaserRangeFinder20HzV2SerialThread* distanceThread); //!< Disconnects signals from "Laser range finder 20 Hz V2"-thread

public slots:
    void on_distanceReceived(const EssentialsForm::DistanceItem& item);
    void on_measuredDistanceReceived(const double& distance, qint64 frameStartTime, qint64 frameEndTime);

private slots:
    void on_pushButton_StartLogging_clicked();
    void on_pushButton_StopLogging_clicked();

    void dataReceived_Base(const QByteArray& bytes);
    void nmeaSentenceReceived_Base(const NMEAMessage& nmeaSentence);
    void ubxMessageReceived_Base(const UBXMessage& ubxMessage);
    void rtcmMessageReceived_Base(const RTCMMessage& rtcmMessage);

    void serialDataReceived_Rover(const QByteArray& bytes, const unsigned int roverId);
    void nmeaSentenceReceived_Rover(const NMEAMessage& nmeaSentence, const unsigned int roverId);
    void ubxMessageReceived_Rover(const UBXMessage& ubxMessage, const unsigned int roverId);
    void postProcessingTagReceived(const qint64 uptime, const PostProcessingForm::Tag& tag);
    void postProcessingDistanceReceived(const qint64, const PostProcessingForm::DistanceItem&);

    void on_pushButton_AddTag_clicked();

    void on_pushButton_MouseTag_clicked();
    void on_pushButton_MouseTag_rightClicked();
    void on_pushButton_MouseTag_middleClicked();

    void on_spinBox_FluctuationHistoryLength_valueChanged(int);

    void on_horizontalScrollBar_Volume_MouseButtonTagging_valueChanged(int value);

    void on_horizontalScrollBar_Volume_DistanceReceived_valueChanged(int value);

    void on_checkBox_PlaySound_stateChanged(int arg1);

protected:
    void showEvent(QShowEvent* event);  //!< To initialize some things

private:

    class Rover
    {
    public:
        QFile logFile_Raw;       //!< Log file for all serial data
        QFile logFile_NMEA;      //!< Log file for NMEA serial data
        QFile logFile_UBX;       //!< Log file for UBX serial data
        QFile logFile_RELPOSNED; //!< Log file for UBX-RELPOSNED serial data

        QMultiMap<SerialThread*, QMetaObject::Connection> serialThreadConnections;
        QMultiMap<UBloxDataStreamProcessor*, QMetaObject::Connection> ubloxDataStreamProcessorConnections;

        QQueue<UBXMessage_RELPOSNED> messageQueue_RELPOSNED; //!< Message queue for RELPOSNED-messages (used to sync rovers)

        UBXMessage_RELPOSNED lastMatchingRoverRELPOSNED;   //!< Last RELPOSNED-message with a matching iTOW with other rovers

        QList<UBXMessage_RELPOSNED> positionHistory;         //!< Used to calculate fluctuation of rover's position
        double distanceBetweenFarthestCoordinates = nan("");     //!< Distance calculated between min/max coordinate values for all NED-axes during spinBox_FluctuationHistoryLength
    };

    Rover rovers[2];

    /**
     * @brief Small helper class to store coordinates, accuracies and ITOW/uptime
     */
    class NEDPoint
    {
        public:

        bool valid = false;

        UBXMessage_RELPOSNED::ITOW iTOW = -1;   //!< GNSS Time Of Week (-1 if not applicable)
        qint64 uptime = 0;                      //!< Uptime (QElapsedTimer->msecsSinceReference())

        double n = 0;       //!< North (m)
        double e = 0;       //!< East (m)
        double d = 0;       //!< Down (m)

        double accN = -1;   //!< Accuracy of North-component (m)
        double accE = -1;   //!< Accuracy of North-component (m)
        double accD = -1;   //!< Accuracy of North-component (m)

        NEDPoint(const UBXMessage_RELPOSNED relposnedMessage);
        NEDPoint() = default;

        double getDistanceTo(const NEDPoint& other);
    };

    Ui::EssentialsForm *ui;

    QFile logFile_Base_Raw;         //!< Log file for all serial data (base)
    QFile logFile_Base_NMEA;        //!< Log file for NMEA serial data (base)
    QFile logFile_Base_UBX;         //!< Log file for UBX serial data (base)
    QFile logFile_Base_RTCM;        //!< Log file for RTCM serial data (base)

    QFile logFile_Tags;             //!< Log file for tags

    QFile logFile_Distances;        //!< Log file for distance measurements
    QFile logFile_Distances_Unfiltered;        //!< Log file for distance measurements (unfiltered)
    QFile logFile_Sync;             //!< Log file for syncing data (RELPOSNED-messages)

    bool treeItemsCreated = false;  //!< Textual items (name/value pairs) created?
    bool loggingActive = 0;         //!< All log files open and logging active?

    UBXMessage_RELPOSNED::ITOW lastMatchingRELPOSNEDiTOW = -1;  //!< Last matching iTOW for both rovers (-1 if no matching iTOW found)
    QElapsedTimer lastMatchingRELPOSNEDiTOWTimer;               //!< Started every time a matching iTOW is found
    UBXMessage_RELPOSNED::ITOW  lastTaggedRELPOSNEDiTOW = -1;   //!< To prevent multiple tags on same iTOW.

    QSoundEffect soundEffect_LMB;       //!< Sound effect for mouse button tagging (Left Mouse Button)
    QSoundEffect soundEffect_RMB;       //!< Sound effect for mouse button tagging (Right Mouse Button)
    QSoundEffect soundEffect_MMB;       //!< Sound effect for mouse button tagging (Middle Mouse Button/"Undo")
    QSoundEffect soundEffect_MBError;   //!< Sound effect for mouse button tagging (Error)
    QSoundEffect soundEffect_Distance;  //!< Sound effect for new distance

    const int maxPositionHistoryLength = 6000;                  //!< Maximum number of position history items to keep (used to calculate fluctuations)

    QList<NEDPoint> positionHistory_StylusTip;                  //!< Used to calculate fluctuation of stylus tip's position
    NEDPoint lastStylusTipPosition;

    double distanceBetweenFarthestCoordinates_StylusTip = nan("");  //!< Distance calculated between min/max coordinate values for all NED-axes during spinBox_FluctuationHistoryLength (stylus tip)

    double distanceBetweenRovers = 0;   //!< Euclidean distance between rovers (m)

    NEDPoint stylusTipPosition_LMB;  //!< Stylus tip position when the Left Mouse Button was pressed last time
    NEDPoint stylusTipPosition_RMB;  //!< Stylus tip position when the Right Mouse Button was pressed last time
    NEDPoint stylusTipPosition_MMB;  //!< Stylus tip position when the Middle Mouse Button was pressed last time

    QTreeWidgetItem *treeItem_DistanceBetweenFarthestCoordinates_StylusTip;
    QTreeWidgetItem *treeItem_DistanceBetweenRovers;
    QTreeWidgetItem *treeItem_LastTag;

    QTreeWidgetItem *treeItem_StylusTipNED;

    QTreeWidgetItem *treeItem_StylusTipAccNED;

    QTreeWidgetItem *treeItem_RoverAITOW;
    QTreeWidgetItem *treeItem_RoverBITOW;

    QTreeWidgetItem *treeItem_RoverASolution;
    QTreeWidgetItem *treeItem_RoverBSolution;

    QTreeWidgetItem *treeItem_RoverADiffSoln;
    QTreeWidgetItem *treeItem_RoverBDiffSoln;

    QTreeWidgetItem *treeItem_LMBNED;
    QTreeWidgetItem *treeItem_RMBNED;

    QTreeWidgetItem *treeItem_Distance_TipToLMB;
    QTreeWidgetItem *treeItem_Distance_TipToRMB;
    QTreeWidgetItem *treeItem_Distance_LMBToRMB;
    QTreeWidgetItem *treeItem_Distance_RoverAToTip;

    void handleRELPOSNEDQueues(void);   //!< Handles also syncing of rover RELPOSNED-messages
    void closeAllLogFiles(void);
    double calcDistanceBetweenFarthestCoordinates(const QList<NEDPoint>& positionHistory, int samples); //!< Calculates distance between min/max coordinate values for all NED-axes of last n samples
    double calcDistanceBetweenFarthestCoordinates(const QList<UBXMessage_RELPOSNED>& positionHistory, int samples); //!< Calculates distance between min/max coordinate values for all NED-axes of last n samples
    void updateTreeItems(void);
    void addMouseButtonTag(const QString& tagtext, QSoundEffect& soundEffect, qint64 uptime = -1);  //!< Adds mouse button tag to log file and plays soundEffect if successful
    void addTextTag(qint64 uptime = -1);
    void addDistanceLogItem(const DistanceItem& item);    //!< Adds distance to log file
    void addDistanceLogItem_Unfiltered(const DistanceItem& item);    //!< Adds distance to log file (unfiltered)
    void updateTipData(void);   //!< Updates tip-related fields

    QString getRoverIdentString(const unsigned int roverId);

    // Settings for video frame writing (ugly I know, but this is just to make video "recording" possible)
    bool video_WriteFrames = false;
    QString video_FileNameBeginning = "D:\\GNSSStylusData\\Videos\\EssentialFrames\\Video";
    int video_FrameCounter = 0;
    QPixmap* video_FrameBuffer = nullptr;
    void handleVideoFrameRecording(qint64 uptime);
    qint64 video_LastWrittenFrameUptime = 0;
    int video_ClipIndex = 0;
    double video_FPS = 30;
    qint64 video_ClipBaseTime = 0;
    double video_ClipDoubleTimer = 0;

    DistanceItem lastDistanceItemIncludingInvalid;          //!< Last distance received
    DistanceItem lastValidDistanceItem;                     //!< Last valid distance received
    QElapsedTimer lastValidDistanceItemTimer;               //!< Started when valid distance received
    QElapsedTimer lastDistanceItemTimerIncludingInvalid;    //!< Started when any distance received
};

#endif // ESSENTIALSFORM_H
