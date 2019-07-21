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
#include "ubloxdatastreamprocessor.h"
#include "gnssmessage.h"
#include "postprocessform.h"

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
    explicit EssentialsForm(QWidget *parent = nullptr); //!< Constructor
    ~EssentialsForm();

    void connectSerialThreadSlots_Base(SerialThread* serThread); //!< Connects signals from SerialThread (base)
    void disconnectSerialThreadSlots_Base(SerialThread* serThread); //!< Disconnects signals from SerialThread (base)
    void connectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor (base)
    void disconnectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor (base)

    void connectSerialThreadSlots_RoverA(SerialThread* serThread); //!< Connects signals from SerialThread (rover A)
    void disconnectSerialThreadSlots_RoverA(SerialThread* serThread); //!< Disconnects signals from SerialThread (rover A)
    void connectUBloxDataStreamProcessorSlots_RoverA(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor (rover A)
    void disconnectUBloxDataStreamProcessorSlots_RoverA(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor (rover A)

    void connectSerialThreadSlots_RoverB(SerialThread* serThread); //!< Connects signals from SerialThread (rover B)
    void disconnectSerialThreadSlots_RoverB(SerialThread* serThread); //!< Disconnects signals from SerialThread (rover B)
    void connectUBloxDataStreamProcessorSlots_RoverB(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor (rover B)
    void disconnectUBloxDataStreamProcessorSlots_RoverB(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor (rover B)

    void connectPostProcessingSlots(PostProcessingForm* postProcessingForm); //!< Connects signals from PostProcessingForm
    void disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm); //!< Disconnects signals from PostProcessingForm

private slots:
    void on_pushButton_StartLogging_clicked();
    void on_pushButton_StopLogging_clicked();

    void serialDataReceived_Base(const QByteArray& bytes);
    void nmeaSentenceReceived_Base(const QByteArray& nmeaSentence);
    void ubxMessageReceived_Base(const UBXMessage& ubxMessage);
    void rtcmMessageReceived_Base(const RTCMMessage& rtcmMessage);

    void serialDataReceived_RoverA(const QByteArray& bytes);
    void nmeaSentenceReceived_RoverA(const QByteArray& nmeaSentence);
    void ubxMessageReceived_RoverA(const UBXMessage& ubxMessage);

    void serialDataReceived_RoverB(const QByteArray& bytes);
    void nmeaSentenceReceived_RoverB(const QByteArray& nmeaSentence);
    void ubxMessageReceived_RoverB(const UBXMessage& ubxMessage);

    void postProcessingTagReceived(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag& tag);

    void on_pushButton_AddTag_clicked();

    void on_pushButton_MouseTag_clicked();
    void on_pushButton_MouseTag_rightClicked();
    void on_pushButton_MouseTag_middleClicked();

    void on_spinBox_FluctuationHistoryLength_valueChanged(int);

protected:
    void showEvent(QShowEvent* event);  //!< To initialize some things

private:
    /**
     * @brief Small helper class to store coordinates, accuracies and ITOW
     */
    class NEDPoint
    {
        public:

        UBXMessage_RELPOSNED::ITOW iTOW = -1;   //!< GNSS Time Of Week

        double n = 0;       //!< North (m)
        double e = 0;       //!< East (m)
        double d = 0;       //!< Down (m)

        double accN = -1;   //!< Accuracy of North-component (m)
        double accE = -1;   //!< Accuracy of North-component (m)
        double accD = -1;   //!< Accuracy of North-component (m)

        NEDPoint(const UBXMessage_RELPOSNED relposnedMessage);
        NEDPoint() = default;
    };

    Ui::EssentialsForm *ui;

    QFile logFile_Base_Raw;         //!< Log file for all serial data (base)
    QFile logFile_Base_NMEA;        //!< Log file for NMEA serial data (base)
    QFile logFile_Base_UBX;         //!< Log file for UBX serial data (base)
    QFile logFile_Base_RTCM;        //!< Log file for RTCM serial data (base)

    QFile logFile_RoverA_Raw;       //!< Log file for all serial data (rover A)
    QFile logFile_RoverA_NMEA;      //!< Log file for NMEA serial data (rover A)
    QFile logFile_RoverA_UBX;       //!< Log file for UBX serial data (rover A)
    QFile logFile_RoverA_RELPOSNED; //!< Log file for UBX-RELPOSNED serial data (rover A)

    QFile logFile_RoverB_Raw;       //!< Log file for all serial data (rover B)
    QFile logFile_RoverB_NMEA;      //!< Log file for NMEA serial data (rover B)
    QFile logFile_RoverB_UBX;       //!< Log file for UBX serial data (rover B)
    QFile logFile_RoverB_RELPOSNED; //!< Log file for UBX-RELPOSNED serial data (rover B)

    QFile logFile_Tags;             //!< Log file for tags

    bool treeItemsCreated = false;  //!< Textual items (name/value pairs) created?
    bool loggingActive = 0;         //!< All log files open and logging active?

    UBXMessage_RELPOSNED::ITOW lastMatchingRELPOSNEDiTOW = -1;  //!< Last matching iTOW for both rovers (-1 if no matching iTOW found)
    UBXMessage_RELPOSNED::ITOW  lastTaggedRELPOSNEDiTOW = -1;   //!< To prevent multiple tags on same iTOW.

    QSoundEffect soundEffect_LMB;       //!< Sound effect for mouse button tagging (Left Mouse Button)
    QSoundEffect soundEffect_RMB;       //!< Sound effect for mouse button tagging (Right Mouse Button)
    QSoundEffect soundEffect_MMB;       //!< Sound effect for mouse button tagging (Middle Mouse Button/"Undo")
    QSoundEffect soundEffect_Error;     //!< Sound effect for mouse button tagging (Error)

    QQueue<UBXMessage_RELPOSNED> messageQueue_RELPOSNED_RoverA; //!< Message queue for rover A RELPOSNED-messages (used to sync rovers)
    QQueue<UBXMessage_RELPOSNED> messageQueue_RELPOSNED_RoverB; //!< Message queue for rover B RELPOSNED-messages (used to sync rovers)

    const int maxPositionHistoryLength = 6000;                  //!< Maximum number of position history items to keep (used to calculate fluctuations)

    QList<UBXMessage_RELPOSNED> positionHistory_RoverA;         //!< Used to calculate fluctuation of rover A's position
    QList<UBXMessage_RELPOSNED> positionHistory_RoverB;         //!< Used to calculate fluctuation of rover A's position
    QList<NEDPoint> positionHistory_StylusTip;                  //!< Used to calculate fluctuation of stylus tip's position

    double distanceBetweenFarthestCoordinates_RoverA = nan("");     //!< Distance calculated between min/max coordinate values for all NED-axes during spinBox_FluctuationHistoryLength (rover A)
    double distanceBetweenFarthestCoordinates_RoverB = nan("");     //!< Distance calculated between min/max coordinate values for all NED-axes during spinBox_FluctuationHistoryLength (rover B)
    double distanceBetweenFarthestCoordinates_StylusTip = nan("");  //!< Distance calculated between min/max coordinate values for all NED-axes during spinBox_FluctuationHistoryLength (stylus tip)

    double distanceBetweenRovers = 0;   //!< Euclidean distance between rovers (m)

    QTreeWidgetItem *treeItem_DistanceBetweenFarthestCoordinates_RoverA;
    QTreeWidgetItem *treeItem_DistanceBetweenFarthestCoordinates_RoverB;
    QTreeWidgetItem *treeItem_DistanceBetweenFarthestCoordinates_StylusTip;
    QTreeWidgetItem *treeItem_DistanceBetweenRovers;
    QTreeWidgetItem *treeItem_LastTag;

    QTreeWidgetItem *treeItem_StylusTipNED;
    QTreeWidgetItem *treeItem_StylusTipXYZ;

    QTreeWidgetItem *treeItem_StylusTipAccNED;
    QTreeWidgetItem *treeItem_StylusTipAccXYZ;

    QTreeWidgetItem *treeItem_RoverAITOW;
    QTreeWidgetItem *treeItem_RoverBITOW;

    QTreeWidgetItem *treeItem_RoverASolution;
    QTreeWidgetItem *treeItem_RoverBSolution;

    QTreeWidgetItem *treeItem_RoverADiffSoln;
    QTreeWidgetItem *treeItem_RoverBDiffSoln;


    void handleRELPOSNEDQueues(void);   //!< Handles also syncing of rover RELPOSNED-messages
    void closeAllLogFiles(void);
    double calcDistanceBetweenFarthestCoordinates(const QList<NEDPoint>& positionHistory, int samples); //!< Calculates distance between min/max coordinate values for all NED-axes of last n samples
    double calcDistanceBetweenFarthestCoordinates(const QList<UBXMessage_RELPOSNED>& positionHistory, int samples); //!< Calculates distance between min/max coordinate values for all NED-axes of last n samples
    void updateTreeItems(void);
    void addMouseButtonTag(const QString& tagtext, QSoundEffect& soundEffect);  //!< Adds mouse button tag to log file and plays soundEffect if successful

    // Settings for video frame writing (ugly I know, but this is just to make video "recording" possible)
    QString video_FileNameBeginning = "";   // Leave empty for no video frame writing
//    QString video_FileNameBeginning = "D:\\GNSSStylusData\\Videos\\EssentialFrames\\Frame_";   // Leave empty for no video frame writing
    int video_FrameCounter = 0;
    UBXMessage_RELPOSNED::ITOW video_LastWrittenFrameITOW = -1;
    QPixmap* video_FrameBuffer = nullptr;
    void handleVideoFrameRecording(void);
};

#endif // ESSENTIALSFORM_H
