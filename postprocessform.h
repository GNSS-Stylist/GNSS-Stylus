/*
    postprocessform.h (part of GNSS-Stylus)
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

#include "gnssmessage.h"
#include "ubloxdatastreamprocessor.h"

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
        QString sourceFile;     //!< Filename of the source file
        int sourceFileLine;     //!< Line of the source file where this tag was read from
        QString ident;          //!< Identifier
        QString text;           //!< Additional textual info (for example a name for a new object)
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

    void on_pushButton_GeneratePointClouds_clicked();

    void on_pushButton_StartReplay_clicked();

    void on_pushButton_StopReplay_clicked();

    void on_pushButton_ContinueReplay_clicked();

    void on_pushButton_Movie_GenerateScript_clicked();

    // Slots for UBloxDataStreamProcessor
    void ubloxProcessor_nmeaSentenceReceived(const QByteArray& nmeaSentence);
    void ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage);
    void ubloxProcessor_rtcmMessageReceived(const RTCMMessage& rtcmMessage);
    void ubloxProcessor_ubxParseError(const QString& errorString);
    void ubloxProcessor_nmeaParseError(const QString& errorString);
    void ubloxProcessor_unidentifiedDataReceived(const QByteArray& data);

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

#if 0
    class MovieFrame
    {
    public:
        QVector3D roverAPos;
        QVector3D roverBPos;
        QVector3D stylusTipAPos;
        QVector3D cameraPos;
        QVector3D cameraLookingDirection;
    };
#endif

    Ui::PostProcessingForm *ui;

    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED> relposnedMessages_RoverA;    //!< RELPOSNED-data for rover A
    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED> relposnedMessages_RoverB;    //!< RELPOSNED-data for rover B
    QMap<UBXMessage_RELPOSNED::ITOW, Tag> tags;     //!< Tags read from a file

    bool fileDialogsInitialized = false;
    QFileDialog fileDialog_UBX;
    QFileDialog fileDialog_Tags;
    QFileDialog fileDialog_PointCloud;
    QFileDialog fileDialog_MovieScript;


    // Replay:
    UBXMessage_RELPOSNED::ITOW lastReplayedITOW = -1;
    QElapsedTimer replayTimeElapsedTimer;
    qint64 cumulativeRequestedWaitTime_ns;
    bool stopReplayRequest = false;

    void addLogLine(const QString& line);
    void addRELPOSNEDFileData(const QStringList& fileNames);
    void handleReplay(bool firstRound);
    UBXMessage_RELPOSNED::ITOW getNextRoverITOW(const UBXMessage_RELPOSNED::ITOW& iTOW);
    UBXMessage_RELPOSNED::ITOW getFirstRoverITOW();
    UBXMessage_RELPOSNED::ITOW getLastRoverITOW();

#if 0
    // Functions below not needed after all (used "cross-producted" unit vectors instead).
    // But left here for possible use in the future.

    // Some helpers to get camera location calculated (need to rotate around arbitrary axis...)
    // Thanks to https://www.programming-techniques.com/2012/03/3d-rotation-algorithm-about-arbitrary-axis-with-c-c-code.html
    void multiplyMatrix(const double inputMatrix[4][1], const double rotationMatrix[4][4], double (&outputMatrix)[4][1]);
    void setUpRotationMatrix(double angle, double u, double v, double w, double  (&rotationMatrix)[4][4]);
#endif

signals:
    void replayData_RoverA(const UBXMessage&);  //!< New data for rover A
    void replayData_RoverB(const UBXMessage&);  //!< New data for rover B

    // QT's signals and slots need to be defined exactly the same way, therefore PostProcessingForm::Tag
    void replayData_Tag(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag&); //!< New tag
};

#endif // POSTPROCESSFORM_H
