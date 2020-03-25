/*
    essentialsform.cpp (part of GNSS-Stylus)
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
 * @brief Definition for a form that has some "essential" things for realtime operation.
 */

#include <QMessageBox>
#include <QDir>
#include <QSettings>
#include <QTime>
#include <QString>
#include <QPalette>
#include <math.h>

#include "essentialsform.h"
#include "ui_essentialsform.h"
#include "Eigen/Geometry"

EssentialsForm::EssentialsForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EssentialsForm)
{
//    loggingActive = false;
//    lastMatchingRELPOSNEDiTOW = -1;
    ui->setupUi(this);

    QSettings settings;

    ui->lineEdit_LoggingDirectory->setText(settings.value("LoggingDirectory", "").toString());
    ui->lineEdit_LoggingFileNamePrefix->setText(settings.value("LoggingFileNamePrefix", "ublox").toString());
    ui->spinBox_FluctuationHistoryLength->setValue(settings.value("FluctuationHistoryLength").toInt());

    ui->comboBox_TagIdent->addItem("Suspend");
    ui->comboBox_TagIdent->addItem("Resume");
    ui->comboBox_TagIdent->addItem("New object");
    ui->comboBox_TagIdent->addItem("Manual tag");

//    QString soundDir = "D:\\GNSSStylusData\\AudioSamples\\";
    QString soundDir = "AudioSamples/";

    soundEffect_LMB.setSource(QUrl::fromLocalFile(soundDir + "LeftMouseButton.wav"));
    soundEffect_RMB.setSource(QUrl::fromLocalFile(soundDir + "RightMouseButton.wav"));
    soundEffect_MMB.setSource(QUrl::fromLocalFile(soundDir + "MiddleMouseButton.wav"));
    soundEffect_MBError.setSource(QUrl::fromLocalFile(soundDir + "ErrorBeep.wav"));
    soundEffect_Distance.setSource(QUrl::fromLocalFile(soundDir + "DistanceClick.wav"));

    soundEffect_LMB.setMuted(true);
    soundEffect_RMB.setMuted(true);
    soundEffect_MMB.setMuted(true);
    soundEffect_MBError.setMuted(true);
    soundEffect_Distance.setMuted(true);
}

EssentialsForm::~EssentialsForm()
{
    QSettings settings;
    settings.setValue("LoggingDirectory", ui->lineEdit_LoggingDirectory->text());
    settings.setValue("LoggingFileNamePrefix", ui->lineEdit_LoggingFileNamePrefix->text());
    settings.setValue("FluctuationHistoryLength", ui->spinBox_FluctuationHistoryLength->value());

    delete ui;
}

void EssentialsForm::closeAllLogFiles(void)
{
    logFile_Base_Raw.close();
    logFile_Base_NMEA.close();
    logFile_Base_UBX.close();
    logFile_Base_RTCM.close();
    logFile_RoverA_Raw.close();
    logFile_RoverA_NMEA.close();
    logFile_RoverA_UBX.close();
    logFile_RoverA_RELPOSNED.close();
    logFile_RoverB_Raw.close();
    logFile_RoverB_NMEA.close();
    logFile_RoverB_UBX.close();
    logFile_RoverB_RELPOSNED.close();
    logFile_Tags.close();
    logFile_Distances.close();
    logFile_Distances_Unfiltered.close();
    logFile_Sync.close();
}

void EssentialsForm::on_pushButton_StartLogging_clicked()
{
    closeAllLogFiles();

    if ((ui->lineEdit_LoggingDirectory->text().length() == 0) ||
            (ui->lineEdit_LoggingFileNamePrefix->text().length() == 0))
    {
        QMessageBox msgBox;
        msgBox.setText("Directory or file name prefix can't be empty.");
        msgBox.exec();
        return;
    }

    QString fileNameBeginning = QDir::cleanPath(ui->lineEdit_LoggingDirectory->text() + "\\" + ui->lineEdit_LoggingFileNamePrefix->text());

    logFile_Base_Raw.setFileName(fileNameBeginning + "_base.raw");
    logFile_Base_NMEA.setFileName(fileNameBeginning + "_base.NMEA");
    logFile_Base_UBX.setFileName(fileNameBeginning + "_base.ubx");
    logFile_Base_RTCM.setFileName(fileNameBeginning + "_base.RTCM");

    logFile_RoverA_Raw.setFileName(fileNameBeginning + "_RoverA.raw");
    logFile_RoverA_NMEA.setFileName(fileNameBeginning + "_RoverA.NMEA");
    logFile_RoverA_UBX.setFileName(fileNameBeginning + "_RoverA.ubx");
    logFile_RoverA_RELPOSNED.setFileName(fileNameBeginning + "_RoverA_RELPOSNED.ubx");

    logFile_RoverB_Raw.setFileName(fileNameBeginning + "_RoverB.raw");
    logFile_RoverB_NMEA.setFileName(fileNameBeginning + "_RoverB.NMEA");
    logFile_RoverB_UBX.setFileName(fileNameBeginning + "_RoverB.ubx");
    logFile_RoverB_RELPOSNED.setFileName(fileNameBeginning + "_RoverB_RELPOSNED.ubx");

    logFile_Tags.setFileName(fileNameBeginning + "_tags.tags");

    logFile_Distances.setFileName(fileNameBeginning + ".distances");
    logFile_Distances_Unfiltered.setFileName(fileNameBeginning + "_Unfiltered.distances");
    logFile_Sync.setFileName(fileNameBeginning + ".sync");

    QIODevice::OpenMode openMode = QIODevice::WriteOnly;

    if (logFile_Base_Raw.exists() ||
            logFile_Base_NMEA.exists() ||
            logFile_Base_UBX.exists() ||
            logFile_Base_RTCM.exists() ||
            logFile_RoverA_Raw.exists() ||
            logFile_RoverA_NMEA.exists() ||
            logFile_RoverA_UBX.exists() ||
            logFile_RoverA_RELPOSNED.exists() ||
            logFile_RoverB_Raw.exists() ||
            logFile_RoverB_NMEA.exists() ||
            logFile_RoverB_UBX.exists() ||
            logFile_RoverB_RELPOSNED.exists() ||
            logFile_Tags.exists() ||
            logFile_Distances.exists() ||
            logFile_Distances_Unfiltered.exists() ||
            logFile_Sync.exists())
    {
        QMessageBox msgBox;
        msgBox.setText("One or more of the files already exists.");
        msgBox.setInformativeText("How to proceed?");

        QPushButton *appendButton = msgBox.addButton(tr("Append"), QMessageBox::ActionRole);
        QPushButton *overwriteButton = msgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
        QPushButton *cancelButton = msgBox.addButton(QMessageBox::Cancel);

        msgBox.setDefaultButton(cancelButton);

        msgBox.exec();

        if (msgBox.clickedButton() == appendButton)
        {
            openMode = QIODevice::Append;
        }
        else if (msgBox.clickedButton() == overwriteButton)
        {
            openMode = QIODevice::WriteOnly;
        }
        else
        {
            return;
        }
    }

    bool addTagFileHeader = true;
    bool addDistanceFileHeader = true;
    bool addDistanceFileHeader_Unfiltered = true;
    bool addSyncFileHeader = true;

    if (logFile_Tags.exists() && (openMode == QIODevice::Append))
    {
        addTagFileHeader = false;
    }

    if (logFile_Distances.exists() && (openMode == QIODevice::Append))
    {
        addDistanceFileHeader = false;
    }

    if (logFile_Distances_Unfiltered.exists() && (openMode == QIODevice::Append))
    {
        addDistanceFileHeader_Unfiltered = false;
    }

    if (logFile_Sync.exists() && (openMode == QIODevice::Append))
    {
        addSyncFileHeader = false;
    }

    logFile_Base_Raw.open(openMode);
    logFile_Base_NMEA.open(openMode);
    logFile_Base_UBX.open(openMode);
    logFile_Base_RTCM.open(openMode);
    logFile_RoverA_Raw.open(openMode);
    logFile_RoverA_NMEA.open(openMode);
    logFile_RoverA_UBX.open(openMode);
    logFile_RoverA_RELPOSNED.open(openMode);
    logFile_RoverB_Raw.open(openMode);
    logFile_RoverB_NMEA.open(openMode);
    logFile_RoverB_UBX.open(openMode);
    logFile_RoverB_RELPOSNED.open(openMode);
    logFile_Tags.open(openMode| QIODevice::Text);
    logFile_Distances.open(openMode| QIODevice::Text);
    logFile_Distances_Unfiltered.open(openMode| QIODevice::Text);
    logFile_Sync.open(openMode| QIODevice::Text);

    if (!logFile_Base_Raw.isOpen() ||
            !logFile_Base_NMEA.isOpen() ||
            !logFile_Base_UBX.isOpen() ||
            !logFile_Base_RTCM.isOpen() ||
            !logFile_RoverA_Raw.isOpen() ||
            !logFile_RoverA_NMEA.isOpen() ||
            !logFile_RoverA_UBX.isOpen() ||
            !logFile_RoverA_RELPOSNED.isOpen() ||
            !logFile_RoverB_Raw.isOpen() ||
            !logFile_RoverB_NMEA.isOpen() ||
            !logFile_RoverB_UBX.isOpen() ||
            !logFile_RoverB_RELPOSNED.isOpen() ||
            !logFile_Tags.isOpen() ||
            !logFile_Distances.isOpen() ||
            !logFile_Distances_Unfiltered.isOpen() ||
            !logFile_Sync.isOpen())
    {
        QMessageBox msgBox;
        msgBox.setText("One or more of the files can't be opened.");

        msgBox.exec();

        closeAllLogFiles();
        return;
    }

    if (addTagFileHeader)
    {
        QTextStream textStream(&logFile_Tags);

        textStream << "Time\tiTOW\tTag\tText\tUptime\n";
    }

    if (addDistanceFileHeader)
    {
        QTextStream textStream(&logFile_Distances);

        textStream << "Time\tDistance\tType\tUptime(Start)\tFrame time\n";
    }

    if (addDistanceFileHeader_Unfiltered)
    {
        QTextStream textStream(&logFile_Distances_Unfiltered);

        textStream << "Time\tDistance\tType\tUptime(Start)\tFrame time\n";
    }

    if (lastValidDistanceItem.type != DistanceItem::Type::UNKNOWN)
    {
        // Log "fallback" item to the start of the file

        DistanceItem dItem = lastValidDistanceItem;

        dItem.frameStartTime = 0;
        dItem.frameEndTime = 0;

        addDistanceLogItem(dItem);
    }

    if (addSyncFileHeader)
    {
        QTextStream textStream(&logFile_Sync);

        textStream << "Time\tSource\tType\tiTOW\tUptime(Start)\tFrame time\n";
    }

    loggingActive = true;

    ui->pushButton_StartLogging->setEnabled(false);
    ui->pushButton_StopLogging->setEnabled(true);
    ui->lineEdit_LoggingDirectory->setEnabled(false);
    ui->lineEdit_LoggingFileNamePrefix->setEnabled(false);
    ui->pushButton_AddTag->setEnabled(true);
}

void EssentialsForm::on_pushButton_StopLogging_clicked()
{
    ui->pushButton_StartLogging->setEnabled(true);
    ui->pushButton_StopLogging->setEnabled(false);
    ui->lineEdit_LoggingDirectory->setEnabled(true);
    ui->lineEdit_LoggingFileNamePrefix->setEnabled(true);
    ui->pushButton_AddTag->setEnabled(false);


    loggingActive = false;

    closeAllLogFiles();
}

void EssentialsForm::dataReceived_Base(const QByteArray& bytes)
{
    if (loggingActive)
    {
        logFile_Base_Raw.write(bytes);
    }
}

void EssentialsForm::nmeaSentenceReceived_Base(const NMEAMessage& nmeaMessage)
{
    if (loggingActive)
    {
        logFile_Base_NMEA.write(nmeaMessage.rawMessage);
    }
}

void EssentialsForm::ubxMessageReceived_Base(const UBXMessage& ubxMessage)
{
    if (loggingActive)
    {
        logFile_Base_UBX.write(ubxMessage.rawMessage);
    }
}

void EssentialsForm::rtcmMessageReceived_Base(const RTCMMessage& rtcmMessage)
{
    if (loggingActive)
    {
        logFile_Base_RTCM.write(rtcmMessage.rawMessage);
    }
}

void EssentialsForm::serialDataReceived_RoverA(const QByteArray& bytes)
{
    if (loggingActive)
    {
        logFile_RoverA_Raw.write(bytes);
    }
}

void EssentialsForm::nmeaSentenceReceived_RoverA(const NMEAMessage& nmeaMessage)
{
    if (loggingActive)
    {
        logFile_RoverA_NMEA.write(nmeaMessage.rawMessage);
    }
}

void EssentialsForm::ubxMessageReceived_RoverA(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        // "Casting" generic UBX-message to RELPOSNED was successful
        positionHistory_RoverA.append(relposned);

        while (positionHistory_RoverA.size() > maxPositionHistoryLength)
        {
            positionHistory_RoverA.removeFirst();
        }

        distanceBetweenFarthestCoordinates_RoverA = calcDistanceBetweenFarthestCoordinates(positionHistory_RoverA, ui->spinBox_FluctuationHistoryLength->value());

        messageQueue_RELPOSNED_RoverA.enqueue(relposned);

        handleVideoFrameRecording(ubxMessage.messageEndTime - 1);

        handleRELPOSNEDQueues();

        updateTreeItems();

//        handleVideoFrameRecording(ubxMessage.messageEndTime);
    }

    if (loggingActive)
    {
        logFile_RoverA_UBX.write(ubxMessage.rawMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            logFile_RoverA_RELPOSNED.write(ubxMessage.rawMessage);

            QTextStream textStream(&logFile_Sync);

            textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" <<
                          "Rover A\tRELPOSNED\t" << QString::number(relposned.iTOW) <<
                          "\t" << QString::number(ubxMessage.messageStartTime) << "\t" <<
                          QString::number(ubxMessage.messageEndTime - ubxMessage.messageStartTime) << "\n";
        }
    }
}

void EssentialsForm::serialDataReceived_RoverB(const QByteArray& bytes)
{
    if (loggingActive)
    {
        logFile_RoverB_Raw.write(bytes);
    }
}

void EssentialsForm::nmeaSentenceReceived_RoverB(const NMEAMessage& nmeaMessage)
{
    if (loggingActive)
    {
        logFile_RoverB_NMEA.write(nmeaMessage.rawMessage);
    }
}

void EssentialsForm::ubxMessageReceived_RoverB(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        // "Casting" generic UBX-message to RELPOSNED was successful

        positionHistory_RoverB.append(relposned);

        while (positionHistory_RoverB.size() > maxPositionHistoryLength)
        {
            positionHistory_RoverB.removeFirst();
        }

        distanceBetweenFarthestCoordinates_RoverB = calcDistanceBetweenFarthestCoordinates(positionHistory_RoverB, ui->spinBox_FluctuationHistoryLength->value());

        messageQueue_RELPOSNED_RoverB.enqueue(relposned);

        handleVideoFrameRecording(ubxMessage.messageEndTime - 1);

        handleRELPOSNEDQueues();

        updateTreeItems();

//        handleVideoFrameRecording(ubxMessage.messageEndTime);
    }

    if (loggingActive)
    {
        logFile_RoverB_UBX.write(ubxMessage.rawMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            logFile_RoverB_RELPOSNED.write(ubxMessage.rawMessage);

            QTextStream textStream(&logFile_Sync);

            textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" <<
                          "Rover B\tRELPOSNED\t" << QString::number(relposned.iTOW) <<
                          "\t" << QString::number(ubxMessage.messageStartTime) << "\t" <<
                          QString::number(ubxMessage.messageEndTime - ubxMessage.messageStartTime) << "\n";
        }
    }
}

void EssentialsForm::postProcessingTagReceived(const qint64 uptime, const PostProcessingForm::Tag& tag)
{
    handleVideoFrameRecording(uptime - 1);

    if (tag.ident == "LMB")
    {
        addMouseButtonTag("LMB", soundEffect_LMB, uptime);
        stylusTipPosition_LMB = lastStylusTipPosition;
    }
    else if (tag.ident == "MMB")
    {
        addMouseButtonTag("MMB", soundEffect_MMB, uptime);
        stylusTipPosition_MMB = lastStylusTipPosition;
    }
    else if (tag.ident == "RMB")
    {
        addMouseButtonTag("RMB", soundEffect_RMB, uptime);
        stylusTipPosition_RMB = lastStylusTipPosition;
    }
    else
    {
        ui->comboBox_TagIdent->setEditText(tag.ident);
        ui->lineEdit_TagText->setText(tag.text);
        addTextTag(uptime);
    }
//    handleVideoFrameRecording(uptime);
}

void EssentialsForm::postProcessingDistanceReceived(const qint64 uptime, const PostProcessingForm::DistanceItem& ppDistanceItem)
{
    handleVideoFrameRecording(uptime - 1);

    DistanceItem localdistanceItem;

    localdistanceItem.distance = ppDistanceItem.distance;

    switch (ppDistanceItem.type)
    {
    case PostProcessingForm::DistanceItem::CONSTANT:
        localdistanceItem.type = DistanceItem::CONSTANT;
        break;
    case PostProcessingForm::DistanceItem::MEASURED:
        localdistanceItem.type = DistanceItem::MEASURED;
        break;
    default:
        localdistanceItem.type = DistanceItem::UNKNOWN;
        break;
    }

    localdistanceItem.frameStartTime = uptime;
    localdistanceItem.frameEndTime = uptime + ppDistanceItem.frameDuration;

    on_distanceReceived(localdistanceItem);

//    handleVideoFrameRecording(uptime);
}

void EssentialsForm::connectSerialThreadSlots_Base(SerialThread* serThread)
{
    QObject::connect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(dataReceived_Base(const QByteArray&)));
}

void EssentialsForm::disconnectSerialThreadSlots_Base(SerialThread* serThread)
{
    QObject::disconnect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(dataReceived_Base(const QByteArray&)));
}

void EssentialsForm::connectNTRIPThreadSlots_Base(NTRIPThread* ntripThread)
{
    QObject::connect(ntripThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(dataReceived_Base(const QByteArray&)));
}

void EssentialsForm::disconnectNTRIPThreadSlots_Base(NTRIPThread* ntripThread)
{
    QObject::disconnect(ntripThread, SIGNAL(dataReceived(const QByteArray&)),
                     this, SLOT(dataReceived_Base(const QByteArray&)));
}

void EssentialsForm::connectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_Base(const NMEAMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_Base(const UBXMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(rtcmMessageReceived_Base(const RTCMMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_Base(const NMEAMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_Base(const UBXMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(rtcmMessageReceived_Base(const RTCMMessage&)));
}

void EssentialsForm::connectSerialThreadSlots_RoverA(SerialThread* serThread)
{
    QObject::connect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(serialDataReceived_RoverA(const QByteArray&)));
}

void EssentialsForm::disconnectSerialThreadSlots_RoverA(SerialThread* serThread)
{
    QObject::disconnect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(serialDataReceived_RoverA(const QByteArray&)));
}

void EssentialsForm::connectUBloxDataStreamProcessorSlots_RoverA(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_RoverA(const NMEAMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_RoverA(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_RoverA(const NMEAMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));
}

void EssentialsForm::connectSerialThreadSlots_RoverB(SerialThread* serThread)
{
    QObject::connect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(serialDataReceived_RoverB(const QByteArray&)));
}

void EssentialsForm::disconnectSerialThreadSlots_RoverB(SerialThread* serThread)
{
    QObject::disconnect(serThread, SIGNAL(dataReceived(const QByteArray&, qint64, qint64, const SerialThread::DataReceivedEmitReason&)),
                     this, SLOT(serialDataReceived_RoverB(const QByteArray&)));
}

void EssentialsForm::connectUBloxDataStreamProcessorSlots_RoverB(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_RoverB(const NMEAMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_RoverB(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                     this, SLOT(nmeaSentenceReceived_RoverB(const NMEAMessage&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));
}

void EssentialsForm::connectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverA(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverB(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Tag(const qint64, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const qint64, const PostProcessingForm::Tag&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Distance(const qint64, const PostProcessingForm::DistanceItem&)),
                     this, SLOT(postProcessingDistanceReceived(const qint64, const PostProcessingForm::DistanceItem&)));
}

void EssentialsForm::disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::disconnect(postProcessingForm, SIGNAL(replayData_RoverA(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_RoverB(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Tag(const qint64, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const qint64, const PostProcessingForm::Tag&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Distance(const qint64, const PostProcessingForm::DistanceItem&)),
                     this, SLOT(postProcessingDistanceReceived(const qint64, const PostProcessingForm::DistanceItem&)));
}

void EssentialsForm::connectLaserRangeFinder20HzV2SerialThreadSlots(LaserRangeFinder20HzV2SerialThread* distanceThread)
{
    QObject::connect(distanceThread, SIGNAL(distanceReceived(const double&, qint64, qint64)),
                     this, SLOT(on_measuredDistanceReceived(const double&, qint64, qint64)));
}

void EssentialsForm::disconnectLaserRangeFinder20HzV2SerialThreadSlots(LaserRangeFinder20HzV2SerialThread* distanceThread)
{
    QObject::disconnect(distanceThread, SIGNAL(distanceReceived(const double&, qint64, qint64)),
                     this, SLOT(on_measuredDistanceReceived(const double&, qint64, qint64)));
}

void EssentialsForm::addTextTag(qint64 uptime)
{
    handleVideoFrameRecording(uptime - 1);
    if (loggingActive)
    {
        QTextStream textStream(&logFile_Tags);

        if (uptime < 0)
        {
            QElapsedTimer uptimeTimer;
            uptimeTimer.start();
            uptime = uptimeTimer.msecsSinceReference();
        }

        textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" << QString::number(lastMatchingRELPOSNEDiTOW) << "\t"
                   << ui->comboBox_TagIdent->lineEdit()->text() << "\t" << ui->lineEdit_TagText->text() << "\t"
                   << QString::number(uptime) << "\n";

        treeItem_LastTag->setText(1, ui->comboBox_TagIdent->lineEdit()->text() +  "; " + ui->lineEdit_TagText->text());

        lastTaggedRELPOSNEDiTOW = lastMatchingRELPOSNEDiTOW;
    }
    else
    {
        treeItem_LastTag->setText(1, "Logging not active!");
    }
//    handleVideoFrameRecording(uptime);
}

void EssentialsForm::on_pushButton_AddTag_clicked()
{
    addTextTag();
}

void EssentialsForm::on_pushButton_MouseTag_clicked()
{
    addMouseButtonTag("LMB", soundEffect_LMB);
    stylusTipPosition_LMB = lastStylusTipPosition;
}

void EssentialsForm::on_pushButton_MouseTag_rightClicked()
{
    addMouseButtonTag("RMB", soundEffect_RMB);
    stylusTipPosition_RMB = lastStylusTipPosition;
}

void EssentialsForm::on_pushButton_MouseTag_middleClicked()
{
    addMouseButtonTag("MMB", soundEffect_MMB);
    stylusTipPosition_MMB = lastStylusTipPosition;
}

void EssentialsForm::addMouseButtonTag(const QString& tagtext, QSoundEffect& soundEffect, qint64 uptime)
{
    handleVideoFrameRecording(uptime - 1);
    if (loggingActive)
    {
        treeItem_LastTag->setText(1, tagtext);

        QTextStream textStream(&logFile_Tags);

        if (uptime < 0)
        {
            QElapsedTimer uptimeTimer;
            uptimeTimer.start();
            uptime = uptimeTimer.msecsSinceReference();
        }

        textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" << QString::number(lastMatchingRELPOSNEDiTOW) << "\t"
                   << tagtext << "\t" << "" << "\t"
                   << QString::number(uptime) << "\n";

        soundEffect.play();

        ui->pushButton_MouseTag->setText("Tagged " + tagtext);
        lastTaggedRELPOSNEDiTOW = lastMatchingRELPOSNEDiTOW;
    }
    else
    {
        soundEffect_MBError.play();

        ui->pushButton_MouseTag->setText("Logging not active!");

        treeItem_LastTag->setText(1, "Logging not active!");
    }
//    handleVideoFrameRecording(uptime);
}


void EssentialsForm::handleRELPOSNEDQueues(void)
{
    bool matchingiTOWFound = false;

    while ((!messageQueue_RELPOSNED_RoverA.isEmpty()) &&
            (!messageQueue_RELPOSNED_RoverB.isEmpty()))
    {
        while ((!messageQueue_RELPOSNED_RoverA.isEmpty()) &&
               (!messageQueue_RELPOSNED_RoverB.isEmpty()) &&
               (messageQueue_RELPOSNED_RoverA.head().iTOW < messageQueue_RELPOSNED_RoverB.head().iTOW))
        {
            // Discard all rover A RELPOSNED-messages that are older (lower iTOW) than the first rover B message in queue
            messageQueue_RELPOSNED_RoverA.dequeue();
        }

        while ((!messageQueue_RELPOSNED_RoverB.isEmpty()) &&
               (!messageQueue_RELPOSNED_RoverA.isEmpty()) &&
               (messageQueue_RELPOSNED_RoverB.head().iTOW < messageQueue_RELPOSNED_RoverA.head().iTOW))
        {
            // Discard all rover B RELPOSNED-messages that are older (lower iTOW) than the first rover A message in queue
            messageQueue_RELPOSNED_RoverB.dequeue();
        }

        if ((!messageQueue_RELPOSNED_RoverA.isEmpty()) &&
                (!messageQueue_RELPOSNED_RoverB.isEmpty()))
        {
            UBXMessage_RELPOSNED roverARELPOSNED = messageQueue_RELPOSNED_RoverA.head();
            UBXMessage_RELPOSNED roverBRELPOSNED = messageQueue_RELPOSNED_RoverB.head();

            if (roverARELPOSNED.iTOW == roverBRELPOSNED.iTOW)
            {
                // Queues are in sync -> process
                messageQueue_RELPOSNED_RoverA.dequeue();
                messageQueue_RELPOSNED_RoverB.dequeue();

                lastMatchingRELPOSNEDiTOW = static_cast<int>(roverARELPOSNED.iTOW);
                matchingiTOWFound = true;
                lastMatchingRELPOSNEDiTOWTimer.start();

                lastMatchingRoverARELPOSNED = roverARELPOSNED;
                lastMatchingRoverBRELPOSNED = roverBRELPOSNED;

                updateTipData();
            }
        }
    }

    // Prevent groving queues too much if only one rover is sending RELPOSNEDS
    while (messageQueue_RELPOSNED_RoverA.size() >= 100)
    {
        messageQueue_RELPOSNED_RoverA.dequeue();
    }
    while (messageQueue_RELPOSNED_RoverB.size() >= 100)
    {
        messageQueue_RELPOSNED_RoverB.dequeue();
    }

    if (matchingiTOWFound)
    {
        ui->label_iTOW_BIG->setNum(lastMatchingRELPOSNEDiTOW);

        if ((lastMatchingRoverARELPOSNED.messageDataStatus == UBXMessage::STATUS_VALID) &&
                (lastMatchingRoverBRELPOSNED.messageDataStatus == UBXMessage::STATUS_VALID))
        {
            double accuracy_RoverA = sqrt(lastMatchingRoverARELPOSNED.accN * lastMatchingRoverARELPOSNED.accN +
                                          lastMatchingRoverARELPOSNED.accE * lastMatchingRoverARELPOSNED.accE +
                                          lastMatchingRoverARELPOSNED.accD * lastMatchingRoverARELPOSNED.accD);

            double accuracy_RoverB = sqrt(lastMatchingRoverBRELPOSNED.accN * lastMatchingRoverBRELPOSNED.accN +
                                          lastMatchingRoverBRELPOSNED.accE * lastMatchingRoverBRELPOSNED.accE +
                                          lastMatchingRoverBRELPOSNED.accD * lastMatchingRoverBRELPOSNED.accD);

            double worstAccuracy = accuracy_RoverA;

            if (accuracy_RoverB > worstAccuracy)
            {
                worstAccuracy = accuracy_RoverB;
            }

            int worstAccuracyInt = static_cast<int>(worstAccuracy * 1000);

            ui->label_WorstAccuracy->setText(QString::number(worstAccuracyInt) + " mm");

            if (worstAccuracyInt > ui->progressBar_Accuracy->maximum())
            {
                worstAccuracyInt = ui->progressBar_Accuracy->maximum();
            }

            ui->progressBar_Accuracy->setValue(static_cast<int>(worstAccuracyInt));

            double distN = lastMatchingRoverARELPOSNED.relPosN - lastMatchingRoverBRELPOSNED.relPosN;
            double distE = lastMatchingRoverARELPOSNED.relPosE - lastMatchingRoverBRELPOSNED.relPosE;
            double distD = lastMatchingRoverARELPOSNED.relPosD - lastMatchingRoverBRELPOSNED.relPosD;

            distanceBetweenRovers = sqrt(distN * distN + distE * distE + distD * distD);
        }
    }
}

EssentialsForm::NEDPoint::NEDPoint(const UBXMessage_RELPOSNED relposnedMessage)
{
    this->valid = true;
    this->iTOW = static_cast<int>(relposnedMessage.iTOW);
    this->uptime = relposnedMessage.messageStartTime;

    this->n = relposnedMessage.relPosN;
    this->e = relposnedMessage.relPosE;
    this->d = relposnedMessage.relPosD;

    this->accN = relposnedMessage.accN;
    this->accE = relposnedMessage.accE;
    this->accD = relposnedMessage.accD;
}

double EssentialsForm::NEDPoint::getDistanceTo(const NEDPoint &other)
{
    double nDiff = this->n - other.n;
    double eDiff = this->e - other.e;
    double dDiff = this->d - other.d;

    return (sqrt(nDiff * nDiff + eDiff * eDiff + dDiff * dDiff));
}


double EssentialsForm::calcDistanceBetweenFarthestCoordinates(const QList<NEDPoint>& positionHistory, int samples)
{
    if (positionHistory.length() >= 2)
    {
        double highN = -1e12;
        double highE = -1e12;
        double highD = -1e12;

        double lowN = 1e12;
        double lowE = 1e12;
        double lowD = 1e12;

        QList<NEDPoint>::const_iterator iter = positionHistory.end() - 1;

        do
        {
            if (iter->n < lowN)
            {
                lowN = iter->n;
            }
            if (iter->e < lowE)
            {
                lowE = iter->e;
            }
            if (iter->d < lowD)
            {
                lowD = iter->d;
            }

            if (iter->n > highN)
            {
                highN = iter->n;
            }
            if (iter->e > highE)
            {
                highE = iter->e;
            }
            if (iter->d > highD)
            {
                highD = iter->d;
            }
            iter--;
            samples--;
        } while ((iter != positionHistory.begin()) && (samples >= 0));

        double diffN = highN - lowN;
        double diffE = highE - lowE;
        double diffD = highD - lowD;

        double distanceHighLow = sqrt(diffN * diffN + diffE * diffE + diffD * diffD);

        return distanceHighLow;
    }
    else
    {
        return 0;
    }
}


double EssentialsForm::calcDistanceBetweenFarthestCoordinates(const QList<UBXMessage_RELPOSNED>& positionHistory, int samples)
{
    if (positionHistory.length() >= 2)
    {
        double highN = -1e12;
        double highE = -1e12;
        double highD = -1e12;

        double lowN = 1e12;
        double lowE = 1e12;
        double lowD = 1e12;

        QList<UBXMessage_RELPOSNED>::const_iterator iter = positionHistory.end() - 1;

        do
        {
            if (iter->relPosN < lowN)
            {
                lowN = iter->relPosN;
            }
            if (iter->relPosE < lowE)
            {
                lowE = iter->relPosE;
            }
            if (iter->relPosD < lowD)
            {
                lowD = iter->relPosD;
            }

            if (iter->relPosN > highN)
            {
                highN = iter->relPosN;
            }
            if (iter->relPosE > highE)
            {
                highE = iter->relPosE;
            }
            if (iter->relPosD > highD)
            {
                highD = iter->relPosD;
            }

            iter--;
            samples--;
        } while ((iter != positionHistory.begin()) && (samples >= 0));

        double diffN = highN - lowN;
        double diffE = highE - lowE;
        double diffD = highD - lowD;

        double distanceHighLow = sqrt(diffN * diffN + diffE * diffE + diffD * diffD);

        return distanceHighLow;
    }
    else
    {
        return 0;
    }
}

void EssentialsForm::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    updateTreeItems();
}

void EssentialsForm::updateTreeItems(void)
{
    if (!treeItemsCreated)
    {
        // Can't create these on constructor -> Do it when running this function first time
        treeItem_RoverAITOW = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverAITOW->setText(0, "Rover A ITOW");

        treeItem_RoverBITOW = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverBITOW->setText(0, "Rover B ITOW");

        treeItem_RoverASolution = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverASolution->setText(0, "Rover A solution status");

        treeItem_RoverBSolution = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverBSolution->setText(0, "Rover B solution status");

        treeItem_StylusTipAccNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipAccNED->setText(0, "Tip AccNED");

        treeItem_RoverADiffSoln = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverADiffSoln->setText(0, "Rover A differential corr");

        treeItem_RoverBDiffSoln = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RoverBDiffSoln->setText(0, "Rover B differential corr");

        treeItem_DistanceBetweenFarthestCoordinates_RoverA = new QTreeWidgetItem(ui->treeWidget);
        treeItem_DistanceBetweenFarthestCoordinates_RoverA->setText(0, "Pos fluctuation Rover A");

        treeItem_DistanceBetweenFarthestCoordinates_RoverB = new QTreeWidgetItem(ui->treeWidget);
        treeItem_DistanceBetweenFarthestCoordinates_RoverB->setText(0, "Pos fluctuation Rover B");

        treeItem_DistanceBetweenFarthestCoordinates_StylusTip = new QTreeWidgetItem(ui->treeWidget);
        treeItem_DistanceBetweenFarthestCoordinates_StylusTip->setText(0, "Pos fluctuation Stylus tip");

        treeItem_DistanceBetweenRovers = new QTreeWidgetItem(ui->treeWidget);
        treeItem_DistanceBetweenRovers->setText(0, "Distance between rovers");

        treeItem_LastTag = new QTreeWidgetItem(ui->treeWidget);
        treeItem_LastTag->setText(0, "Last tag");

        treeItem_StylusTipNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipNED->setText(0, "Tip NED");

        treeItem_LMBNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_LMBNED->setText(0, "LMB NED");

        treeItem_RMBNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_RMBNED->setText(0, "RMB NED");

        treeItem_Distance_TipToLMB = new QTreeWidgetItem(ui->treeWidget);
        treeItem_Distance_TipToLMB->setText(0, "Distance tip<->LMB");

        treeItem_Distance_TipToRMB = new QTreeWidgetItem(ui->treeWidget);
        treeItem_Distance_TipToRMB->setText(0, "Distance tip<->RMB");

        treeItem_Distance_LMBToRMB = new QTreeWidgetItem(ui->treeWidget);
        treeItem_Distance_LMBToRMB->setText(0, "Distance LMB<->RMB");

        treeItem_Distance_RoverAToTip = new QTreeWidgetItem(ui->treeWidget);
        treeItem_Distance_RoverAToTip->setText(0, "Distance RoverA<->Tip");

        treeItemsCreated = true;
    }

    treeItem_DistanceBetweenFarthestCoordinates_RoverA->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverA, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_RoverB->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverB, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_StylusTip->setText(1, QString::number(distanceBetweenFarthestCoordinates_StylusTip, 'f', 3));
    treeItem_DistanceBetweenRovers->setText(1, QString::number(distanceBetweenRovers, 'f', 3));

    treeItem_StylusTipNED->setText(1, QString::number(lastStylusTipPosition.n, 'f', 3) + ", " + QString::number(lastStylusTipPosition.e, 'f', 3) + ", " + QString::number(lastStylusTipPosition.d, 'f', 3));
    treeItem_StylusTipAccNED->setText(1, QString::number(lastStylusTipPosition.accN, 'f', 3) + ", " + QString::number(lastStylusTipPosition.accE, 'f', 3) + ", " + QString::number(lastStylusTipPosition.accD, 'f', 3));

    const QColor positionValidColor = QColor(128,255,128);
    const QColor positionInvalidColor = QColor(255,128,128);

    if (lastStylusTipPosition.valid)
    {
        treeItem_StylusTipNED->setBackgroundColor(1, positionValidColor);
        treeItem_StylusTipAccNED->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_StylusTipNED->setBackgroundColor(1, positionInvalidColor);
        treeItem_StylusTipAccNED->setBackgroundColor(1, positionInvalidColor);
    }

    const QColor solutionColors[4] =
    {
        QColor(255,128,128),
        QColor(255,255,0),
        QColor(128,255,128),
        QColor(255,128,128)
    };

    if (positionHistory_RoverA.isEmpty())
    {
        treeItem_RoverAITOW->setText(1, "N/A");
        treeItem_RoverASolution->setText(1, "N/A");
        treeItem_RoverADiffSoln->setText(1, "N/A");
        treeItem_RoverASolution->setBackgroundColor(1, solutionColors[UBXMessage_RELPOSNED::UNDEFINED]);
        treeItem_RoverASolution->setText(1, "N/A");
    }
    else
    {
        UBXMessage_RELPOSNED roverA = positionHistory_RoverA.last();

        treeItem_RoverAITOW->setText(1, QString::number(roverA.iTOW));
        treeItem_RoverASolution->setBackgroundColor(1, solutionColors[roverA.flag_carrSoln % 4]);
        treeItem_RoverASolution->setText(1, roverA.getCarrSolnString());
        treeItem_RoverADiffSoln->setText(1, QString::number(roverA.flag_diffSoln));
    }

    if (positionHistory_RoverB.isEmpty())
    {
        treeItem_RoverBITOW->setText(1, "N/A");
        treeItem_RoverBSolution->setText(1, "N/A");
        treeItem_RoverBDiffSoln->setText(1, "N/A");
        treeItem_RoverBSolution->setBackgroundColor(1, solutionColors[UBXMessage_RELPOSNED::UNDEFINED]);
        treeItem_RoverBSolution->setText(1, "N/A");
    }
    else
    {
        UBXMessage_RELPOSNED roverB = positionHistory_RoverB.last();

        treeItem_RoverBITOW->setText(1, QString::number(roverB.iTOW));
        treeItem_RoverBSolution->setBackgroundColor(1, solutionColors[roverB.flag_carrSoln % 4]);
        treeItem_RoverBSolution->setText(1, roverB.getCarrSolnString());
        treeItem_RoverBDiffSoln->setText(1, QString::number(roverB.flag_diffSoln));
    }

    treeItem_LMBNED->setText(1, QString::number(stylusTipPosition_LMB.n, 'f', 3) + ", " + QString::number(stylusTipPosition_LMB.e, 'f', 3) + ", " + QString::number(stylusTipPosition_LMB.d, 'f', 3));

    if (stylusTipPosition_LMB.valid)
    {
        treeItem_LMBNED->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_LMBNED->setBackgroundColor(1, positionInvalidColor);
    }

    treeItem_RMBNED->setText(1, QString::number(stylusTipPosition_RMB.n, 'f', 3) + ", " + QString::number(stylusTipPosition_RMB.e, 'f', 3) + ", " + QString::number(stylusTipPosition_RMB.d, 'f', 3));

    if (stylusTipPosition_RMB.valid)
    {
        treeItem_RMBNED->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_RMBNED->setBackgroundColor(1, positionInvalidColor);
    }

    double distanceTipToLMB = lastStylusTipPosition.getDistanceTo(stylusTipPosition_LMB);

    treeItem_Distance_TipToLMB->setText(1, QString::number(distanceTipToLMB, 'f', 3));

    if ((stylusTipPosition_LMB.valid) && (lastStylusTipPosition.valid))
    {
        treeItem_Distance_TipToLMB->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_Distance_TipToLMB->setBackgroundColor(1, positionInvalidColor);
    }

    double distanceTipToRMB = lastStylusTipPosition.getDistanceTo(stylusTipPosition_RMB);

    treeItem_Distance_TipToRMB->setText(1, QString::number(distanceTipToRMB, 'f', 3));

    if ((stylusTipPosition_RMB.valid) && (lastStylusTipPosition.valid))
    {
        treeItem_Distance_TipToRMB->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_Distance_TipToRMB->setBackgroundColor(1, positionInvalidColor);
    }

    double distanceLMBToRMB = stylusTipPosition_LMB.getDistanceTo(stylusTipPosition_RMB);

    treeItem_Distance_LMBToRMB->setText(1, QString::number(distanceLMBToRMB, 'f', 3));

    if ((stylusTipPosition_LMB.valid) && (stylusTipPosition_RMB.valid))
    {
        treeItem_Distance_LMBToRMB->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_Distance_LMBToRMB->setBackgroundColor(1, positionInvalidColor);
    }

    bool roverAToTipDistanceValid = true;

    if (((lastValidDistanceItem.type == DistanceItem::Type::MEASURED) &&
            (!lastValidDistanceItemTimer.isValid() ||
                (lastValidDistanceItemTimer.elapsed() > 500))) ||
            (lastValidDistanceItem.type == DistanceItem::Type::UNKNOWN))

    {
        roverAToTipDistanceValid = false;
    }

    treeItem_Distance_RoverAToTip->setText(1, QString::number(lastValidDistanceItem.distance, 'f', 3));

    if (roverAToTipDistanceValid)
    {
        treeItem_Distance_RoverAToTip->setBackgroundColor(1, positionValidColor);
    }
    else
    {
        treeItem_Distance_RoverAToTip->setBackgroundColor(1, positionInvalidColor);
    }
}



void EssentialsForm::on_spinBox_FluctuationHistoryLength_valueChanged(int)
{
    updateTreeItems();
}

#if 0
void EssentialsForm::handleVideoFrameRecording(void)
{
    if ((video_FileNameBeginning.length() != 0) && (lastMatchingRELPOSNEDiTOW != -1) && (
                (video_LastWrittenFrameITOW == -1) ||
                (lastMatchingRELPOSNEDiTOW - video_LastWrittenFrameITOW >= 100))
            )
    {
        // This block is really ugly. Sorry!
        // Just want to make video "recording" possible quickly...

        if (!video_FrameBuffer)
        {
            // Framebuffer is defined in class-level so that the same frame can be output several
            // times in the case of RELPOSNED "frame drops".
            video_FrameBuffer = new QPixmap(this->size());  // Will not be explicitly deleted, but who cares?
        }

        if (video_LastWrittenFrameITOW != -1)
        {
            // Write previous frame as many times as "frame drops" happened to keep video in sync

            while ((video_LastWrittenFrameITOW + 100) < lastMatchingRELPOSNEDiTOW)
            {
                video_FrameCounter++;
                QString fileName = video_FileNameBeginning + "_" + QString("%1").arg(video_FrameCounter, 5, 10, QChar('0')) + + ".png";
                video_FrameBuffer->save(fileName);

                video_LastWrittenFrameITOW += 100;
            }
        }

        repaint();  // To be sure that rover data shown in window is in sync (does render force repaint?)
        render(video_FrameBuffer);

        video_FrameCounter++;
        QString fileName = video_FileNameBeginning + "_" + QString("%1").arg(video_FrameCounter, 5, 10, QChar('0')) + + ".png";
        video_FrameBuffer->save(fileName);

        if (video_LastWrittenFrameITOW != -1)
        {
            // Keep frames synced to 100 ms interval even when incoming data is not (should not happen, though)
            video_LastWrittenFrameITOW += 100;
        }
        else
        {
            video_LastWrittenFrameITOW = lastMatchingRELPOSNEDiTOW - (lastMatchingRELPOSNEDiTOW % 100);
        }
    }
}
#endif

void EssentialsForm::handleVideoFrameRecording(qint64 uptime)
{
    if (video_FileNameBeginning.length() && video_WriteFrames)
    {
        // This block is really ugly. Sorry!
        // Just want to make video "recording" possible quickly...

        bool startNewClip = false;

        if (!video_FrameBuffer)
        {
            // Framebuffer is defined in class-level so that the same frame can be output several
            // times in the case of RELPOSNED "frame drops".
            video_FrameBuffer = new QPixmap(this->size());  // Will not be explicitly deleted, but who cares?

            startNewClip = true;
        }
        else if (uptime < video_LastWrittenFrameUptime)
        {
            // End video frame writing when older than last uptime is given
//            video_WriteFrames = false;
//            return;
        }
        else if ((uptime - video_LastWrittenFrameUptime) > 1000)
        {
            // Too long skip between frames -> Start new clip
            startNewClip = true;
        }

        if (startNewClip)
        {
            video_ClipIndex++;
            video_FrameCounter = 1;

            repaint();  // To be sure that rover data shown in window is in sync (does render force repaint?)
            render(video_FrameBuffer);
            QString fileName = video_FileNameBeginning + QString::number(video_ClipIndex) + "_" + QString("%1").arg(video_FrameCounter, 5, 10, QChar('0')) + + ".png";
            video_FrameBuffer->save(fileName);
            video_LastWrittenFrameUptime = uptime;
            video_ClipBaseTime = uptime;
            video_ClipDoubleTimer = 0;
        }
        else
        {
            qint64 msecTimeDiff = uptime - video_ClipBaseTime;

            bool renderFrame = true;

            while (video_ClipDoubleTimer <= msecTimeDiff)
            {
                if (renderFrame)
                {
                    repaint();  // To be sure that rover data shown in window is in sync (does render force repaint?)
                    render(video_FrameBuffer);
                    renderFrame = false;
                }

                video_FrameCounter++;
                QString fileName = video_FileNameBeginning + QString::number(video_ClipIndex) + "_" + QString("%1").arg(video_FrameCounter, 5, 10, QChar('0')) + + ".png";
                video_FrameBuffer->save(fileName);
                video_ClipDoubleTimer += (1000. / video_FPS);
            }
            video_LastWrittenFrameUptime = uptime;
        }
    }
}


void EssentialsForm::on_distanceReceived(const EssentialsForm::DistanceItem& distanceItem)
{
    // This "distanceValid"-handling is just to get rid of the ridicilous
    // invalid values returned from
    // "20hz high Accuracy 80m Laser Sensor Range finder Distance measuring module TTL interface ardunio".
    // TODO: This shouldn't actually be here but in the handling thread instead
    // as this is really a problem with the module itself.
    // At least make this optional when time allows...

    bool distanceValid = true;

    if (distanceItem.type == DistanceItem::MEASURED)
    {
        // "Disqualification rules" defined here separately to make them easier to modify
        // (module works a bit erratically to say the least so trying to find some way to filter the measurements).

        if ((distanceItem.frameStartTime - lastDistanceItemIncludingInvalid.frameStartTime) > 500)
        {
            // Module seems to think it needs to give some "valid" measurements
            // now and then when they definitely can NOT be valid
            // -> Discard measurements that are too far (in time) from the last one.

            distanceValid = false;
        }
        else if (fabs(distanceItem.distance - lastDistanceItemIncludingInvalid.distance) <= 0.001)
        {
            // If the laser is pointing in the sky or somewhere far away the module
            // seems to stay stuck to _about_ the same value it lastly was able to measure.
            // -> Discard measurements too close to the previous one

            distanceValid = false;
        }
        else if (fabs(distanceItem.distance - lastDistanceItemIncludingInvalid.distance) >= 0.01)
        {
            // Sometimes the module just returns insane values between valid ones
            // -> Discard measurements that differ too much from the previous one.

            distanceValid = false;
        }
    }

    if (distanceValid)
    {
        handleVideoFrameRecording(distanceItem.frameStartTime - 1);
        addDistanceLogItem(distanceItem);

        lastValidDistanceItem = distanceItem;
        lastValidDistanceItemTimer.start();

        soundEffect_Distance.play();
    }

    addDistanceLogItem_Unfiltered(distanceItem);

    lastDistanceItemIncludingInvalid = distanceItem;
    lastDistanceItemTimerIncludingInvalid.start();

//    handleVideoFrameRecording(distanceItem.frameStartTime);
}

void EssentialsForm::on_measuredDistanceReceived(const double& distance, qint64 frameStartTime, qint64 frameEndTime)
{
    DistanceItem distanceItem;

    distanceItem.type = DistanceItem::MEASURED;
    distanceItem.distance = distance;
    distanceItem.frameStartTime = frameStartTime;
    distanceItem.frameEndTime = frameEndTime;

    on_distanceReceived(distanceItem);
}

void EssentialsForm::addDistanceLogItem(const DistanceItem& item)
{
    if (logFile_Distances.isOpen())
    {
        QTextStream textStream(&logFile_Distances);

        QString distanceTypeString = "Unknown";

        if (item.type == DistanceItem::Type::CONSTANT)
        {
            distanceTypeString = "constant";
        }
        else if (item.type == DistanceItem::Type::MEASURED)
        {
            distanceTypeString = "measured";
        }

        // header = "Distance\tType\tUptime(Start)\tFrame time\n";
        textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" <<
                      QString::number(item.distance, 'g', 4) << "\t" <<
                      distanceTypeString << "\t" <<
                      QString::number(item.frameStartTime) << "\t" <<
                      QString::number(item.frameEndTime- item.frameStartTime) << "\n";
    }
}

void EssentialsForm::addDistanceLogItem_Unfiltered(const DistanceItem& item)
{
    if (logFile_Distances_Unfiltered.isOpen())
    {
        QTextStream textStream(&logFile_Distances_Unfiltered);

        QString distanceTypeString = "Unknown";

        if (item.type == DistanceItem::Type::CONSTANT)
        {
            distanceTypeString = "constant";
        }
        else if (item.type == DistanceItem::Type::MEASURED)
        {
            distanceTypeString = "measured";
        }

        // header = "Distance\tType\tUptime(Start)\tFrame time\n";
        textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" <<
                      QString::number(item.distance, 'g', 4) << "\t" <<
                      distanceTypeString << "\t" <<
                      QString::number(item.frameStartTime) << "\t" <<
                      QString::number(item.frameEndTime- item.frameStartTime) << "\n";
    }
}


void EssentialsForm::updateTipData(void)
{
    NEDPoint stylusTipPosition;

    double tipDistanceFromRoverA = lastValidDistanceItem.distance;

    stylusTipPosition.iTOW = lastMatchingRELPOSNEDiTOW;

    Eigen::Vector3d roverAPosNED(
            lastMatchingRoverARELPOSNED.relPosN,
            lastMatchingRoverARELPOSNED.relPosE,
            lastMatchingRoverARELPOSNED.relPosD);

    Eigen::Vector3d roverBPosNED(
            lastMatchingRoverBRELPOSNED.relPosN,
            lastMatchingRoverBRELPOSNED.relPosE,
            lastMatchingRoverBRELPOSNED.relPosD);

    Eigen::Vector3d roverBToANED = roverAPosNED - roverBPosNED;
    Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
    Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * tipDistanceFromRoverA;

    stylusTipPosition.n = stylusTipPosNED(0);
    stylusTipPosition.e = stylusTipPosNED(1);
    stylusTipPosition.d = stylusTipPosNED(2);

    // Stylus tip accuracy is now the same as rover A's
    // TODO: Could be calculated based on both rovers somehow ("worst case"/some combined value)
    stylusTipPosition.accN = lastMatchingRoverARELPOSNED.accN;
    stylusTipPosition.accE = lastMatchingRoverARELPOSNED.accE;
    stylusTipPosition.accD = lastMatchingRoverARELPOSNED.accD;

    bool valid = true;

    if (((lastValidDistanceItem.type == DistanceItem::Type::MEASURED) &&
            (!lastValidDistanceItemTimer.isValid() ||
                (lastValidDistanceItemTimer.elapsed() > 500))) ||
            (lastMatchingRELPOSNEDiTOW == -1) ||
            ((!lastMatchingRELPOSNEDiTOWTimer.isValid()) ||
                (lastMatchingRELPOSNEDiTOWTimer.elapsed() > 500)))
    {
        valid = false;
    }

    stylusTipPosition.valid = valid;

    lastStylusTipPosition = stylusTipPosition;

    positionHistory_StylusTip.append(stylusTipPosition);

    while (positionHistory_StylusTip.size() > maxPositionHistoryLength)
    {
        positionHistory_StylusTip.removeFirst();
    }

    distanceBetweenFarthestCoordinates_StylusTip =
            calcDistanceBetweenFarthestCoordinates(positionHistory_StylusTip, ui->spinBox_FluctuationHistoryLength->value());
}



void EssentialsForm::on_horizontalScrollBar_Volume_MouseButtonTagging_valueChanged(int value)
{
    soundEffect_LMB.setVolume(value/100.);
    soundEffect_RMB.setVolume(value/100.);
    soundEffect_MMB.setVolume(value/100.);
    soundEffect_MBError.setVolume(value/100.);
    soundEffect_LMB.play();
}

void EssentialsForm::on_horizontalScrollBar_Volume_DistanceReceived_valueChanged(int value)
{
    soundEffect_Distance.setVolume(value/100.);
    soundEffect_Distance.play();
}

void EssentialsForm::on_checkBox_PlaySound_stateChanged(int arg1)
{
    soundEffect_LMB.setMuted(!arg1);
    soundEffect_MMB.setMuted(!arg1);
    soundEffect_RMB.setMuted(!arg1);
    soundEffect_MBError.setMuted(!arg1);
    soundEffect_Distance.setMuted(!arg1);
}
