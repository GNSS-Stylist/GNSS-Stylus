/*
    essentialsform.cpp (part of GNSS-Stylus)
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

    ui->spinBox_NumberOfRovers->setValue(settings.value("NumberOfRovers").toInt());
    ui->spinBox_FluctuationHistoryLength->setValue(settings.value("FluctuationHistoryLength").toInt());
    ui->horizontalScrollBar_Volume_MouseButtonTagging->setValue(settings.value("Volume_MouseButtonTagging").toInt());
    ui->horizontalScrollBar_Volume_DistanceReceived->setValue(settings.value("Volume_DistanceReceived").toInt());

    // Just some valid values
    const double defaultAntennaLocations[3][3] = {
        { 1, 0, 0 },
        { -1, -1, 0 },
        { -1, 1, 0 }
    };

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            QString settingKey = "Essentials_AntennaLocations_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            QString defaultValue = QString::number(defaultAntennaLocations[row][column], 'g', 3);

            ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->setText(settings.value(settingKey, defaultValue).toString());
        }
    }

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

    qreal volume = ui->horizontalScrollBar_Volume_MouseButtonTagging->value() / 100.;
    soundEffect_LMB.setVolume(volume);
    soundEffect_RMB.setVolume(volume);
    soundEffect_MMB.setVolume(volume);
    soundEffect_MBError.setVolume(volume);

    volume = ui->horizontalScrollBar_Volume_DistanceReceived->value() / 100.;
    soundEffect_Distance.setVolume(volume);

    ui->checkBox_PlaySound->setChecked(settings.value("PlaySound").toBool());
    on_checkBox_PlaySound_stateChanged(ui->checkBox_PlaySound->checkState());

    fileDialog_AntennaLocations_Load.setFileMode(QFileDialog::ExistingFile);

    QStringList antennaLocationsFilters;

    antennaLocationsFilters << "Antenna locations-files (*.AntennaLocations)"
            << "Any files (*)";

    fileDialog_AntennaLocations_Load.setNameFilters(antennaLocationsFilters);


    fileDialog_AntennaLocations_Save.setFileMode(QFileDialog::AnyFile);
    fileDialog_AntennaLocations_Save.setDefaultSuffix("AntennaLocations");

    fileDialog_AntennaLocations_Save.setNameFilters(antennaLocationsFilters);

    lidarTimeoutTimer.setSingleShot(true);

    connect(&lidarTimeoutTimer, &QTimer::timeout, this, &EssentialsForm::on_lidarTimeoutTimerTimeout);

    connect(&sideBarUpdateTimer, &QTimer::timeout, this, &EssentialsForm::on_sideBarUpdateTimerTimeout);
    sideBarUpdateTimer.start(10);
}

EssentialsForm::~EssentialsForm()
{
    QSettings settings;
    settings.setValue("LoggingDirectory", ui->lineEdit_LoggingDirectory->text());
    settings.setValue("LoggingFileNamePrefix", ui->lineEdit_LoggingFileNamePrefix->text());

    settings.setValue("NumberOfRovers", ui->spinBox_NumberOfRovers->value());
    settings.setValue("FluctuationHistoryLength", ui->spinBox_FluctuationHistoryLength->value());

    settings.setValue("PlaySound", ui->checkBox_PlaySound->isChecked());
    settings.setValue("Volume_MouseButtonTagging", ui->horizontalScrollBar_Volume_MouseButtonTagging->value());
    settings.setValue("Volume_DistanceReceived", ui->horizontalScrollBar_Volume_DistanceReceived->value());

    for (int row = 0; row < 3; row++)
    {
        for (int column = 0; column < 3; column++)
        {
            QString settingKey = "Essentials_AntennaLocations_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            settings.setValue(settingKey, ui->tableWidget_AntennaLocations_LOSolver->item(row, column)->text());
        }
    }

    delete ui;
}

void EssentialsForm::closeAllLogFiles(void)
{
    logFile_Base_Raw.close();
    logFile_Base_NMEA.close();
    logFile_Base_UBX.close();
    logFile_Base_RTCM.close();

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        rovers[i].logFile_Raw.close();
        rovers[i].logFile_NMEA.close();
        rovers[i].logFile_UBX.close();
        rovers[i].logFile_RELPOSNED.close();
    }

    logFile_Tags.close();
    logFile_Distances.close();
    logFile_Distances_Unfiltered.close();
    logFile_Sync.close();
    logFile_Lidar.close();
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

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        QString roverString = "_Rover" + getRoverIdentString(i);

        rovers[i].logFile_Raw.setFileName(fileNameBeginning + roverString + ".raw");
        rovers[i].logFile_NMEA.setFileName(fileNameBeginning + roverString + ".NMEA");
        rovers[i].logFile_UBX.setFileName(fileNameBeginning + roverString + ".ubx");
        rovers[i].logFile_RELPOSNED.setFileName(fileNameBeginning + roverString + "_RELPOSNED.ubx");
    }

    logFile_Tags.setFileName(fileNameBeginning + "_tags.tags");

    logFile_Distances.setFileName(fileNameBeginning + ".distances");
    logFile_Distances_Unfiltered.setFileName(fileNameBeginning + "_Unfiltered.distances");
    logFile_Sync.setFileName(fileNameBeginning + ".sync");
    logFile_Lidar.setFileName(fileNameBeginning + ".lidar");

    QIODevice::OpenMode openMode = QIODevice::WriteOnly;

    bool roverFileExists = false;

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (    rovers[i].logFile_Raw.exists() ||
                rovers[i].logFile_NMEA.exists() ||
                rovers[i].logFile_UBX.exists() ||
                rovers[i].logFile_RELPOSNED.exists())
        {
            roverFileExists = true;
            break;
        }
    }

    if (roverFileExists ||
            logFile_Base_Raw.exists() ||
            logFile_Base_NMEA.exists() ||
            logFile_Base_UBX.exists() ||
            logFile_Base_RTCM.exists() ||
            logFile_Tags.exists() ||
            logFile_Distances.exists() ||
            logFile_Distances_Unfiltered.exists() ||
            logFile_Sync.exists() ||
            logFile_Lidar.exists())
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

    bool logFilesOpen = true;

    logFilesOpen = logFilesOpen && logFile_Base_Raw.open(openMode);
    logFilesOpen = logFilesOpen && logFile_Base_NMEA.open(openMode);
    logFilesOpen = logFilesOpen && logFile_Base_UBX.open(openMode);
    logFilesOpen = logFilesOpen && logFile_Base_RTCM.open(openMode);

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        logFilesOpen = logFilesOpen && rovers[i].logFile_Raw.open(openMode);
        logFilesOpen = logFilesOpen && rovers[i].logFile_NMEA.open(openMode);
        logFilesOpen = logFilesOpen && rovers[i].logFile_UBX.open(openMode);
        logFilesOpen = logFilesOpen && rovers[i].logFile_RELPOSNED.open(openMode);
    }

    logFilesOpen = logFilesOpen && logFile_Tags.open(openMode| QIODevice::Text);
    logFilesOpen = logFilesOpen && logFile_Distances.open(openMode| QIODevice::Text);
    logFilesOpen = logFilesOpen && logFile_Distances_Unfiltered.open(openMode| QIODevice::Text);
    logFilesOpen = logFilesOpen && logFile_Sync.open(openMode| QIODevice::Text);
    logFilesOpen = logFilesOpen && logFile_Lidar.open(openMode);

    if (!logFilesOpen)
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

void EssentialsForm::serialDataReceived_Rover(const QByteArray& bytes, const unsigned int roverId)
{
    if ((loggingActive) && (roverId < sizeof(rovers) / sizeof(rovers[0])))
    {
        rovers[roverId].logFile_Raw.write(bytes);
    }
}

void EssentialsForm::nmeaSentenceReceived_Rover(const NMEAMessage& nmeaMessage, const unsigned int roverId)
{
    if ((loggingActive) && (roverId < sizeof(rovers) / sizeof(rovers[0])))
    {
        rovers[roverId].logFile_NMEA.write(nmeaMessage.rawMessage);
    }
}

void EssentialsForm::ubxMessageReceived_Rover(const UBXMessage& ubxMessage, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]) && (int(roverId) < ui->spinBox_NumberOfRovers->value()))
    {
        UBXMessage_RELPOSNED relposned(ubxMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            // "Casting" generic UBX-message to RELPOSNED was successful
            rovers[roverId].locationHistory.append(relposned);

            while (rovers[roverId].locationHistory.size() > maxLocationHistoryLength)
            {
                rovers[roverId].locationHistory.removeFirst();
            }

            rovers[roverId].distanceBetweenFarthestCoordinates = calcDistanceBetweenFarthestCoordinates(rovers[roverId].locationHistory, ui->spinBox_FluctuationHistoryLength->value());

            while ((!rovers[roverId].messageQueue_RELPOSNED.isEmpty()) && (rovers[roverId].messageQueue_RELPOSNED.head().iTOW > relposned.iTOW))
            {
                // Remove items from the queue that are from "the future".
                // This is needed for a case when (re)replaying data that has RELPOSNED-items
                // in the end of the logs (for example) for some rover(s) but not all
                // (missing sync due to item from "the future" in the head of the queue
                // -> queues fill up -> queues not handled in time).
                rovers[roverId].messageQueue_RELPOSNED.dequeue();
            }

            rovers[roverId].messageQueue_RELPOSNED.enqueue(relposned);

            handleVideoFrameRecording(ubxMessage.messageEndTime - 1);

            handleRELPOSNEDQueues();

            updateTreeItems();

    //        handleVideoFrameRecording(ubxMessage.messageEndTime);
        }

        if (loggingActive)
        {
            rovers[roverId].logFile_UBX.write(ubxMessage.rawMessage);

            if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
            {
                rovers[roverId].logFile_RELPOSNED.write(ubxMessage.rawMessage);

                QTextStream textStream(&logFile_Sync);

                QString roverString = "Rover " + getRoverIdentString(roverId);

                textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" <<
                              roverString << "\tRELPOSNED\t" << QString::number(relposned.iTOW) <<
                              "\t" << QString::number(ubxMessage.messageStartTime) << "\t" <<
                              QString::number(ubxMessage.messageEndTime - ubxMessage.messageStartTime) << "\n";
            }
        }
    }
}


void EssentialsForm::postProcessingTagReceived(const qint64 uptime, const PostProcessingForm::Tag& tag)
{
    handleVideoFrameRecording(uptime - 1);

    if (tag.ident == "LMB")
    {
        addMouseButtonTag("LMB", soundEffect_LMB, uptime);
        stylusTipLocation_LMB = lastStylusTipLocation;
    }
    else if (tag.ident == "MMB")
    {
        addMouseButtonTag("MMB", soundEffect_MMB, uptime);
        stylusTipLocation_MMB = lastStylusTipLocation;
    }
    else if (tag.ident == "RMB")
    {
        addMouseButtonTag("RMB", soundEffect_RMB, uptime);
        stylusTipLocation_RMB = lastStylusTipLocation;
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





void EssentialsForm::connectSerialThreadSlots_Rover(SerialThread* serThread, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        rovers[roverId].serialThreadConnections.insert(serThread,
            QObject::connect(serThread, &SerialThread::dataReceived,
                this, [=](const QByteArray& bytes, qint64, qint64, const SerialThread::DataReceivedEmitReason&)
                {
                    serialDataReceived_Rover(bytes, roverId);
                }
            )
        );
    }
}

void EssentialsForm::disconnectSerialThreadSlots_Rover(SerialThread* serThread, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        QList<QMetaObject::Connection> connections = rovers[roverId].serialThreadConnections.values(serThread);

        for (int i = 0; i < connections.size(); i++)
        {
            QObject::disconnect(connections.at(i));
        }
        rovers[roverId].serialThreadConnections.remove(serThread);
    }
}

void EssentialsForm::connectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        rovers[roverId].ubloxDataStreamProcessorConnections.insert(ubloxDataStreamProcessor,
            QObject::connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::nmeaSentenceReceived,
                this, [=](const NMEAMessage& message)
                {
                    nmeaSentenceReceived_Rover(message, roverId);
                }
            )
        );

        rovers[roverId].ubloxDataStreamProcessorConnections.insert(ubloxDataStreamProcessor,
            QObject::connect(ubloxDataStreamProcessor, &UBloxDataStreamProcessor::ubxMessageReceived,
                this, [=](const UBXMessage& message)
                {
                    ubxMessageReceived_Rover(message, roverId);
                }
            )
        );
    }
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        QList<QMetaObject::Connection> connections = rovers[roverId].ubloxDataStreamProcessorConnections.values(ubloxDataStreamProcessor);

        for (int i = 0; i < connections.size(); i++)
        {
            QObject::disconnect(connections.at(i));
        }
        rovers[roverId].ubloxDataStreamProcessorConnections.remove(ubloxDataStreamProcessor);
    }
}

void EssentialsForm::connectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::connect(postProcessingForm, SIGNAL(replayData_Rover(const UBXMessage&, const unsigned int)),
                     this, SLOT(ubxMessageReceived_Rover(const UBXMessage&, const unsigned int)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Tag(const qint64, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const qint64, const PostProcessingForm::Tag&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Distance(const qint64, const PostProcessingForm::DistanceItem&)),
                     this, SLOT(postProcessingDistanceReceived(const qint64, const PostProcessingForm::DistanceItem&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Lidar(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void EssentialsForm::disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Rover(const UBXMessage&, const unsigned int)),
                     this, SLOT(ubxMessageReceived_Rover(const UBXMessage&, const unsigned int)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Tag(const qint64, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const qint64, const PostProcessingForm::Tag&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Distance(const qint64, const PostProcessingForm::DistanceItem&)),
                     this, SLOT(postProcessingDistanceReceived(const qint64, const PostProcessingForm::DistanceItem&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Lidar(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
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

        treeItem_LastTag_Stylus->setText(1, ui->comboBox_TagIdent->lineEdit()->text() +  "; " + ui->lineEdit_TagText->text());

        lastTaggedRELPOSNEDiTOW = lastMatchingRELPOSNEDiTOW;
    }
    else
    {
        treeItem_LastTag_Stylus->setText(1, "Logging not active!");
    }

    treeItem_LastTag_LOSolver->setText(1, treeItem_LastTag_Stylus->text(1));
//    handleVideoFrameRecording(uptime);
}

void EssentialsForm::on_pushButton_AddTag_clicked()
{
    addTextTag();
}

void EssentialsForm::on_pushButton_MouseTag_clicked()
{
    addMouseButtonTag("LMB", soundEffect_LMB);
    stylusTipLocation_LMB = lastStylusTipLocation;
}

void EssentialsForm::on_pushButton_MouseTag_rightClicked()
{
    addMouseButtonTag("RMB", soundEffect_RMB);
    stylusTipLocation_RMB = lastStylusTipLocation;
}

void EssentialsForm::on_pushButton_MouseTag_middleClicked()
{
    addMouseButtonTag("MMB", soundEffect_MMB);
    stylusTipLocation_MMB = lastStylusTipLocation;
}

void EssentialsForm::addMouseButtonTag(const QString& tagtext, QSoundEffect& soundEffect, qint64 uptime)
{
    handleVideoFrameRecording(uptime - 1);
    if (loggingActive)
    {
        treeItem_LastTag_Stylus->setText(1, tagtext);

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

        treeItem_LastTag_Stylus->setText(1, "Logging not active!");
    }

    treeItem_LastTag_LOSolver->setText(1, treeItem_LastTag_Stylus->text(1));

//    handleVideoFrameRecording(uptime);
}


void EssentialsForm::handleRELPOSNEDQueues(void)
{
    int numOfRovers = ui->spinBox_NumberOfRovers->value();

    bool matchingiTOWFound = false;

    while (1)
    {
        bool someQueueEmpty = false;

        for (int i = 0; i < numOfRovers; i++)
        {
            someQueueEmpty = someQueueEmpty | rovers[i].messageQueue_RELPOSNED.isEmpty();
        }

        if (someQueueEmpty)
        {
            break;
        }

        // Highest ITOW value in handled rovers in the heads of the queues
        UBXMessage_RELPOSNED::ITOW highestLowestITOW = 0;

        for (int i = 0; i < numOfRovers; i++)
        {
            if (rovers[i].messageQueue_RELPOSNED.head().iTOW > highestLowestITOW)
            {
                highestLowestITOW = rovers[i].messageQueue_RELPOSNED.head().iTOW;
            }
        }

        // Discard all RELPOSNED-messages that are older (lower iTOW) than the "highest lowest"
        // (If time keeps running forward, they would never be handled).

        for (int i = 0; i < numOfRovers; i++)
        {
            while ((!rovers[i].messageQueue_RELPOSNED.isEmpty()) &&
                   (rovers[i].messageQueue_RELPOSNED.head().iTOW < highestLowestITOW))
            {
                rovers[i].messageQueue_RELPOSNED.dequeue();
            }
        }

        for (int i = 0; i < numOfRovers; i++)
        {
            someQueueEmpty = someQueueEmpty | rovers[i].messageQueue_RELPOSNED.isEmpty();
        }

        if (someQueueEmpty)
        {
            break;
        }

        bool iTOWMismatch = false;

        for (int i = 1; i < numOfRovers; i++)
        {
            if (rovers[0].messageQueue_RELPOSNED.head().iTOW != rovers[i].messageQueue_RELPOSNED.head().iTOW)
            {
                iTOWMismatch = true;
                break;
            }
        }

        if (iTOWMismatch)
        {
            // ITOW-values in the heads of the queues still doesn't match -> try again from start
            continue;
        }

        // Queues are in sync -> process

        lastMatchingRELPOSNEDiTOW = rovers[0].messageQueue_RELPOSNED.head().iTOW;
        matchingiTOWFound = true;
        lastMatchingRELPOSNEDiTOWTimer.start();

        for (int i = 0; i < numOfRovers; i++)
        {
            rovers[i].lastMatchingRoverRELPOSNED = rovers[i].messageQueue_RELPOSNED.dequeue();
        }

        updateTipData();
    }

    // Prevent groving queues too much if not all rovers are sending RELPOSNEDS
    for (int i = 0; i < numOfRovers; i++)
    {
        while (rovers[i].messageQueue_RELPOSNED.size() > 100)
        {
            rovers[i].messageQueue_RELPOSNED.dequeue();
        }
    }

    if (matchingiTOWFound)
    {
        ui->label_iTOW_BIG->setNum(lastMatchingRELPOSNEDiTOW);
        updateSideBar();

        double worstAccuracy = 0;

        for (int i = 0; i < numOfRovers; i++)
        {
            double accuracy = sqrt(rovers[i].lastMatchingRoverRELPOSNED.accN * rovers[i].lastMatchingRoverRELPOSNED.accN +
                    rovers[i].lastMatchingRoverRELPOSNED.accE * rovers[i].lastMatchingRoverRELPOSNED.accE +
                    rovers[i].lastMatchingRoverRELPOSNED.accD * rovers[i].lastMatchingRoverRELPOSNED.accD);

            if (accuracy > worstAccuracy)
            {
                worstAccuracy = accuracy;
            }
        }

        int worstAccuracyInt = static_cast<int>(worstAccuracy * 1000);

        ui->label_WorstAccuracy->setText(QString::number(worstAccuracyInt) + " mm");

        if (worstAccuracyInt > ui->progressBar_Accuracy->maximum())
        {
            worstAccuracyInt = ui->progressBar_Accuracy->maximum();
        }

        ui->progressBar_Accuracy->setValue(static_cast<int>(worstAccuracyInt));

        double distN = rovers[0].lastMatchingRoverRELPOSNED.relPosN - rovers[1].lastMatchingRoverRELPOSNED.relPosN;
        double distE = rovers[0].lastMatchingRoverRELPOSNED.relPosE - rovers[1].lastMatchingRoverRELPOSNED.relPosE;
        double distD = rovers[0].lastMatchingRoverRELPOSNED.relPosD - rovers[1].lastMatchingRoverRELPOSNED.relPosD;

        distanceBetweenRovers = sqrt(distN * distN + distE * distE + distD * distD);

        if (numOfRovers == 3)
        {
            Eigen::Vector3d antennaLocations[3];

            for (int i = 0; i < 3; i++)
            {
                antennaLocations[i](0) = rovers[i].lastMatchingRoverRELPOSNED.relPosN;
                antennaLocations[i](1) = rovers[i].lastMatchingRoverRELPOSNED.relPosE;
                antennaLocations[i](2) = rovers[i].lastMatchingRoverRELPOSNED.relPosD;
            }

            loSolverLocationOrientation.iTOW = lastMatchingRELPOSNEDiTOW;
            QElapsedTimer timer;
            timer.start();
            loSolverLocationOrientation.uptime = timer.msecsSinceReference();

            Eigen::Vector3d origin;

            bool valid = true;
            valid = valid && loSolver.setPoints(antennaLocations);
            valid = valid && loSolver.getTransformMatrix(loSolverLocationOrientation.transform);
            if (valid)
            {
                origin = loSolverLocationOrientation.transform * Eigen::Vector3d(0,0,0);
            }
            else
            {
                origin = Eigen::Vector3d(0,0,0);
            }

            loSolverLocationOrientation.n = origin(0);
            loSolverLocationOrientation.e = origin(1);
            loSolverLocationOrientation.d = origin(2);

            valid = valid && loSolver.getYawPitchRollAngles(loSolverLocationOrientation.transform, loSolverLocationOrientation.heading, loSolverLocationOrientation.pitch, loSolverLocationOrientation.roll);
            loSolverLocationOrientation.valid = valid;
        }
        else
        {
            loSolverLocationOrientation.valid = false;
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


double EssentialsForm::calcDistanceBetweenFarthestCoordinates(const QList<NEDPoint>& locationHistory, qint64 time_ms)
{
    if (locationHistory.length() >= 2)
    {
        double highN = -1e12;
        double highE = -1e12;
        double highD = -1e12;

        double lowN = 1e12;
        double lowE = 1e12;
        double lowD = 1e12;

        QList<NEDPoint>::const_iterator iter = locationHistory.end() - 1;

        qint64 lastUpTime = iter->uptime;

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
        } while ((iter != locationHistory.begin()) && ((lastUpTime - iter->uptime) < time_ms));

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


double EssentialsForm::calcDistanceBetweenFarthestCoordinates(const QList<UBXMessage_RELPOSNED>& locationHistory, qint64 time_ms)
{
    if (locationHistory.length() >= 2)
    {
        double highN = -1e12;
        double highE = -1e12;
        double highD = -1e12;

        double lowN = 1e12;
        double lowE = 1e12;
        double lowD = 1e12;

        QList<UBXMessage_RELPOSNED>::const_iterator iter = locationHistory.end() - 1;

        qint64 lastUpTime = iter->messageStartTime;

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
        } while ((iter != locationHistory.begin()) && ((lastUpTime - iter->messageStartTime) < time_ms));

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
        treeItem_RoverAITOW_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_RoverAITOW_Stylus->setText(0, "Rover A ITOW");

        treeItem_RoverBITOW_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_RoverBITOW_Stylus->setText(0, "Rover B ITOW");

        treeItem_RoverASolutionStatus_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_RoverASolutionStatus_Stylus->setText(0, "Rover A solution status");

        treeItem_RoverBSolutionStatus_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_RoverBSolutionStatus_Stylus->setText(0, "Rover B solution status");

        treeItem_AccNED_StylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_AccNED_StylusTip->setText(0, "Tip AccNED");

/*
        treeItem_DistanceBetweenFarthestCoordinates_RoverA = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_DistanceBetweenFarthestCoordinates_RoverA->setText(0, "Pos fluctuation Rover A");

        treeItem_DistanceBetweenFarthestCoordinates_RoverB = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_DistanceBetweenFarthestCoordinates_RoverB->setText(0, "Pos fluctuation Rover B");

        treeItem_DistanceBetweenFarthestCoordinates_StylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_DistanceBetweenFarthestCoordinates_StylusTip->setText(0, "Pos fluctuation Stylus tip");
*/
        treeItem_DistanceBetweenRovers_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_DistanceBetweenRovers_Stylus->setText(0, "Distance between rovers");

        treeItem_LastTag_Stylus = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_LastTag_Stylus->setText(0, "Last tag");

        treeItem_NED_StylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_NED_StylusTip->setText(0, "Tip NED");

        treeItem_NED_LMB_StylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_NED_LMB_StylusTip->setText(0, "LMB NED");

        treeItem_NED_RMB_StylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_NED_RMB_StylusTip->setText(0, "RMB NED");

        treeItem_Distance_StylusTipToLMB = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_Distance_StylusTipToLMB->setText(0, "Distance tip<->LMB");

        treeItem_Distance_StylusTipToRMB = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_Distance_StylusTipToRMB->setText(0, "Distance tip<->RMB");

        treeItem_Distance_StylusTip_LMBToRMB = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_Distance_StylusTip_LMBToRMB->setText(0, "Distance LMB<->RMB");

        treeItem_Distance_RoverAToStylusTip = new QTreeWidgetItem(ui->treeWidget_Stylus);
        treeItem_Distance_RoverAToStylusTip->setText(0, "Distance RoverA<->Tip");


        treeItem_RoverAITOW_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverAITOW_LOSolver->setText(0, "Rover A ITOW");
        treeItem_RoverBITOW_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverBITOW_LOSolver->setText(0, "Rover B ITOW");
        treeItem_RoverCITOW_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverCITOW_LOSolver->setText(0, "Rover C ITOW");

        treeItem_RoverASolutionStatus_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverASolutionStatus_LOSolver->setText(0, "Rover A solution status");
        treeItem_RoverBSolutionStatus_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverBSolutionStatus_LOSolver->setText(0, "Rover B solution status");
        treeItem_RoverCSolutionStatus_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_RoverCSolutionStatus_LOSolver->setText(0, "Rover C solution status");

        treeItem_LastTag_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_LastTag_LOSolver->setText(0, "Last tag");

        treeItem_NED_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_NED_LOSolver->setText(0, "Location (NED)");

        treeItem_Heading_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_Heading_LOSolver->setText(0, "Heading");

        treeItem_Pitch_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_Pitch_LOSolver->setText(0, "Pitch");

        treeItem_Roll_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_Roll_LOSolver->setText(0, "Roll");

        treeItem_LidarRoundFrequency_LOSolver = new QTreeWidgetItem(ui->treeWidget_LOSolver);
        treeItem_LidarRoundFrequency_LOSolver->setText(0, "Lidar data frequency");

        treeItemsCreated = true;
    }

/*
    treeItem_DistanceBetweenFarthestCoordinates_RoverA->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverA, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_RoverB->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverB, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_StylusTip->setText(1, QString::number(distanceBetweenFarthestCoordinates_StylusTip, 'f', 3));
*/
    treeItem_DistanceBetweenRovers_Stylus->setText(1, QString::number(distanceBetweenRovers, 'f', 3));

    treeItem_NED_StylusTip->setText(1, QString::number(lastStylusTipLocation.n, 'f', 3) + ", " + QString::number(lastStylusTipLocation.e, 'f', 3) + ", " + QString::number(lastStylusTipLocation.d, 'f', 3));
    treeItem_AccNED_StylusTip->setText(1, QString::number(lastStylusTipLocation.accN, 'f', 3) + ", " + QString::number(lastStylusTipLocation.accE, 'f', 3) + ", " + QString::number(lastStylusTipLocation.accD, 'f', 3));

    const QBrush brush_Valid = QBrush(QColor(128,255,128));
    const QBrush brush_Invalid = QBrush(QColor(255,128,128));

    if (lastStylusTipLocation.valid)
    {
        treeItem_NED_StylusTip->setBackground(1, brush_Valid);
        treeItem_AccNED_StylusTip->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_NED_StylusTip->setBackground(1, brush_Invalid);
        treeItem_AccNED_StylusTip->setBackground(1, brush_Invalid);
    }

    const QBrush solutionBrushes[4] =
    {
        QBrush(QColor(255,128,128)),
        QBrush(QColor(255,255,0)),
        QBrush(QColor(128,255,128)),
        QBrush(QColor(255,128,128))
    };

    struct
    {
        QTreeWidgetItem* treeItem_ITOW[2];
        QTreeWidgetItem* treeItem_SolutionStatus[2];
    } roverItems[3] =
    {
        {
            { treeItem_RoverAITOW_Stylus, treeItem_RoverAITOW_LOSolver },
            { treeItem_RoverASolutionStatus_Stylus, treeItem_RoverASolutionStatus_LOSolver }
        },
        {
            { treeItem_RoverBITOW_Stylus, treeItem_RoverBITOW_LOSolver },
            { treeItem_RoverBSolutionStatus_Stylus, treeItem_RoverBSolutionStatus_LOSolver }
        },
        {
            { nullptr, treeItem_RoverCITOW_LOSolver },
            { nullptr, treeItem_RoverCSolutionStatus_LOSolver }
        },
    };

    for (unsigned int i = 0; i < sizeof(rovers) / sizeof(rovers[0]); i++)
    {
        if (rovers[i].locationHistory.isEmpty())
        {
            for (unsigned int ii = 0; ii < sizeof(roverItems[i].treeItem_ITOW) / sizeof(roverItems[i].treeItem_ITOW[0]); ii++)
            {
                if (roverItems[i].treeItem_ITOW[ii])
                {
                    roverItems[i].treeItem_ITOW[ii]->setText(1, "N/A");
                }
            }
            for (unsigned int ii = 0; ii < sizeof(roverItems[i].treeItem_SolutionStatus) / sizeof(roverItems[i].treeItem_SolutionStatus[0]); ii++)
            {
                if (roverItems[i].treeItem_SolutionStatus[ii])
                {
                    roverItems[i].treeItem_SolutionStatus[ii]->setText(1, "N/A");
                    roverItems[i].treeItem_SolutionStatus[ii]->setBackground(1, solutionBrushes[UBXMessage_RELPOSNED::UNDEFINED]);
                }
            }
        }
        else
        {
            UBXMessage_RELPOSNED relposned = rovers[i].locationHistory.last();

            for (unsigned int ii = 0; ii < sizeof(roverItems[i].treeItem_ITOW) / sizeof(roverItems[i].treeItem_ITOW[0]); ii++)
            {
                if (roverItems[i].treeItem_ITOW[ii])
                {
                    roverItems[i].treeItem_ITOW[ii]->setText(1, QString::number(relposned.iTOW));
                }
            }
            for (unsigned int ii = 0; ii < sizeof(roverItems[i].treeItem_SolutionStatus) / sizeof(roverItems[i].treeItem_SolutionStatus[0]); ii++)
            {
                if (roverItems[i].treeItem_SolutionStatus[ii])
                {
                    roverItems[i].treeItem_SolutionStatus[ii]->setText(1, relposned.getCarrSolnString());
                    roverItems[i].treeItem_SolutionStatus[ii]->setBackground(1, solutionBrushes[relposned.flag_carrSoln % 4]);
                }
            }
        }
    }

    treeItem_NED_LMB_StylusTip->setText(1, QString::number(stylusTipLocation_LMB.n, 'f', 3) + ", " + QString::number(stylusTipLocation_LMB.e, 'f', 3) + ", " + QString::number(stylusTipLocation_LMB.d, 'f', 3));

    if (stylusTipLocation_LMB.valid)
    {
        treeItem_NED_LMB_StylusTip->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_NED_LMB_StylusTip->setBackground(1, brush_Invalid);
    }

    treeItem_NED_RMB_StylusTip->setText(1, QString::number(stylusTipLocation_RMB.n, 'f', 3) + ", " + QString::number(stylusTipLocation_RMB.e, 'f', 3) + ", " + QString::number(stylusTipLocation_RMB.d, 'f', 3));

    if (stylusTipLocation_RMB.valid)
    {
        treeItem_NED_RMB_StylusTip->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_NED_RMB_StylusTip->setBackground(1, brush_Invalid);
    }

    double distanceTipToLMB = lastStylusTipLocation.getDistanceTo(stylusTipLocation_LMB);

    treeItem_Distance_StylusTipToLMB->setText(1, QString::number(distanceTipToLMB, 'f', 3));

    if ((stylusTipLocation_LMB.valid) && (lastStylusTipLocation.valid))
    {
        treeItem_Distance_StylusTipToLMB->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_Distance_StylusTipToLMB->setBackground(1, brush_Invalid);
    }

    double distanceTipToRMB = lastStylusTipLocation.getDistanceTo(stylusTipLocation_RMB);

    treeItem_Distance_StylusTipToRMB->setText(1, QString::number(distanceTipToRMB, 'f', 3));

    if ((stylusTipLocation_RMB.valid) && (lastStylusTipLocation.valid))
    {
        treeItem_Distance_StylusTipToRMB->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_Distance_StylusTipToRMB->setBackground(1, brush_Invalid);
    }

    double distanceLMBToRMB = stylusTipLocation_LMB.getDistanceTo(stylusTipLocation_RMB);

    treeItem_Distance_StylusTip_LMBToRMB->setText(1, QString::number(distanceLMBToRMB, 'f', 3));

    if ((stylusTipLocation_LMB.valid) && (stylusTipLocation_RMB.valid))
    {
        treeItem_Distance_StylusTip_LMBToRMB->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_Distance_StylusTip_LMBToRMB->setBackground(1, brush_Invalid);
    }

    bool roverAToTipDistanceValid = true;

    if (((lastValidDistanceItem.type == DistanceItem::Type::MEASURED) &&
            (!lastValidDistanceItemTimer.isValid() ||
                (lastValidDistanceItemTimer.elapsed() > 500))) ||
            (lastValidDistanceItem.type == DistanceItem::Type::UNKNOWN))

    {
        roverAToTipDistanceValid = false;
    }

    treeItem_Distance_RoverAToStylusTip->setText(1, QString::number(lastValidDistanceItem.distance, 'f', 3));

    if (roverAToTipDistanceValid)
    {
        treeItem_Distance_RoverAToStylusTip->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_Distance_RoverAToStylusTip->setBackground(1, brush_Invalid);
    }

    if (loSolverLocationOrientation.valid)
    {
        treeItem_NED_LOSolver->setText(1, QString::number(loSolverLocationOrientation.n, 'f', 3) + ", " + QString::number(loSolverLocationOrientation.e, 'f', 3) + ", " + QString::number(loSolverLocationOrientation.d, 'f', 3));

        if (loSolverLocationOrientation.heading > 0)
        {
            treeItem_Heading_LOSolver->setText(1, QString::number(fmod((loSolverLocationOrientation.heading * 360 / (2. * M_PI) + 360), 360), 'f', 3));
        }
        else
        {
            treeItem_Heading_LOSolver->setText(1, QString::number(fmod((loSolverLocationOrientation.heading * 360 / (2. * M_PI) + 360), 360), 'f', 3) + " (" +
                                               QString::number(loSolverLocationOrientation.heading * 360 / (2. * M_PI), 'f', 3) + ")");
        }

        treeItem_Pitch_LOSolver->setText(1, QString::number(loSolverLocationOrientation.pitch * 360 / (2. * M_PI), 'f', 3));
        treeItem_Roll_LOSolver->setText(1, QString::number(loSolverLocationOrientation.roll * 360 / (2. * M_PI), 'f', 3));

        treeItem_NED_LOSolver->setBackground(1, brush_Valid);
        treeItem_Heading_LOSolver->setBackground(1, brush_Valid);
        treeItem_Pitch_LOSolver->setBackground(1, brush_Valid);
        treeItem_Roll_LOSolver->setBackground(1, brush_Valid);
    }
    else
    {
        treeItem_NED_LOSolver->setText(1, "N/A");
        treeItem_Heading_LOSolver->setText(1, "N/A");
        treeItem_Pitch_LOSolver->setText(1, "N/A");
        treeItem_Roll_LOSolver->setText(1, "N/A");

        treeItem_NED_LOSolver->setBackground(1, brush_Invalid);
        treeItem_Heading_LOSolver->setBackground(1, brush_Invalid);
        treeItem_Pitch_LOSolver->setBackground(1, brush_Invalid);
        treeItem_Roll_LOSolver->setBackground(1, brush_Invalid);
    }

    if (lidarTimeout)
    {
        treeItem_LidarRoundFrequency_LOSolver->setText(1, "0");
        treeItem_LidarRoundFrequency_LOSolver->setBackground(1, brush_Invalid);
    }
    else
    {
        treeItem_LidarRoundFrequency_LOSolver->setText(1, QString::number(lidarRoundFrequency, 'f', 1));
        treeItem_LidarRoundFrequency_LOSolver->setBackground(1, brush_Valid);
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
        else if (fabs(distanceItem.distance - lastValidDistanceItem.distance) <= 0.001)
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
    NEDPoint stylusTipLocation;

    double tipDistanceFromRoverA = lastValidDistanceItem.distance;

    stylusTipLocation.iTOW = lastMatchingRELPOSNEDiTOW;

    Eigen::Vector3d roverAPosNED(
            rovers[0].lastMatchingRoverRELPOSNED.relPosN,
            rovers[0].lastMatchingRoverRELPOSNED.relPosE,
            rovers[0].lastMatchingRoverRELPOSNED.relPosD);

    Eigen::Vector3d roverBPosNED(
            rovers[1].lastMatchingRoverRELPOSNED.relPosN,
            rovers[1].lastMatchingRoverRELPOSNED.relPosE,
            rovers[1].lastMatchingRoverRELPOSNED.relPosD);

    Eigen::Vector3d roverBToANED = roverAPosNED - roverBPosNED;
    Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
    Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * tipDistanceFromRoverA;

    stylusTipLocation.n = stylusTipPosNED(0);
    stylusTipLocation.e = stylusTipPosNED(1);
    stylusTipLocation.d = stylusTipPosNED(2);

    // Stylus tip accuracy is now the same as rover A's
    // TODO: Could be calculated based on both rovers somehow ("worst case"/some combined value)
    stylusTipLocation.accN = rovers[0].lastMatchingRoverRELPOSNED.accN;
    stylusTipLocation.accE = rovers[0].lastMatchingRoverRELPOSNED.accE;
    stylusTipLocation.accD = rovers[0].lastMatchingRoverRELPOSNED.accD;

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

    stylusTipLocation.valid = valid;

    lastStylusTipLocation = stylusTipLocation;

    locationHistory_StylusTip.append(stylusTipLocation);

    while (locationHistory_StylusTip.size() > maxLocationHistoryLength)
    {
        locationHistory_StylusTip.removeFirst();
    }

    distanceBetweenFarthestCoordinates_StylusTip =
            calcDistanceBetweenFarthestCoordinates(locationHistory_StylusTip, ui->spinBox_FluctuationHistoryLength->value());
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

QString EssentialsForm::getRoverIdentString(const unsigned int roverId)
{
    if (roverId < ('X' - 'A'))
    {
        return QString(char('A' + (char)roverId));
    }
    else
    {
        // Should not happen
        return("X");
    }
}

void EssentialsForm::on_tableWidget_AntennaLocations_LOSolver_cellChanged(int row, int column)
{
    (void) row;
    (void) column;

    updateLOSolverReferencePointLocations();
}


bool EssentialsForm::updateLOSolverReferencePointLocations(void)
{
    Eigen::Vector3d antennaLocations[3];

    for (int roverIndex = 0; roverIndex < 3; roverIndex++)
    {
        bool ok;
        for (int valueIndex = 0; valueIndex < 3; valueIndex++)
        {
            antennaLocations[roverIndex](valueIndex) = ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, valueIndex)->text().toDouble(&ok);

            if (!ok)
            {
                loSolver.init();
                return false;
            }
        }
    }

    if (!loSolver.setReferencePoints(antennaLocations))
    {
        return false;
    }

    return true;
}

static QString getAntennaLocationsFileHeaderLine(void)
{
    QString line = "Rover\tCoord_N\tCoord_E\tCoord_D";
    return line;
}

void EssentialsForm::loadAntennaLocations(const QString fileName)
{
    QMessageBox msgBox;
    QFileInfo fileInfo(fileName);

    QFile antennaLocationsFile;
    antennaLocationsFile.setFileName(fileName);
    if (antennaLocationsFile.open(QIODevice::ReadOnly))
    {
        QTextStream textStream(&antennaLocationsFile);

        QString headerLine = textStream.readLine();

        if (headerLine.compare(getAntennaLocationsFileHeaderLine(), Qt::CaseInsensitive))
        {
            msgBox.setText("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Data not read.");
            msgBox.exec();
            antennaLocationsFile.close();
            return;
        }

        QString coordItems[3][3];

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            QString dataLine = textStream.readLine();

            QStringList dataLineItems = dataLine.split("\t");

            if (dataLineItems.count() != (1 + 3))
            {
                msgBox.setText("Error: File's \"" + fileInfo.fileName() + "\" line " + QString::number(roverIndex + 1) +
                           " doesn't have correct number of items (4). Data not read.");
                msgBox.exec();

                antennaLocationsFile.close();
                return;
            }
            else
            {
                QString expectedRoverIdent = "Rover " + getRoverIdentString(roverIndex);

                if (expectedRoverIdent.compare(dataLineItems[0], Qt::CaseInsensitive))
                {
                    msgBox.setText("Error: File's \"" + fileInfo.fileName() + "\" line " + QString::number(roverIndex + 1) +
                               " Rover ident string error. Data not read.");
                    msgBox.exec();

                    return;
                }

                for (int i = 0; i < 3; i++)
                {
                    coordItems[roverIndex][i] = dataLineItems[i + 1];
                }
            }
        }

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            for (int coordIndex = 0; coordIndex < 3; coordIndex++)
            {
                ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, coordIndex)->setText(coordItems[roverIndex][coordIndex]);
            }
        }
    }
    else
    {
        msgBox.setText("Error: can't open file \"" + fileInfo.fileName() + "\". Data not read.");
        msgBox.exec();
    }
}


void EssentialsForm::on_pushButton_LoadAntennaLocations_clicked()
{
    QMessageBox msgBox;

    if (fileDialog_AntennaLocations_Load.exec())
    {
        QStringList fileNames = fileDialog_AntennaLocations_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_AntennaLocations_Load.setDirectory(QFileInfo(fileNames[0]).path());

            loadAntennaLocations(fileNames[0]);
        }
        else
        {
            msgBox.setText("Warning: No antenna locations file selected. Data not read.");
            msgBox.exec();
        }
    }
}

void EssentialsForm::on_pushButton_SaveAntennaLocations_clicked()
{
    QMessageBox msgBox;

    if (fileDialog_AntennaLocations_Save.exec())
    {
        QStringList fileNameList = fileDialog_AntennaLocations_Save.selectedFiles();

        if (fileNameList.length() != 1)
        {
            msgBox.setText("Error: Multiple file selection not supported. Antenna locations not saved.");
            msgBox.exec();
            return;
        }

        QFile antennaLocationsFile;

        antennaLocationsFile.setFileName(fileNameList[0]);

        if (antennaLocationsFile.exists())
        {
            QMessageBox overwriteMsgBox;
            overwriteMsgBox.setText("File already exists.");
            overwriteMsgBox.setInformativeText("How to proceed?");

            QPushButton *overwriteButton = overwriteMsgBox.addButton(tr("Overwrite"), QMessageBox::ActionRole);
            QPushButton *cancelButton = overwriteMsgBox.addButton(QMessageBox::Cancel);

            overwriteMsgBox.setDefaultButton(cancelButton);

            overwriteMsgBox.exec();

            if (overwriteMsgBox.clickedButton() != overwriteButton)
            {
                // msgBox.setText("Antenna locations not saved.");
                // msgBox.exec();
                return;
            }
        }

        if (!antennaLocationsFile.open(QIODevice::WriteOnly))
        {
            msgBox.setText("Error: Can't open antenna locations file.");
            msgBox.exec();
            return;
        }

        QTextStream textStream(&antennaLocationsFile);

        textStream << getAntennaLocationsFileHeaderLine() << "\n";

        for (int roverIndex = 0; roverIndex < 3; roverIndex++)
        {
            QString expectedRoverIdent = "Rover " + getRoverIdentString(roverIndex);

            textStream << expectedRoverIdent;

            for (int i = 0; i < 3; i++)
            {
                textStream << "\t" << ui->tableWidget_AntennaLocations_LOSolver->item(roverIndex, i)->text();
            }

            textStream << "\n";
        }
    }

}

void EssentialsForm::connectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::connect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void EssentialsForm::disconnectRPLidarThreadSlots(RPLidarThread* rpLidarThread)
{
    QObject::disconnect(rpLidarThread, SIGNAL(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)),
                     this, SLOT(distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>&, qint64, qint64)));
}

void EssentialsForm::distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime)
{
    (void) data;

    int timeDiff = endTime - startTime;

    if (timeDiff > 0)
    {
        lidarTimeout = false;
        lidarTimeoutTimer.start(1000);

        lidarRoundFrequency = 1000. / timeDiff;
    }
    else
    {
        // Indicate strange times as timeout
        lidarTimeout = true;
        lidarTimeoutTimer.stop();
        lidarRoundFrequency = 0;
    }

    if (logFile_Lidar.isOpen())
    {
        // Log as binary data. This already makes about
        // 3600 * 16000 * 3 * 4 + 3600 × 10 * (3 * 4 + 2* 8) = 692 208 000 bytes per hour

        QDataStream dataStream(&logFile_Lidar);
        dataStream.setFloatingPointPrecision(QDataStream::SinglePrecision);

        unsigned int dataType = 1;  // Datatype for possible future extensions (like compression?)
        unsigned int numOfItems = data.size();
        unsigned int dataChunkLength = sizeof(numOfItems) + sizeof(startTime) + sizeof(endTime) + numOfItems * 3 * sizeof(float);

        dataStream << dataType << dataChunkLength;

//        qint64 chunkStartFileSize = logFile_Lidar.pos();

        dataStream << numOfItems << startTime << endTime;

        for (int i = 0; i < data.size(); i++)
        {
            dataStream << data[i].distance << data[i].angle << data[i].quality;
        }

//        qint64 chunkEndFileSize = logFile_Lidar.pos();
//        Q_ASSERT(chunkEndFileSize - chunkStartFileSize == dataChunkLength);
    }

    updateTreeItems();
}


void EssentialsForm::on_lidarTimeoutTimerTimeout()
{
    lidarTimeout = true;
    updateTreeItems();
}

void EssentialsForm::on_sideBarUpdateTimerTimeout()
{
    updateSideBar();
}

void EssentialsForm::updateSideBar(void)
{
    QElapsedTimer uptimeTimer;
    uptimeTimer.start();
    qint64 uptime = uptimeTimer.msecsSinceReference();

    QString uptimeString = QString::number(uptime);
    QString lastITOWString = QString::number(lastMatchingRELPOSNEDiTOW);

    QString sideString;

    bool firstLine = true;

    for (int i = uptimeString.length() - 1 - 6; i < uptimeString.length(); i++)
    {
        if (!firstLine)
        {
            sideString.append("\n");
        }
        else
        {
            firstLine = false;
        }

        if (i < 0)
        {
            sideString.append("x");
        }
        else
        {
            sideString.append(uptimeString[i]);
        }
    }

    sideString.append("\n");

    for (int i = lastITOWString.length() - 1 - 6; i < lastITOWString.length(); i++)
    {
        if (i < 0)
        {
            sideString.append("\nx");
        }
        else
        {
            sideString.append("\n");
            sideString.append(lastITOWString[i]);
        }
    }

    ui->label_Side->setText(sideString);
}





