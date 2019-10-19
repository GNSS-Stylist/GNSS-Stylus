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
    ui->doubleSpinBox_StylusTipDistanceFromRoverA->setValue(settings.value("StylusTipDistanceFromRoverA").toDouble());
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
    soundEffect_Error.setSource(QUrl::fromLocalFile(soundDir + "ErrorBeep.wav"));
}

EssentialsForm::~EssentialsForm()
{
    QSettings settings;
    settings.setValue("LoggingDirectory", ui->lineEdit_LoggingDirectory->text());
    settings.setValue("LoggingFileNamePrefix", ui->lineEdit_LoggingFileNamePrefix->text());
    settings.setValue("StylusTipDistanceFromRoverA", ui->doubleSpinBox_StylusTipDistanceFromRoverA->value());
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
            logFile_Tags.exists())
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

    if (logFile_Tags.exists() && (openMode == QIODevice::Append))
    {
        addTagFileHeader = false;
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
            !logFile_Tags.isOpen())
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

        textStream << "Time\tiTOW\tTag\tText\n";
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

void EssentialsForm::nmeaSentenceReceived_Base(const QByteArray& nmeaSentence)
{
    if (loggingActive)
    {
        logFile_Base_NMEA.write(nmeaSentence);
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

void EssentialsForm::nmeaSentenceReceived_RoverA(const QByteArray& nmeaSentence)
{
    if (loggingActive)
    {
        logFile_RoverA_NMEA.write(nmeaSentence);
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
        handleRELPOSNEDQueues();

        updateTreeItems();

        handleVideoFrameRecording();
    }

    if (loggingActive)
    {
        logFile_RoverA_UBX.write(ubxMessage.rawMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            logFile_RoverA_RELPOSNED.write(ubxMessage.rawMessage);
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

void EssentialsForm::nmeaSentenceReceived_RoverB(const QByteArray& nmeaSentence)
{
    if (loggingActive)
    {
        logFile_RoverB_NMEA.write(nmeaSentence);
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
        handleRELPOSNEDQueues();

        updateTreeItems();

        handleVideoFrameRecording();
    }

    if (loggingActive)
    {
        logFile_RoverB_UBX.write(ubxMessage.rawMessage);

        if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
        {
            logFile_RoverB_RELPOSNED.write(ubxMessage.rawMessage);
        }
    }
}

void EssentialsForm::postProcessingTagReceived(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag& tag)
{
    if (tag.ident == "LMB")
    {
        on_pushButton_MouseTag_clicked();
    }
    else if (tag.ident == "MMB")
    {
        on_pushButton_MouseTag_middleClicked();
    }
    else if (tag.ident == "RMB")
    {
        on_pushButton_MouseTag_rightClicked();
    }
    else
    {
        ui->comboBox_TagIdent->setEditText(tag.ident);
        ui->lineEdit_TagText->setText(tag.text);

        on_pushButton_AddTag_clicked();
    }
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
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_Base(const QByteArray&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
                     this, SLOT(ubxMessageReceived_Base(const UBXMessage&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                     this, SLOT(rtcmMessageReceived_Base(const RTCMMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_Base(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_Base(const QByteArray&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
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
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_RoverA(const QByteArray&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_RoverA(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_RoverA(const QByteArray&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
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
    QObject::connect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_RoverB(const QByteArray&)));

    QObject::connect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));
}

void EssentialsForm::disconnectUBloxDataStreamProcessorSlots_RoverB(UBloxDataStreamProcessor* ubloxDataStreamProcessor)
{
    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                     this, SLOT(nmeaSentenceReceived_RoverB(const QByteArray&)));

    QObject::disconnect(ubloxDataStreamProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&, qint64, qint64)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));
}

void EssentialsForm::connectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverA(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_RoverB(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));

    QObject::connect(postProcessingForm, SIGNAL(replayData_Tag(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag&)));
}

void EssentialsForm::disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    QObject::disconnect(postProcessingForm, SIGNAL(replayData_RoverA(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverA(const UBXMessage&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_RoverB(const UBXMessage&)),
                     this, SLOT(ubxMessageReceived_RoverB(const UBXMessage&)));

    QObject::disconnect(postProcessingForm, SIGNAL(replayData_Tag(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag&)),
                     this, SLOT(postProcessingTagReceived(const UBXMessage_RELPOSNED::ITOW&, const PostProcessingForm::Tag&)));
}

void EssentialsForm::on_pushButton_AddTag_clicked()
{
    if (loggingActive)
    {
        if (lastMatchingRELPOSNEDiTOW != lastTaggedRELPOSNEDiTOW)
        {
            // Only allow single tag for any iTOW
            // TODO: This could be removed if post-processing is modified to allow multiple tags with the same iTOW (QMap -> QMultiMap there?)

            QTextStream textStream(&logFile_Tags);

            textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" << QString::number(lastMatchingRELPOSNEDiTOW) << "\t"
                       << ui->comboBox_TagIdent->lineEdit()->text() << "\t" << ui->lineEdit_TagText->text() << "\n";

            treeItem_LastTag->setText(1, ui->comboBox_TagIdent->lineEdit()->text() +  "; " + ui->lineEdit_TagText->text());

            lastTaggedRELPOSNEDiTOW = lastMatchingRELPOSNEDiTOW;
        }
        else
        {
            QMessageBox msgBox;
            msgBox.setText("Tag not added. Only one tag allowed for any iTOW.");

            msgBox.exec();
        }
    }
    else
    {
        treeItem_LastTag->setText(1, "Logging not active!");
    }
}

void EssentialsForm::on_pushButton_MouseTag_clicked()
{
    addMouseButtonTag("LMB", soundEffect_LMB);
}

void EssentialsForm::on_pushButton_MouseTag_rightClicked()
{
    addMouseButtonTag("RMB", soundEffect_RMB);
}

void EssentialsForm::on_pushButton_MouseTag_middleClicked()
{
    addMouseButtonTag("MMB", soundEffect_MMB);
}

void EssentialsForm::addMouseButtonTag(const QString& tagtext, QSoundEffect& soundEffect)
{
    if (loggingActive)
    {
        treeItem_LastTag->setText(1, tagtext);

        if (lastMatchingRELPOSNEDiTOW != lastTaggedRELPOSNEDiTOW)
        {
            // Only allow single tag for any iTOW
            // TODO: This could be removed if post-processing is modified to allow multiple tags with the same iTOW (QMap -> QMultiMap there?)

            QTextStream textStream(&logFile_Tags);

            textStream << QTime::currentTime().toString("hh:mm:ss:zzz") << "\t" << QString::number(lastMatchingRELPOSNEDiTOW) << "\t"
                       << tagtext << "\t" << "" << "\n";

            if (ui->checkBox_MouseButtonTaggingSound->isChecked())
            {
                soundEffect.play();
            }

            ui->pushButton_MouseTag->setText("Tagged " + tagtext);
            lastTaggedRELPOSNEDiTOW = lastMatchingRELPOSNEDiTOW;
        }
        else
        {
            if (ui->checkBox_MouseButtonTaggingSound->isChecked())
            {
                soundEffect_Error.play();
            }
            ui->pushButton_MouseTag->setText("Same ITOW!");
        }
    }
    else
    {
        if (ui->checkBox_MouseButtonTaggingSound->isChecked())
        {
            soundEffect_Error.play();
        }
        ui->pushButton_MouseTag->setText("Logging not active!");

        treeItem_LastTag->setText(1, "Logging not active!");
    }
}


void EssentialsForm::handleRELPOSNEDQueues(void)
{
    bool matchingiTOWFound = false;

    UBXMessage_RELPOSNED lastMatchingRoverARELPOSNED;
    UBXMessage_RELPOSNED lastMatchingRoverBRELPOSNED;

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

                lastMatchingRoverARELPOSNED = roverARELPOSNED;
                lastMatchingRoverBRELPOSNED = roverBRELPOSNED;

                // Vector pointing from rover B to A
                double diffN = roverARELPOSNED.relPosN - roverBRELPOSNED.relPosN;
                double diffE = roverARELPOSNED.relPosE - roverBRELPOSNED.relPosE;
                double diffD = roverARELPOSNED.relPosD - roverBRELPOSNED.relPosD;

                double vectorLength = sqrt(diffN * diffN + diffE * diffE + diffD * diffD);

                // Unit vector based on the rover B->A-vector
                double unitVecN = diffN / vectorLength;
                double unitVecE = diffE / vectorLength;
                double unitVecD = diffD / vectorLength;

                NEDPoint stylusTipPosition;

                double tipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA->value();

                stylusTipPosition.iTOW = lastMatchingRELPOSNEDiTOW;

                // Vector pointing from rover A to tip
                stylusTipPosition.n = unitVecN * tipDistanceFromRoverA;
                stylusTipPosition.e = unitVecE * tipDistanceFromRoverA;
                stylusTipPosition.d = unitVecD * tipDistanceFromRoverA;

                // Add rover A position to stylus tip position vector to get final position
                stylusTipPosition.n += roverARELPOSNED.relPosN;
                stylusTipPosition.e += roverARELPOSNED.relPosE;
                stylusTipPosition.d += roverARELPOSNED.relPosD;

                // Stylus tip accuracy is now the same as rover A's
                // TODO: Could be calculated based on both rovers somehow ("worst case"/some combined vlaue)
                stylusTipPosition.accN = roverARELPOSNED.accN;
                stylusTipPosition.accE = roverARELPOSNED.accE;
                stylusTipPosition.accD = roverARELPOSNED.accD;

                positionHistory_StylusTip.append(stylusTipPosition);

                while (positionHistory_StylusTip.size() > maxPositionHistoryLength)
                {
                    positionHistory_StylusTip.removeFirst();
                }

                distanceBetweenFarthestCoordinates_StylusTip = calcDistanceBetweenFarthestCoordinates(positionHistory_StylusTip, ui->spinBox_FluctuationHistoryLength->value());
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
    this->iTOW = static_cast<int>(relposnedMessage.iTOW);

    this->n = relposnedMessage.relPosN;
    this->e = relposnedMessage.relPosE;
    this->d = relposnedMessage.relPosD;

    this->accN = relposnedMessage.accN;
    this->accE = relposnedMessage.accE;
    this->accD = relposnedMessage.accD;
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

        treeItem_StylusTipNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipNED->setText(0, "Tip NED");

        treeItem_StylusTipXYZ = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipXYZ->setText(0, "Tip XYZ");

        treeItem_StylusTipAccNED = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipAccNED->setText(0, "Tip AccNED");

        treeItem_StylusTipAccXYZ = new QTreeWidgetItem(ui->treeWidget);
        treeItem_StylusTipAccXYZ->setText(0, "Tip AccXYZ");

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

        treeItemsCreated = true;
    }

    treeItem_DistanceBetweenFarthestCoordinates_RoverA->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverA, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_RoverB->setText(1, QString::number(distanceBetweenFarthestCoordinates_RoverB, 'f', 3));
    treeItem_DistanceBetweenFarthestCoordinates_StylusTip->setText(1, QString::number(distanceBetweenFarthestCoordinates_StylusTip, 'f', 3));
    treeItem_DistanceBetweenRovers->setText(1, QString::number(distanceBetweenRovers, 'f', 3));

    if (positionHistory_StylusTip.isEmpty())
    {
        treeItem_StylusTipNED->setText(1, "N/A");
        treeItem_StylusTipXYZ->setText(1, "N/A");
        treeItem_StylusTipAccNED->setText(1, "N/A");
        treeItem_StylusTipAccXYZ->setText(1, "N/A");
    }
    else
    {
        NEDPoint tip = positionHistory_StylusTip.last();

        treeItem_StylusTipNED->setText(1, QString::number(tip.n, 'f', 3) + ", " + QString::number(tip.e, 'f', 3) + ", " + QString::number(tip.d, 'f', 3));
        treeItem_StylusTipXYZ->setText(1, QString::number(tip.e, 'f', 3) + ", " + QString::number(tip.d, 'f', 3) + ", " + QString::number(-tip.n, 'f', 3));

        treeItem_StylusTipAccNED->setText(1, QString::number(tip.accN, 'f', 3) + ", " + QString::number(tip.accE, 'f', 3) + ", " + QString::number(tip.accD, 'f', 3));
        treeItem_StylusTipAccXYZ->setText(1, QString::number(tip.accE, 'f', 3) + ", " + QString::number(tip.accD, 'f', 3) + ", " + QString::number(tip.accN, 'f', 3));
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

}



void EssentialsForm::on_spinBox_FluctuationHistoryLength_valueChanged(int)
{
    updateTreeItems();
}

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



