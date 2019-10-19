/*
    postprocessform.cpp (part of GNSS-Stylus)
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

#include <memory>
#include <math.h>

#include <QTime>
#include <QSettings>
#include <QTextStream>
#include <QMessageBox>

#include "postprocessform.h"
#include "ui_postprocessform.h"
//#include "tinyspline/tinysplinecpp.h"

PostProcessingForm::PostProcessingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PostProcessingForm)
{
    ui->setupUi(this);

    QSettings settings;

    ui->doubleSpinBox_StylusTipDistanceFromRoverA->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA", "900").toDouble());
    ui->lineEdit_TagIndicatingBeginningOfNewObject->setText(settings.value("PostProcessing_TagIndicatingBeginningOfNewObject", "New object").toString());
    ui->lineEdit_TagIndicatingBeginningOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingBeginningOfObjectPoints", "LMB").toString());
    ui->lineEdit_TagIndicatingEndOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingEndOfObjectPoints", "RMB").toString());

    ui->spinBox_MaxLogLines->setValue(settings.value("PostProcessing_MaxLogLines", "1000").toInt());

    ui->checkBox_IncludeNormals->setChecked(settings.value("PostProcessing_IncludeNormals", false).toBool());

    ui->spinBox_ExpectedITOWAlignment->setValue(settings.value("PostProcessing_ExpectedITOWAlignment", "100").toInt());
    ui->checkBox_ReportMissingITOWs->setChecked(settings.value("PostProcessing_ReportMissingITOWs", false).toBool());
    ui->checkBox_ReportUnalignedITOWS->setChecked(settings.value("PostProcessing_ReportUnalignedITOWS", false).toBool());

    ui->doubleSpinBox_Movie_FPS->setValue(settings.value("PostProcessing_FPS", "30").toDouble());
}

PostProcessingForm::~PostProcessingForm()
{
    QSettings settings;

    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA", ui->doubleSpinBox_StylusTipDistanceFromRoverA->value());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfNewObject", ui->lineEdit_TagIndicatingBeginningOfNewObject->text());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfObjectPoints", ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text());
    settings.setValue("PostProcessing_TagIndicatingEndOfObjectPoints", ui->lineEdit_TagIndicatingEndOfObjectPoints->text());

    settings.setValue("PostProcessing_MaxLogLines", ui->spinBox_MaxLogLines->value());

    settings.setValue("PostProcessing_IncludeNormals", ui->checkBox_IncludeNormals->isChecked());

    settings.setValue("PostProcessing_ExpectedITOWAlignment", ui->spinBox_ExpectedITOWAlignment->value());
    settings.setValue("PostProcessing_ReportMissingITOWs", ui->checkBox_ReportMissingITOWs->isChecked());
    settings.setValue("PostProcessing_ReportUnalignedITOWS", ui->checkBox_ReportUnalignedITOWS->isChecked());

    settings.setValue("PostProcessing_FPS", ui->doubleSpinBox_Movie_FPS->value());

    delete ui;
}

void PostProcessingForm::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    if (!fileDialogsInitialized)
    {
        fileDialog_UBX.setFileMode(QFileDialog::ExistingFiles);

        QStringList roverFilters;

        roverFilters << "UBX log files (*.ubx)"
                << "Raw log files (*.raw)"
                << "Any files (*)";

        fileDialog_UBX.setNameFilters(roverFilters);

        fileDialog_Tags.setFileMode(QFileDialog::ExistingFiles);

        QStringList tagFilters;

        tagFilters << "Tag-files (*.tags)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Tags.setNameFilters(tagFilters);

        fileDialog_PointCloud.setFileMode(QFileDialog::Directory);

        fileDialog_MovieScript.setFileMode(QFileDialog::AnyFile);
        fileDialog_MovieScript.setDefaultSuffix("MovieScript");

        QStringList movieScriptFilters;

        movieScriptFilters << "moviescript files (*.moviescript)"
                << "Any files (*)";

        fileDialog_MovieScript.setNameFilters(movieScriptFilters);

        fileDialogsInitialized = true;
    }
}

void PostProcessingForm::addLogLine(const QString& line)
{
    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz");

    ui->plainTextEdit_Log->setMaximumBlockCount(ui->spinBox_MaxLogLines->value());
//    ui->plainTextEdit_Log->setCenterOnScroll(ui->checkBox_PagedScroll->isChecked());
    ui->plainTextEdit_Log->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Log->appendPlainText(timeString + ": " + line);

    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

//#include "geodetic_conv.hpp"

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverA_clicked()
{


    relposnedMessages_RoverA.clear();
    addLogLine("Rover A RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverB_clicked()
{
    relposnedMessages_RoverB.clear();
    addLogLine("Rover B RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearTagData_clicked()
{
    tags.clear();
    addLogLine("Tag data cleared.");
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverA_clicked()
{
    if (fileDialog_UBX.exec())
    {
        QStringList fileNames = fileDialog_UBX.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_UBX.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addLogLine("Reading files into rover A relposned-data...");

        currentRELPOSNEDReadingData.relposnedMessages = &relposnedMessages_RoverA;

        addRELPOSNEDFileData(fileNames);

        currentRELPOSNEDReadingData.relposnedMessages = nullptr;
    }
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverB_clicked()
{
    if (fileDialog_UBX.exec())
    {
        QStringList fileNames = fileDialog_UBX.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_UBX.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addLogLine("Reading files into rover B relposned-data...");

        currentRELPOSNEDReadingData.relposnedMessages = &relposnedMessages_RoverB;

        addRELPOSNEDFileData(fileNames);

        currentRELPOSNEDReadingData.relposnedMessages = nullptr;
    }
}


void PostProcessingForm::addRELPOSNEDFileData(const QStringList& fileNames)
{
    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile ubxFile;
        ubxFile.setFileName(fileName);
        if (ubxFile.open(QIODevice::ReadOnly))
        {
            QDataStream stream(&ubxFile);

            int64_t fileLength = ubxFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
            }
            else
            {
                // Interpreting of the RELPOSNED-data is done using UBloxDataStreamProcessor.
                // Therefore the actual messages are received using slots.

                auto fileData = std::make_unique<unsigned char[]>(static_cast<std::size_t>(fileLength));

                stream.readRawData(reinterpret_cast<char *>(fileData.get()), static_cast<int>(fileLength));

                UBloxDataStreamProcessor ubloxProcessor;

                currentRELPOSNEDReadingData.init();

                QObject::connect(&ubloxProcessor, SIGNAL(nmeaSentenceReceived(const QByteArray&)),
                                 this, SLOT(ubloxProcessor_nmeaSentenceReceived(const QByteArray&)));

                QObject::connect(&ubloxProcessor, SIGNAL(ubxMessageReceived(const UBXMessage&)),
                                 this, SLOT(ubloxProcessor_ubxMessageReceived(const UBXMessage&)));

                QObject::connect(&ubloxProcessor, SIGNAL(rtcmMessageReceived(const RTCMMessage&)),
                                 this, SLOT(ubloxProcessor_rtcmMessageReceived(const RTCMMessage&)));

                QObject::connect(&ubloxProcessor, SIGNAL(ubxParseError(const QString&)),
                                 this, SLOT(ubloxProcessor_ubxParseError(const QString&)));

                QObject::connect(&ubloxProcessor, SIGNAL(nmeaParseError(const QString&)),
                                 this, SLOT(ubloxProcessor_nmeaParseError(const QString&)));

                QObject::connect(&ubloxProcessor, SIGNAL(unidentifiedDataReceived(const QByteArray&)),
                                 this, SLOT(ubloxProcessor_unidentifiedDataReceived(const QByteArray&)));

                for (currentRELPOSNEDReadingData.currentFileByteIndex = 0;
                     currentRELPOSNEDReadingData.currentFileByteIndex < fileLength;
                     currentRELPOSNEDReadingData.currentFileByteIndex++)
                {
                    // Handle data byte-by-byte using UBloxDataStreamProcessor.
                    // It will send signals when necessary
                    ubloxProcessor.process(static_cast<char>(fileData[static_cast<std::size_t>(currentRELPOSNEDReadingData.currentFileByteIndex)]), 0);
                }

                if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
                {
                    addLogLine("Warning: Duplicate iTOWS found at the end of file. Number of messages: " + QString::number(currentRELPOSNEDReadingData.duplicateITOWCounter) +
                               ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOW) + "..." +
                               QString::number(currentRELPOSNEDReadingData.lastReadITOW) +
                               ". Bytes " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex) +
                               "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex) +
                               ". Only previous messages preserved.");

                    currentRELPOSNEDReadingData.firstDuplicateITOW = -1;
                    currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = -1;
                    currentRELPOSNEDReadingData.duplicateITOWCounter = 0;
                }

                unsigned int numOfUnprocessedBytes = ubloxProcessor.getNumOfUnprocessedBytes();

                if (numOfUnprocessedBytes != 0)
                {
                    addLogLine("Warning: Unprocessed bytes at the end of the file: " + QString::number(numOfUnprocessedBytes));
                }

                addLogLine("File \"" + fileInfo.fileName() + "\" processed. Message counts: " +
                           "RELPOSNED: " + QString::number(currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_Total) +
                           " (" + QString::number(currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_UniqueITOWs) + " unique iTOWS)" +
                           ", UBX: " + QString::number(currentRELPOSNEDReadingData.messageCount_UBX) +
                           ", NMEA: " + QString::number(currentRELPOSNEDReadingData.messageCount_NMEA) +
                           ", RTCM: " + QString::number(currentRELPOSNEDReadingData.messageCount_RTCM) +
                           ". Discarded bytes: " + QString::number(currentRELPOSNEDReadingData.discardedBytesCount) +
                           " (" + QString::number(currentRELPOSNEDReadingData.discardedBytesCount * 100. / fileLength) + "%).");
            }

            ubxFile.close();
        }
        else
        {
            addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
        }
    }
    addLogLine("Files read.");
}

void PostProcessingForm::RELPOSNEDReadingData::init()
{
    messageCount_UBX = 0;
    messageCount_NMEA = 0;
    messageCount_RTCM = 0;
    messageCount_UBX_RELPOSNED_Total = 0;
    messageCount_UBX_RELPOSNED_UniqueITOWs = 0;

    lastReadITOW = -1;
    firstDuplicateITOW = -1;
    firstDuplicateITOWByteIndex = -1;
    duplicateITOWCounter = 0;

    currentFileByteIndex = 0;
    lastHandledDataByteIndex = 0;
    discardedBytesCount = 0;
}

void PostProcessingForm::ubloxProcessor_nmeaSentenceReceived(const QByteArray& nmeaSentence)
{
    // NMEA-messages are not utilized, but count them anyway
    Q_UNUSED(nmeaSentence);
    currentRELPOSNEDReadingData.messageCount_NMEA++;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_ubxMessageReceived(const UBXMessage& ubxMessage)
{
    currentRELPOSNEDReadingData.messageCount_UBX++;

    unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        // Casting of UBX-message to RELPOSNED was successful

        if ((ui->checkBox_ReportUnalignedITOWS->isChecked()) &&
                ((currentRELPOSNEDReadingData.lastReadITOW != -1) && ((relposned.iTOW % expectedITOWAlignment) != 0)))
        {
            addLogLine("Warning: iTOW not aligned to expected interval (" +
                       QString::number(expectedITOWAlignment) +" ms). iTOW: " + QString::number(relposned.iTOW) +
                       ". Bytes " + QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1) +
                       "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex));
        }
        if ((ui->checkBox_ReportMissingITOWs->isChecked()) &&
                ((currentRELPOSNEDReadingData.lastReadITOW != -1) &&
                 (static_cast<unsigned int>(relposned.iTOW - currentRELPOSNEDReadingData.lastReadITOW) > expectedITOWAlignment)))
        {
            int missingITOWS = (relposned.iTOW - currentRELPOSNEDReadingData.lastReadITOW - 1) / expectedITOWAlignment;

            addLogLine("Warning: iTOWs not consecutive with expected interval (" +
                       QString::number(expectedITOWAlignment) +" ms). Number of missing iTOWs: " + QString::number(missingITOWS) +
                       ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.lastReadITOW + 1) + "..." +
                       QString::number(relposned.iTOW - 1) +
                       ". Bytes " + QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1) +
                       "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex));
        }

        currentRELPOSNEDReadingData.lastReadITOW = relposned.iTOW;
        currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_Total++;

        if (currentRELPOSNEDReadingData.relposnedMessages->find(relposned.iTOW) != currentRELPOSNEDReadingData.relposnedMessages->end())
        {
            // RELPOSNED-message with the same iTOW already existed
            if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
            {
                // This was not the first already existing RELPOSNED-message with duplicate iTOW -> Increase counter
                currentRELPOSNEDReadingData.duplicateITOWCounter++;
            }
            else
            {
                // This is the first RELPOSNED-message with duplicate iTOW -> Store starting values
                currentRELPOSNEDReadingData.firstDuplicateITOW = relposned.iTOW;
                currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1;
                currentRELPOSNEDReadingData.duplicateITOWCounter = 1;
            }
        }
        else
        {
            if (currentRELPOSNEDReadingData.firstDuplicateITOW != -1)
            {
                // Duplicate iTOW(s) were found before this message
                addLogLine("Warning: Duplicate iTOWS found. Number of messages: " + QString::number(currentRELPOSNEDReadingData.duplicateITOWCounter) +
                           ". iTOW range: " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOW) + "..." +
                           QString::number(relposned.iTOW - 1) +
                           ". Bytes " + QString::number(currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex) +
                           "..." + QString::number(currentRELPOSNEDReadingData.currentFileByteIndex) +
                           ". Only previous messages preserved.");

                currentRELPOSNEDReadingData.firstDuplicateITOW = -1;
                currentRELPOSNEDReadingData.firstDuplicateITOWByteIndex = -1;
                currentRELPOSNEDReadingData.duplicateITOWCounter = 0;
            }

            currentRELPOSNEDReadingData.messageCount_UBX_RELPOSNED_UniqueITOWs++;
            if (currentRELPOSNEDReadingData.relposnedMessages)
            {
                currentRELPOSNEDReadingData.relposnedMessages->operator[](relposned.iTOW) = relposned;
            }
        }
    }

    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_rtcmMessageReceived(const RTCMMessage& rtcmMessage)
{
    // RTCM-messages are not utilized, but count them anyway
    Q_UNUSED(rtcmMessage);
    currentRELPOSNEDReadingData.messageCount_RTCM++;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_ubxParseError(const QString& errorString)
{
    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: UBX parse error: \"" + errorString + "\". " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_nmeaParseError(const QString& errorString)
{
    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: NMEA parse error: \"" + errorString + "\". " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}

void PostProcessingForm::ubloxProcessor_unidentifiedDataReceived(const QByteArray& data)
{
    Q_UNUSED(data);

    int discardedBytes = currentRELPOSNEDReadingData.currentFileByteIndex - currentRELPOSNEDReadingData.lastHandledDataByteIndex;

    addLogLine("Warning: Unidentified data. " +
               QString::number(discardedBytes) + " bytes discarded, beginning at byte " +
               QString::number(currentRELPOSNEDReadingData.lastHandledDataByteIndex + 1));

    currentRELPOSNEDReadingData.discardedBytesCount += discardedBytes;
    currentRELPOSNEDReadingData.lastHandledDataByteIndex = currentRELPOSNEDReadingData.currentFileByteIndex;
}



void PostProcessingForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Log->clear();
}


void PostProcessingForm::on_pushButton_AddTagData_clicked()
{
    if (fileDialog_Tags.exec())
    {
        QStringList fileNames = fileDialog_Tags.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Tags.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addLogLine("Reading tags...");

        for (const auto& fileName : fileNames)
        {
            QFileInfo fileInfo(fileName);
            addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

            QFile tagFile;
            tagFile.setFileName(fileName);
            if (tagFile.open(QIODevice::ReadOnly))
            {
                int numberOfTags = 0;

                QTextStream textStream(&tagFile);

                int64_t fileLength = tagFile.size();

                if (fileLength > 0x7FFFFFFFLL)
                {
                    addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                    tagFile.close();
                    continue;
                }

                QString headerLine = textStream.readLine();

                if (headerLine.compare("Time\tiTOW\tTag\tText", Qt::CaseInsensitive))
                {
                    addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Skipped.");
                    tagFile.close();
                    continue;
                }

                int lineNumber = 1;
                int discardedLines = 0;

                while (!textStream.atEnd())
                {
                    lineNumber++;

                    QString line = textStream.readLine();

                    QStringList subItems = line.split("\t");

                    if (subItems.count() < 3)
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                        continue;
                    }

                    bool iTOWConvOk;
                    int iTOW = subItems[1].toInt(&iTOWConvOk);

                    if (!iTOWConvOk)
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 2 (iTOW) to integer. Line skipped.");
                        continue;
                    }

                    if (subItems[2].length() == 0)
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Empty tag. Line skipped.");
                        continue;
                    }

                    Tag newTag;

                    newTag.sourceFile = fileName;
                    newTag.sourceFileLine = lineNumber;
//                    newTag.iTOW = iTOW;
                    newTag.ident = subItems[2];

                    if (subItems.count() > 3)
                    {
                        newTag.text = subItems[3];
                    }

                    if (tags.find(iTOW) != tags.end())
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Tag with duplicate iTOW. Line skipped.");
                        continue;
                    }

                    tags[iTOW] = newTag;

                    numberOfTags ++;
                }

                addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid tags: " +
                           QString::number(numberOfTags) +
                           ", total lines: " + QString::number(lineNumber) +
                           ", discarded lines: " + QString::number(discardedLines) + ".");
            }
            else
            {
                addLogLine("Error: Can not open file \"" + fileInfo.fileName() + "\". Skipped.");
            }
        }
        addLogLine("Files read.");
    }

}

void PostProcessingForm::on_pushButton_GeneratePointClouds_clicked()
{
    if (fileDialog_PointCloud.exec())
    {
        QDir dir = fileDialog_PointCloud.directory();

        if (!dir.exists())
        {
            addLogLine("Directory \"" + dir.path() + "\" doesn't exist. Point cloud files not created.");
            return;
        }

        addLogLine("Processing...");

        // Some locals to prevent excessive typing:
        QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA->value();
        bool includeNormals = ui->checkBox_IncludeNormals->checkState();

        QMap<UBXMessage_RELPOSNED::ITOW, Tag>::const_iterator currentTagIterator;

        bool objectActive = false;

        int beginningITOW = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QFile* outFile = nullptr;
        QTextStream* outStream = nullptr;

        for (currentTagIterator = tags.begin(); currentTagIterator != tags.end(); currentTagIterator++)
        {
            const Tag& currentTag = currentTagIterator.value();

            if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
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
                        addLogLine("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsWritten));
                        outFile->close();
                        delete outFile;
                        outFile = nullptr;
                    }
                    objectActive = false;
                }

                if (currentTag.text.length() == 0)
                {
                    // Empty name for the new object not allowed

                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": New object without a name. Ending previous object, but not beginning new nor creating a new file. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    continue;
                }

                QString fileName = QDir::cleanPath(dir.path() + "/" + currentTag.text + ".xyz");

                outFile = new QFile(fileName);

                if (outFile->exists())
                {
                    // File already exists -> Not allowed

                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    delete outFile;
                    outFile = nullptr;
                    continue;
                }

                addLogLine("Creating file \"" + fileName + "\"...");

                if (!outFile->open(QIODevice::WriteOnly | QIODevice::Text))
                {
                    // Creating the file failed

                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": File \"" + fileName + "\" can't be created. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    delete outFile;
                    outFile = nullptr;
                    continue;
                }

                outStream = new QTextStream(outFile);
                objectActive = true;
                ignoreBeginningAndEndingTags = false;
                beginningITOW = -1;
                pointsWritten = 0;
            }
            else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: Begin points

                if (!objectActive)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": Beginning tag outside object. Skipped.");
                    continue;
                }

                if (beginningITOW != -1)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": Duplicate beginning tag. Skipped.");
                    continue;
                }

                // Just store the beginning iTOW-value. Writing of the points is done in ending tag-branch
                beginningITOW = currentTagIterator.key();
            }
            else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
            {
                // Tag type: end points

                if (!objectActive)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": End tag outside object. Skipped.");
                    continue;
                }

                if (beginningITOW == -1)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": End tag without beginning tag. Skipped.");
                    continue;
                }

                const Tag& endingTag = currentTag;
                const Tag& beginningTag = tags[beginningITOW];

                if (endingTag.sourceFile != beginningTag.sourceFile)
                {
                    addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                               beginningTag.sourceFile + "\", line " +
                               QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                               endingTag.sourceFile + "\", line " +
                               QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                    continue;
                }

                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = relposnedMessages_RoverA.upperBound(beginningITOW);
                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = relposnedMessages_RoverB.upperBound(beginningITOW);

                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = relposnedMessages_RoverA.upperBound(currentTagIterator.key());
                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = relposnedMessages_RoverB.upperBound(currentTagIterator.key());

                int pointsBetweenTags = 0;

                while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                    (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                {
                    while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                           (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                           (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
                    {
                        // Skip all rover A RELPOSNEDs that have lower iTOW than the next rover B RELPOSNED (sync)
                        relposIterator_RoverA++;
                    }

                    while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                           (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                           (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
                    {
                        // Skip all rover B RELPOSNEDs that have lower iTOW than the next rover A RELPOSNED (sync)
                        relposIterator_RoverB++;
                    }

                    if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                        (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                    {
                        double n_RoverA = relposIterator_RoverA.value().relPosN;
                        double e_RoverA = relposIterator_RoverA.value().relPosE;
                        double d_RoverA = relposIterator_RoverA.value().relPosD;

                        double n_RoverB = relposIterator_RoverB.value().relPosN;
                        double e_RoverB = relposIterator_RoverB.value().relPosE;
                        double d_RoverB = relposIterator_RoverB.value().relPosD;

                        double diffN = n_RoverA - n_RoverB;
                        double diffE = e_RoverA - e_RoverB;
                        double diffD = d_RoverA - d_RoverB;

                        double vectorLength = sqrt(diffN * diffN + diffE * diffE + diffD * diffD);

                        // Unit vector pointing from B to A (= from point A towards tip) in NED coordinate system
                        double unitVecN = diffN / vectorLength;
                        double unitVecE = diffE / vectorLength;
                        double unitVecD = diffD / vectorLength;

                        double n_StylusTip = n_RoverA + unitVecN * stylusTipDistanceFromRoverA;
                        double e_StylusTip = e_RoverA + unitVecE * stylusTipDistanceFromRoverA;
                        double d_StylusTip = d_RoverA + unitVecD * stylusTipDistanceFromRoverA;

                        // Convert to left-handed XYZ-coordinate system for post-prosessing with Processing (yeah, really).
                        // (positive X = east, Y = down, Z = south)
                        // This will cause Y-axis to be mirrored in MeshLab since it seems to use right-handed-coordinate system
                        // (also the default orientation of the point cloud is upside-down).
                        // Doesn't really matter now as mirroring can be done in MeshLab (Transform: Flip and / or swap axis) if needed.
                        // But do NOT mirror it there if you intend to use obj-file in Processing.
                        // You may need to change these to get other coordinate systems into use.

                        double x = e_StylusTip;
                        double y = d_StylusTip;
                        double z = -n_StylusTip;

                        double x_SurfaceNormal = -unitVecE;
                        double y_SurfaceNormal = -unitVecD;
                        double z_SurfaceNormal = unitVecN;

                        QString lineOut;

                        if (includeNormals)
                        {
                            lineOut = QString::number(x, 'f', 4) +
                                    "\t" + QString::number(y, 'f', 4) +
                                    "\t" + QString::number(z, 'f', 4) +
                                    "\t" + QString::number(x_SurfaceNormal, 'f', 4) +
                                    "\t" + QString::number(y_SurfaceNormal, 'f', 4) +
                                    "\t" + QString::number(z_SurfaceNormal, 'f', 4);
                        }
                        else
                        {
                            lineOut = QString::number(x, 'f', 4) +
                                    "\t" + QString::number(y, 'f', 4) +
                                    "\t" + QString::number(z, 'f', 4);
                        }

                        outStream->operator<<(lineOut + "\n");

                        pointsWritten++;

                        pointsBetweenTags++;
                        relposIterator_RoverA++;
                        relposIterator_RoverB++;
                    }
                }

                if (pointsBetweenTags == 0)
                {
                    addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                               QString::number(beginningTag.sourceFileLine) +
                               ", iTOW " + QString::number(beginningITOW) + ", ending tag line " +
                               QString::number(endingTag.sourceFileLine) +
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ", File \"" + endingTag.sourceFile + "\""
                               " No points between tags.");
                }

                beginningITOW = -1;
            }
        }

        if (beginningITOW != -1)
        {
            const Tag& beginningTag = tags[beginningITOW];

            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
                       QString::number(beginningTag.sourceFileLine) +
                       ", iTOW " + QString::number(beginningITOW) +
                       " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
        }

        if (outStream)
        {
            delete outStream;
        }

        if (outFile)
        {
            addLogLine("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsWritten));
            outFile->close();
            delete outFile;
        }

        addLogLine("Point cloud files generated.");
    }
}

void PostProcessingForm::on_pushButton_StartReplay_clicked()
{
    lastReplayedITOW = ui->spinBox_Replay_ITOW_Min->value() -1;

    if (getLastRoverITOW() < 0)
    {
        addLogLine("No data to replay.\nData for both rovers empty or no valid data found\n(tags are synced to rovers' iTOWs).");
    }
    else
    {
        addLogLine("Replay started.");
        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(false);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(true);
        ui->spinBox_Replay_ITOW_Min->setEnabled(false);
        ui->spinBox_Replay_ITOW_Max->setEnabled(false);

        stopReplayRequest = false;

        handleReplay(true);
    }
}

void PostProcessingForm::on_replayTimerTimeout()
{
    handleReplay(false);
}


void PostProcessingForm::handleReplay(bool firstRound)
{
    UBXMessage_RELPOSNED::ITOW lastITOWInRoverFiles = getLastRoverITOW();

    if (stopReplayRequest)
    {
        addLogLine("Replay stopped. Last replayed ITOW: " + QString::number(lastReplayedITOW));
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(true);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->spinBox_Replay_ITOW_Min->setEnabled(true);
        ui->spinBox_Replay_ITOW_Max->setEnabled(true);

        stopReplayRequest = false;
    } else if ((lastReplayedITOW < lastITOWInRoverFiles) && (lastReplayedITOW <= ui->spinBox_Replay_ITOW_Max->value()))
    {
        UBXMessage_RELPOSNED::ITOW nextITOWInRoverFiles = getNextRoverITOW(lastReplayedITOW);

        bool relposnedReplayed_RoverA = false;
        bool relposnedReplayed_RoverB = false;

        if (relposnedMessages_RoverA.find(nextITOWInRoverFiles) != relposnedMessages_RoverA.end())
        {
            emit replayData_RoverA(relposnedMessages_RoverA[nextITOWInRoverFiles]);
            relposnedReplayed_RoverA = true;
        }

        if (relposnedMessages_RoverB.find(nextITOWInRoverFiles) != relposnedMessages_RoverB.end())
        {
            emit replayData_RoverB(relposnedMessages_RoverB[nextITOWInRoverFiles]);
            relposnedReplayed_RoverB = true;
        }

        // Skip tags not matching any rover iTOW

        QMap<UBXMessage_RELPOSNED::ITOW, Tag>::const_iterator orphanedTagIterator = tags.upperBound(lastReplayedITOW);

        while ((orphanedTagIterator != tags.end()) &&
               (orphanedTagIterator.key() < nextITOWInRoverFiles))
        {
            const UBXMessage_RELPOSNED::ITOW orphanedTagITOW = orphanedTagIterator.key();
            const Tag& orphanedTag = orphanedTagIterator.value();

            addLogLine("Warning: Tag in file \"" + orphanedTag.sourceFile + "\", line " +
                       QString::number(orphanedTag.sourceFileLine)+
                       ", iTOW " + QString::number(orphanedTagITOW) +
                       ": File \"" + orphanedTag.sourceFile + "\": No matching RELPOSNED-data found in rover files. Tag ignored.");

            orphanedTagIterator++;
        }

        if (tags.find(nextITOWInRoverFiles) != tags.end())
        {
            if (relposnedReplayed_RoverA && relposnedReplayed_RoverB)
            {
                emit replayData_Tag(nextITOWInRoverFiles, tags[nextITOWInRoverFiles]);
            }
            else
            {
                const UBXMessage_RELPOSNED::ITOW orphanedTagITOW = orphanedTagIterator.key();
                const Tag& orphanedTag = orphanedTagIterator.value();

                addLogLine("Warning: Tag in file \"" + orphanedTag.sourceFile + "\", line " +
                           QString::number(orphanedTag.sourceFileLine)+
                           ", iTOW " + QString::number(orphanedTagITOW) +
                           ": File \"" + orphanedTag.sourceFile + "\": No matching RELPOSNED-data found in both rover files. Tag ignored.");
            }
        }

        lastReplayedITOW = nextITOWInRoverFiles;

        // Go on to next ITOW
        nextITOWInRoverFiles = getNextRoverITOW(lastReplayedITOW);

        if (nextITOWInRoverFiles >= 0)
        {
            qint64 timerTotalError_ns = 0;

            if (firstRound)
            {
                cumulativeRequestedWaitTime_ns = 0;
                replayTimeElapsedTimer.start();
            }
            else
            {
                timerTotalError_ns = static_cast<qint64>(replayTimeElapsedTimer.nsecsElapsed()) - cumulativeRequestedWaitTime_ns;
            }

            int iTOWDifference = nextITOWInRoverFiles - lastReplayedITOW;

            qint64 expectedWaitTime_ns;

            if (ui->doubleSpinBox_ReplaySpeed->value() >= 1000)
            {
                // Max speed
                expectedWaitTime_ns = 0;
            }
            else
            {
                expectedWaitTime_ns = static_cast<qint64>((1000000. * iTOWDifference) / ui->doubleSpinBox_ReplaySpeed->value());
            }

            if ((timerTotalError_ns >= 1e9) && (ui->doubleSpinBox_ReplaySpeed->value() <= 1))
            {
                addLogLine("Replay timer total error exceeded 1s (computer was in sleep or otherwise laggy?), timer reset.");
                timerTotalError_ns = 0;
                cumulativeRequestedWaitTime_ns = 0;
                replayTimeElapsedTimer.restart();
            }
            else if (timerTotalError_ns >= 1e9)
            {
                // Limit maximum error to 1 s if computer can't keep up with the pace when replaying overspeed
                timerTotalError_ns = 1e9;
            }

            cumulativeRequestedWaitTime_ns += expectedWaitTime_ns;

            qint64 waitTime_ns = expectedWaitTime_ns - timerTotalError_ns;

            if (waitTime_ns < 0)
            {
                waitTime_ns = 0;
            }

            int waitTime_ms = static_cast<int>(waitTime_ns / 1000000);

            UBXMessage_RELPOSNED::ITOW replayRange_Min = getFirstRoverITOW();
            if (ui->spinBox_Replay_ITOW_Min->value() > replayRange_Min)
            {
                replayRange_Min = ui->spinBox_Replay_ITOW_Min->value();
            }

            UBXMessage_RELPOSNED::ITOW replayRange_Max = lastITOWInRoverFiles;
            if (ui->spinBox_Replay_ITOW_Max->value() < replayRange_Max)
            {
                replayRange_Max = ui->spinBox_Replay_ITOW_Max->value();
            }

            int progress = (lastReplayedITOW - replayRange_Min) * 100 / (replayRange_Max - replayRange_Min);

            ui->progress_ReplayProgress->setValue(progress);

            QTimer::singleShot(waitTime_ms, this, SLOT(on_replayTimerTimeout()));
        }
        else
        {
            // Replay is finished

            // List tags with iTOW larger than any of the rover iTOWs

            QMap<UBXMessage_RELPOSNED::ITOW, Tag>::const_iterator orphanedTagIterator = tags.upperBound(lastReplayedITOW);

            while (orphanedTagIterator != tags.end())
            {
                const UBXMessage_RELPOSNED::ITOW orphanedTagITOW = orphanedTagIterator.key();
                const Tag& orphanedTag = orphanedTagIterator.value();

                addLogLine("Warning: Tag in file \"" + orphanedTag.sourceFile + "\", line " +
                           QString::number(orphanedTag.sourceFileLine)+
                           ", iTOW " + QString::number(orphanedTagITOW) +
                           ": File \"" + orphanedTag.sourceFile + "\": iTOW larger than any rover's iTOW. Tag ignored.");

                orphanedTagIterator++;
            }

            addLogLine("Replay finished.");
            ui->progress_ReplayProgress->setValue(0);
            ui->pushButton_StartReplay->setEnabled(true);
            ui->pushButton_ContinueReplay->setEnabled(false);
            ui->pushButton_StopReplay->setEnabled(false);
            ui->spinBox_Replay_ITOW_Min->setEnabled(true);
            ui->spinBox_Replay_ITOW_Max->setEnabled(true);
        }
    }
    else
    {
        addLogLine("Replay finished unexpectedly. Did you clear some data during replay/pause?");
        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->spinBox_Replay_ITOW_Min->setEnabled(true);
        ui->spinBox_Replay_ITOW_Max->setEnabled(true);
    }

}

UBXMessage_RELPOSNED::ITOW PostProcessingForm::getFirstRoverITOW()
{
    UBXMessage_RELPOSNED::ITOW firstITOWInRoverFiles = -1;

    if (!relposnedMessages_RoverA.empty())
    {
        firstITOWInRoverFiles = relposnedMessages_RoverA.firstKey();
    }

    if ((!relposnedMessages_RoverB.empty()) && (relposnedMessages_RoverB.firstKey() < firstITOWInRoverFiles))
    {
        firstITOWInRoverFiles = relposnedMessages_RoverB.lastKey();
    }

    return firstITOWInRoverFiles;
}

UBXMessage_RELPOSNED::ITOW PostProcessingForm::getLastRoverITOW()
{
    UBXMessage_RELPOSNED::ITOW lastITOWInRoverFiles = -1;

    if ((!relposnedMessages_RoverA.empty()) && (relposnedMessages_RoverA.lastKey() > lastITOWInRoverFiles))
    {
        lastITOWInRoverFiles = relposnedMessages_RoverA.lastKey();
    }

    if ((!relposnedMessages_RoverB.empty()) && (relposnedMessages_RoverB.lastKey() > lastITOWInRoverFiles))
    {
        lastITOWInRoverFiles = relposnedMessages_RoverB.lastKey();
    }

    return lastITOWInRoverFiles;
}

UBXMessage_RELPOSNED::ITOW PostProcessingForm::getNextRoverITOW(const UBXMessage_RELPOSNED::ITOW& iTOW)
{
    UBXMessage_RELPOSNED::ITOW nextITOWInRoverFiles = -1;

    if ((!relposnedMessages_RoverA.empty()) && (relposnedMessages_RoverA.upperBound(iTOW) != relposnedMessages_RoverA.end()))
    {
        nextITOWInRoverFiles = relposnedMessages_RoverA.upperBound(iTOW).key();
    }

    if ((!relposnedMessages_RoverB.empty()) && (relposnedMessages_RoverB.upperBound(iTOW) != relposnedMessages_RoverB.end()) &&
            relposnedMessages_RoverB.upperBound(iTOW).key() < nextITOWInRoverFiles)
    {
        nextITOWInRoverFiles = relposnedMessages_RoverB.upperBound(iTOW).key();
    }

    return nextITOWInRoverFiles;
}



void PostProcessingForm::on_pushButton_StopReplay_clicked()
{
    stopReplayRequest = true;
}

void PostProcessingForm::on_pushButton_ContinueReplay_clicked()
{
    if (getLastRoverITOW() < 0)
    {
        addLogLine("No data to replay.\nData for both rovers empty or no valid data found\n(tags are synced to rovers' iTOWs).");
    }
    else
    {
        addLogLine("Replay continued.");
//        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(false);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(true);
        ui->spinBox_Replay_ITOW_Min->setEnabled(false);
        ui->spinBox_Replay_ITOW_Max->setEnabled(false);

        stopReplayRequest = false;
        handleReplay(true);
    }
}

void PostProcessingForm::on_pushButton_Movie_GenerateScript_clicked()
{
    if (fileDialog_MovieScript.exec())
    {
        QStringList fileNameList = fileDialog_MovieScript.selectedFiles();

        if (fileNameList.length() != 1)
        {
            addLogLine("Movie script: Multiple file selection not supported. Script not created.");
            return;
        }

        QFile movieScriptFile;

        movieScriptFile.setFileName(fileNameList[0]);

        if (movieScriptFile.exists())
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
                addLogLine("Movie script not created.");
                return;
            }
        }

        if (!movieScriptFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Can't open movie script file.");
            return;
        }

        QTextStream textStream(&movieScriptFile);

        textStream << "// Lines\tiTOW\tX\tY\tZ\taccX\tAccY\tAccZ\tObject\n";

        addLogLine("Processing line sets...");

        // Some locals to prevent excessive typing:
        QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Min = ui->spinBox_Movie_ITOW_Points_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Max = ui->spinBox_Movie_ITOW_Points_Max->value();
        unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

        QMap<UBXMessage_RELPOSNED::ITOW, Tag>::const_iterator currentTagIterator;

        bool objectActive = false;

        int beginningITOW = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QString objectName = "N/A";

        for (currentTagIterator = tags.begin(); currentTagIterator != tags.end(); currentTagIterator++)
        {
            const Tag& currentTag = currentTagIterator.value();

            if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
            {
                objectActive = false;

                if (currentTag.text.length() == 0)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": New object without a name. Ending previous object, but not beginning new nor creating a new line. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;
                    objectName = "N/A";

                    continue;
                }

#if 0
                // Take this into use (and modify accordingly) if you need to filter out multiple objects with the same name
                QString fileName = QDir::cleanPath(dir.path() + "/" + currentTag.Text + ".xyz");

                outFile = new QFile(fileName);

                if (outFile->exists())
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": File \"" + fileName + "\" already exists. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                    ignoreBeginningAndEndingTags = true;

                    delete outFile;
                    outFile = nullptr;
                    continue;
                }
                addLogLine("Creating file \"" + fileName + "\"...");
#endif

                addLogLine("Object \"" + currentTag.text + "\"...");

                objectActive = true;
                objectName = currentTag.text;
                ignoreBeginningAndEndingTags = false;
                beginningITOW = -1;
                pointsWritten = 0;
            }
            else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
            {
                if (!objectActive)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": Beginning tag outside object. Skipped.");
                    continue;
                }

                if (beginningITOW != -1)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": Duplicate beginning tag. Skipped.");
                    continue;
                }

                beginningITOW = currentTagIterator.key();
            }
            else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
            {
                if (!objectActive)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": End tag outside object. Skipped.");
                    continue;
                }

                if (beginningITOW == -1)
                {
                    addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                               QString::number(currentTag.sourceFileLine)+
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ": End tag without beginning tag. Skipped.");
                    continue;
                }

                const Tag& endingTag = currentTag;
                const Tag& beginningTag = tags[beginningITOW];

                if (endingTag.sourceFile != beginningTag.sourceFile)
                {
                    addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                               beginningTag.sourceFile + "\", line " +
                               QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                               endingTag.sourceFile + "\", line " +
                               QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                    continue;
                }

                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = relposnedMessages_RoverA.upperBound(beginningITOW);
                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = relposnedMessages_RoverB.upperBound(beginningITOW);

                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = relposnedMessages_RoverA.upperBound(currentTagIterator.key());
                QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = relposnedMessages_RoverB.upperBound(currentTagIterator.key());

                int pointsBetweenTags = 0;

                while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                    (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                {
                    while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                           (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                           (relposIterator_RoverA.key() < relposIterator_RoverB.key()))
                    {
                        relposIterator_RoverA++;
                    }

                    while ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                           (relposIterator_RoverB != relposIterator_RoverB_EndTag) &&
                           (relposIterator_RoverB.key() < relposIterator_RoverA.key()))
                    {
                        relposIterator_RoverB++;
                    }

                    if ((relposIterator_RoverA != relposIterator_RoverA_EndTag) &&
                        (relposIterator_RoverB != relposIterator_RoverB_EndTag))
                    {
                        if ((relposIterator_RoverA.key() >= iTOWRange_Lines_Min) &&
                        (relposIterator_RoverA.key() <= iTOWRange_Lines_Max))
                        {
                            double n_RoverA = relposIterator_RoverA.value().relPosN;
                            double e_RoverA = relposIterator_RoverA.value().relPosE;
                            double d_RoverA = relposIterator_RoverA.value().relPosD;

                            double n_RoverB = relposIterator_RoverB.value().relPosN;
                            double e_RoverB = relposIterator_RoverB.value().relPosE;
                            double d_RoverB = relposIterator_RoverB.value().relPosD;

                            double diffN = n_RoverA - n_RoverB;
                            double diffE = e_RoverA - e_RoverB;
                            double diffD = d_RoverA - d_RoverB;

                            double vectorLength = sqrt(diffN * diffN + diffE * diffE + diffD * diffD);

                            // Unit vector pointing from B to A (= from point A towards tip) in NED coordinate system
                            double unitVecN = diffN / vectorLength;
                            double unitVecE = diffE / vectorLength;
                            double unitVecD = diffD / vectorLength;

                            double n_StylusTip = n_RoverA + unitVecN * stylusTipDistanceFromRoverA;
                            double e_StylusTip = e_RoverA + unitVecE * stylusTipDistanceFromRoverA;
                            double d_StylusTip = d_RoverA + unitVecD * stylusTipDistanceFromRoverA;

                            // Convert to left-handed XYZ-coordinate system for post-prosessing with Processing (yeah, really).
                            // (positive X = east, Y = down, Z = south)
                            // You may need to change these to get other coordinate systems into use.

                            double x = e_StylusTip;
                            double y = d_StylusTip;
                            double z = -n_StylusTip;

                            // Use accuracies of rover A (used for stylus tip accuracy)
                            // Could calculate some kind of "worst case" scenario using both rovers,
                            // but probably errors are mostly common to both of them.
                            double accX = relposIterator_RoverA.value().accE;
                            double accY = relposIterator_RoverA.value().accD;
                            double accZ = relposIterator_RoverA.value().accN;

    /*                        double x_SurfaceNormal = -unitVecE;
                            double y_SurfaceNormal = -unitVecD;
                            double z_SurfaceNormal = unitVecN;
    */
                            QString lineOut;

                            if (pointsBetweenTags == 0)
                            {
                                lineOut = "LStart";

                            }
                            else
                            {
                                lineOut = "LCont";
                            }

                            lineOut +=
                                    "\t" + QString::number(relposIterator_RoverA.key()) +
                                    "\t" + QString::number(x, 'f', 4) +
                                    "\t" + QString::number(y, 'f', 4) +
                                    "\t" + QString::number(z, 'f', 4) +
                                    "\t" + QString::number(accX, 'f', 4) +
                                    "\t" + QString::number(accY, 'f', 4) +
                                    "\t" + QString::number(accZ, 'f', 4) +
                                    "\t" + objectName;

                            textStream << (lineOut + "\n");

                            pointsWritten++;

                            pointsBetweenTags++;
                        }
                        relposIterator_RoverA++;
                        relposIterator_RoverB++;
                    }
                }

                if (pointsBetweenTags == 0)
                {
                    addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                               QString::number(beginningTag.sourceFileLine) +
                               ", iTOW " + QString::number(beginningITOW) + ", ending tag line " +
                               QString::number(endingTag.sourceFileLine) +
                               ", iTOW " + QString::number(currentTagIterator.key()) +
                               ", File \"" + endingTag.sourceFile + "\""
                               " No points between tags.");
                }

                beginningITOW = -1;
            }
        }

        if (beginningITOW != -1)
        {
            const Tag& beginningTag = tags[beginningITOW];

            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
                       QString::number(beginningTag.sourceFileLine) +
                       ", iTOW " + QString::number(beginningITOW) +
                       " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
        }

        addLogLine("Processing script...");
        textStream << "// Frame type\tiTOW"
                      "\tTip_X\tTip_Y\tTip_Z"
                      "\tRoverA_X\tRoverA_Y\tRoverA_Z"
                      "\tRoverB_X\tRoverB_Y\tRoverB_Z"
                      "\tTip_acc_X\tTip_Acc_Y\tTip_Acc_Z"
                      "\tRoverA_acc_X\tRoverA_Acc_Y\tRoverA_Acc_Z"
                      "\tRoverB_acc_X\tRoverB_Acc_Y\tRoverB_Acc_Z"
                      "\tCamera_X\tCamera_Y\tCamera_Z"
                      "\tLookAt_X\tLookAt_Y\tLookAt_X\n";

        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = ui->spinBox_Movie_ITOW_Script_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = ui->spinBox_Movie_ITOW_Script_Max->value();

        double fps = ui->doubleSpinBox_Movie_FPS->value();

        iTOWRange_Script_Min -= iTOWRange_Script_Min % expectedITOWAlignment; // Round to previous aligned ITOW

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = relposnedMessages_RoverA.lowerBound(iTOWRange_Script_Min);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = relposnedMessages_RoverB.lowerBound(iTOWRange_Script_Min);

        UBXMessage_RELPOSNED::ITOW startingITOW = (relposIterator_RoverA.value().iTOW > relposIterator_RoverB.value().iTOW) ?
                    relposIterator_RoverA.value().iTOW : relposIterator_RoverB.value().iTOW;

        startingITOW -= startingITOW % expectedITOWAlignment; // Should not be needed, but just to be sure...

        int frameCounter = 0;

        // Some variables for camera:
        double cameraXShift = ui->doubleSpinBox_Movie_CameraX->value();
        double cameraYShift = ui->doubleSpinBox_Movie_CameraY->value();
        double cameraZShift = ui->doubleSpinBox_Movie_CameraZ->value();

        UBXMessage_RELPOSNED::ITOW iTOW = (frameCounter * 1000) / fps + startingITOW;

        while ((iTOW <= iTOWRange_Script_Max) &&
               (relposnedMessages_RoverA.upperBound(iTOW) != relposnedMessages_RoverA.end()) &&
               (relposnedMessages_RoverB.upperBound(iTOW) != relposnedMessages_RoverB.end()))
        {
            QString frameType;

            UBXMessage_RELPOSNED relposned_RoverA;
            UBXMessage_RELPOSNED relposned_RoverB;

            if ((relposnedMessages_RoverA.find(iTOW) != relposnedMessages_RoverA.end()) &&
                    (relposnedMessages_RoverB.find(iTOW) != relposnedMessages_RoverB.end()))
            {
                frameType = "F_Key";
                relposned_RoverA = relposnedMessages_RoverA.find(iTOW).value();
                relposned_RoverB = relposnedMessages_RoverB.find(iTOW).value();
            }
            else
            {
                frameType = "F_Interp";

                UBXMessage_RELPOSNED interpAStart = (relposnedMessages_RoverA.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpAEnd = relposnedMessages_RoverA.upperBound(iTOW).value();

                UBXMessage_RELPOSNED interpBStart = (relposnedMessages_RoverB.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpBEnd = relposnedMessages_RoverB.upperBound(iTOW).value();

                relposned_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(interpAStart, interpAEnd, iTOW);
                relposned_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(interpBStart, interpBEnd, iTOW);
            }

            double x_RoverA = relposned_RoverA.relPosE;
            double y_RoverA = relposned_RoverA.relPosD;
            double z_RoverA = -relposned_RoverA.relPosN;

            double x_RoverB = relposned_RoverB.relPosE;
            double y_RoverB = relposned_RoverB.relPosD;
            double z_RoverB = -relposned_RoverB.relPosN;

            double diffX = x_RoverB - x_RoverA;
            double diffY = y_RoverB - y_RoverA;
            double diffZ = z_RoverB - z_RoverA;

            // Unit vector pointing from rover A to B (= from tip to point A)
            QVector3D unitVec_StylusZAxis(static_cast<float>(diffX), static_cast<float>(diffY), static_cast<float>((diffZ)));
            unitVec_StylusZAxis.normalize();

            double x_StylusTip = x_RoverA - static_cast<double>(unitVec_StylusZAxis.x()) * stylusTipDistanceFromRoverA;
            double y_StylusTip = y_RoverA - static_cast<double>(unitVec_StylusZAxis.y()) * stylusTipDistanceFromRoverA;
            double z_StylusTip = z_RoverA - static_cast<double>(unitVec_StylusZAxis.z()) * stylusTipDistanceFromRoverA;

            double accX_RoverA = relposned_RoverA.accE;
            double accY_RoverA = relposned_RoverA.accD;
            double accZ_RoverA = relposned_RoverA.accN;

            double accX_RoverB = relposned_RoverB.accE;
            double accY_RoverB = relposned_RoverB.accD;
            double accZ_RoverB = relposned_RoverB.accN;

            // Use accuracies of rover A (used for stylus tip accuracy)
            // Could calculate some kind of "worst case" scenario using both rovers,
            // but probably errors are mostly common to both of them.
            double AccX_Tip = accX_RoverA;
            double accY_Tip = accY_RoverA;
            double accZ_Tip = accZ_RoverA;

            QVector3D downVec(0,1,0);
            QVector3D unitVec_StylusXAxis = QVector3D::crossProduct(downVec, unitVec_StylusZAxis);
            unitVec_StylusXAxis.normalize();
            if (unitVec_StylusXAxis.isNull())
            {
                // Stylus is pointing exactly up or down
                unitVec_StylusXAxis.setX(1);
            }

            QVector3D unitVec_StylusYAxis = QVector3D::crossProduct(unitVec_StylusZAxis, unitVec_StylusXAxis);
            unitVec_StylusYAxis.normalize();

            double x_Camera = x_StylusTip;
            double y_Camera = y_StylusTip;
            double z_Camera = z_StylusTip;

            // X-shift
            x_Camera += cameraXShift * static_cast<double>(unitVec_StylusXAxis.x());
            y_Camera += cameraXShift * static_cast<double>(unitVec_StylusXAxis.y());
            z_Camera += cameraXShift * static_cast<double>(unitVec_StylusXAxis.z());

            // Y-shift
            x_Camera += cameraYShift * static_cast<double>(unitVec_StylusYAxis.x());
            y_Camera += cameraYShift * static_cast<double>(unitVec_StylusYAxis.y());
            z_Camera += cameraYShift * static_cast<double>(unitVec_StylusYAxis.z());

            // Z-shift
            x_Camera += cameraZShift * static_cast<double>(unitVec_StylusZAxis.x());
            y_Camera += cameraZShift * static_cast<double>(unitVec_StylusZAxis.y());
            z_Camera += cameraZShift * static_cast<double>(unitVec_StylusZAxis.z());

            double x_lookAt = x_StylusTip;
            double y_lookAt = y_StylusTip;
            double z_lookAt = z_StylusTip;

            QString lineOut =
                    frameType +
                    "\t" + QString::number(iTOW) +
                    "\t" + QString::number(x_StylusTip, 'f', 4) +
                    "\t" + QString::number(y_StylusTip, 'f', 4) +
                    "\t" + QString::number(z_StylusTip, 'f', 4) +
                    "\t" + QString::number(x_RoverA, 'f', 4) +
                    "\t" + QString::number(y_RoverA, 'f', 4) +
                    "\t" + QString::number(z_RoverA, 'f', 4) +
                    "\t" + QString::number(x_RoverB, 'f', 4) +
                    "\t" + QString::number(y_RoverB, 'f', 4) +
                    "\t" + QString::number(z_RoverB, 'f', 4) +
                    "\t" + QString::number(AccX_Tip, 'f', 4) +
                    "\t" + QString::number(accY_Tip, 'f', 4) +
                    "\t" + QString::number(accZ_Tip, 'f', 4) +
                    "\t" + QString::number(accX_RoverA, 'f', 4) +
                    "\t" + QString::number(accY_RoverA, 'f', 4) +
                    "\t" + QString::number(accZ_RoverA, 'f', 4) +
                    "\t" + QString::number(accX_RoverB, 'f', 4) +
                    "\t" + QString::number(accY_RoverB, 'f', 4) +
                    "\t" + QString::number(accZ_RoverB, 'f', 4) +
                    "\t" + QString::number(x_Camera, 'f', 4) +
                    "\t" + QString::number(y_Camera, 'f', 4) +
                    "\t" + QString::number(z_Camera, 'f', 4) +
                    "\t" + QString::number(x_lookAt, 'f', 4) +
                    "\t" + QString::number(y_lookAt, 'f', 4) +
                    "\t" + QString::number(z_lookAt, 'f', 4);


            textStream << (lineOut + "\n");

            frameCounter++;
            iTOW = (frameCounter * 1000) / fps + startingITOW;
        }
    }
    addLogLine("Movie script generated.");
}

#if 0
void PostProcessingForm::multiplyMatrix(const double inputMatrix[4][1], const double rotationMatrix[4][4], double (&outputMatrix)[4][1])
{
    for(int i = 0; i < 4; i++ )
    {
        for(int j = 0; j < 1; j++)
        {
            outputMatrix[i][j] = 0;

            for(int k = 0; k < 4; k++)
            {
                outputMatrix[i][j] += rotationMatrix[i][k] * inputMatrix[k][j];
            }
        }
    }
}

void PostProcessingForm::setUpRotationMatrix(double angle, double u, double v, double w, double (&rotationMatrix)[4][4])
{
    double L = (u*u + v * v + w * w);
//    angle = angle * M_PI / 180.0; //converting to radian value
    double u2 = u * u;
    double v2 = v * v;
    double w2 = w * w;

    rotationMatrix[0][0] = (u2 + (v2 + w2) * cos(angle)) / L;
    rotationMatrix[0][1] = (u * v * (1 - cos(angle)) - w * sqrt(L) * sin(angle)) / L;
    rotationMatrix[0][2] = (u * w * (1 - cos(angle)) + v * sqrt(L) * sin(angle)) / L;
    rotationMatrix[0][3] = 0.0;

    rotationMatrix[1][0] = (u * v * (1 - cos(angle)) + w * sqrt(L) * sin(angle)) / L;
    rotationMatrix[1][1] = (v2 + (u2 + w2) * cos(angle)) / L;
    rotationMatrix[1][2] = (v * w * (1 - cos(angle)) - u * sqrt(L) * sin(angle)) / L;
    rotationMatrix[1][3] = 0.0;

    rotationMatrix[2][0] = (u * w * (1 - cos(angle)) - v * sqrt(L) * sin(angle)) / L;
    rotationMatrix[2][1] = (v * w * (1 - cos(angle)) + u * sqrt(L) * sin(angle)) / L;
    rotationMatrix[2][2] = (w2 + (u2 + v2) * cos(angle)) / L;
    rotationMatrix[2][3] = 0.0;

    rotationMatrix[3][0] = 0.0;
    rotationMatrix[3][1] = 0.0;
    rotationMatrix[3][2] = 0.0;
    rotationMatrix[3][3] = 1.0;
}

#endif
