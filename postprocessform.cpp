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

struct
{
    QString name;
    double values[4][4];
} static const transformationPresets[] =
{
    { "XYZ = +N+E+D or NED -> +X+Y+Z (default \"no conversion\")",
        {
            {  1,  0,  0,  0 },
            {  0,  1,  0,  0 },
            {  0,  0,  1,  0 },
            {  0,  0,  0,  1 },
        }
    },
    { "XYZ = +E+D-N or NED -> -Z+X+Y (Processing's \"movie script\" default left-handed)",
        {
            {  0,  1,  0,  0 },
            {  0,  0,  1,  0 },
            { -1,  0,  0,  0 },
            {  0,  0,  0,  1 },
        }
    },
};


PostProcessingForm::PostProcessingForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PostProcessingForm)
{
    ui->setupUi(this);

    QSettings settings;

    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Fallback", "900").toDouble());
    ui->lineEdit_TagIndicatingBeginningOfNewObject->setText(settings.value("PostProcessing_TagIndicatingBeginningOfNewObject", "New object").toString());
    ui->lineEdit_TagIndicatingBeginningOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingBeginningOfObjectPoints", "LMB").toString());
    ui->lineEdit_TagIndicatingEndOfObjectPoints->setText(settings.value("PostProcessing_TagIndicatingEndOfObjectPoints", "RMB").toString());

    ui->spinBox_MaxLogLines->setValue(settings.value("PostProcessing_MaxLogLines", "1000").toInt());

    ui->checkBox_IncludeNormals->setChecked(settings.value("PostProcessing_IncludeNormals", false).toBool());

    ui->spinBox_ExpectedITOWAlignment->setValue(settings.value("PostProcessing_ExpectedITOWAlignment", "100").toInt());
    ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->setValue(settings.value("PostProcessing_StylusTipDistanceFromRoverA_Correction", "0").toDouble());
    ui->checkBox_ReportMissingITOWs->setChecked(settings.value("PostProcessing_ReportMissingITOWs", false).toBool());
    ui->checkBox_ReportUnalignedITOWS->setChecked(settings.value("PostProcessing_ReportUnalignedITOWS", false).toBool());

    ui->doubleSpinBox_Translation_N->setValue(settings.value("PostProcessing_Translation_N", "0").toDouble());
    ui->doubleSpinBox_Translation_E->setValue(settings.value("PostProcessing_Translation_E", "0").toDouble());
    ui->doubleSpinBox_Translation_D->setValue(settings.value("PostProcessing_Translation_D", "0").toDouble());

    ui->doubleSpinBox_Movie_Camera_N->setValue(settings.value("PostProcessing_Movie_Camera_N", "-1").toDouble());
    ui->doubleSpinBox_Movie_Camera_E->setValue(settings.value("PostProcessing_Movie_Camera_E", "0").toDouble());
    ui->doubleSpinBox_Movie_Camera_D->setValue(settings.value("PostProcessing_Movie_Camera_D", "-0.05").toDouble());

    ui->doubleSpinBox_Movie_LookAt_N->setValue(settings.value("PostProcessing_Movie_LookAt_N", "0").toDouble());
    ui->doubleSpinBox_Movie_LookAt_E->setValue(settings.value("PostProcessing_Movie_LookAt_E", "0").toDouble());
    ui->doubleSpinBox_Movie_LookAt_D->setValue(settings.value("PostProcessing_Movie_LookAt_D", "-0.05").toDouble());

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            QString settingKey = "PostProcessing_Transform_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            QString defaultValue = "0";

            if (row == column)
            {
                defaultValue = "1";
            }

            ui->tableWidget_TransformationMatrix->item(row, column)->setText(settings.value(settingKey, defaultValue).toString());
        }
    }

    ui->doubleSpinBox_Movie_FPS->setValue(settings.value("PostProcessing_FPS", "30").toDouble());
}

PostProcessingForm::~PostProcessingForm()
{
    QSettings settings;

    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Fallback", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfNewObject", ui->lineEdit_TagIndicatingBeginningOfNewObject->text());
    settings.setValue("PostProcessing_TagIndicatingBeginningOfObjectPoints", ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text());
    settings.setValue("PostProcessing_TagIndicatingEndOfObjectPoints", ui->lineEdit_TagIndicatingEndOfObjectPoints->text());

    settings.setValue("PostProcessing_MaxLogLines", ui->spinBox_MaxLogLines->value());

    settings.setValue("PostProcessing_IncludeNormals", ui->checkBox_IncludeNormals->isChecked());

    settings.setValue("PostProcessing_ExpectedITOWAlignment", ui->spinBox_ExpectedITOWAlignment->value());
    settings.setValue("PostProcessing_StylusTipDistanceFromRoverA_Correction", ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value());
    settings.setValue("PostProcessing_ReportMissingITOWs", ui->checkBox_ReportMissingITOWs->isChecked());
    settings.setValue("PostProcessing_ReportUnalignedITOWS", ui->checkBox_ReportUnalignedITOWS->isChecked());

    settings.setValue("PostProcessing_Translation_N", ui->doubleSpinBox_Translation_N->value());
    settings.setValue("PostProcessing_Translation_E", ui->doubleSpinBox_Translation_E->value());
    settings.setValue("PostProcessing_Translation_D", ui->doubleSpinBox_Translation_D->value());

    settings.setValue("PostProcessing_Movie_Camera_N", ui->doubleSpinBox_Movie_Camera_N->value());
    settings.setValue("PostProcessing_Movie_Camera_E", ui->doubleSpinBox_Movie_Camera_E->value());
    settings.setValue("PostProcessing_Movie_Camera_D", ui->doubleSpinBox_Movie_Camera_D->value());

    settings.setValue("PostProcessing_Movie_LookAt_N", ui->doubleSpinBox_Movie_LookAt_N->value());
    settings.setValue("PostProcessing_Movie_LookAt_E", ui->doubleSpinBox_Movie_LookAt_E->value());
    settings.setValue("PostProcessing_Movie_LookAt_D", ui->doubleSpinBox_Movie_LookAt_D->value());

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            QString settingKey = "PostProcessing_Transform_Row" +
                    QString::number(row) + "_Column" +
                    QString::number(column);

            QString defaultValue = "0";

            if (row == column)
            {
                defaultValue = "1";
            }

            settings.setValue(settingKey, ui->tableWidget_TransformationMatrix->item(row, column)->text());
        }
    }

    settings.setValue("PostProcessing_FPS", ui->doubleSpinBox_Movie_FPS->value());

    delete ui;
}

void PostProcessingForm::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);

    if (!onShowInitializationsDone)
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

        fileDialog_Distances.setFileMode(QFileDialog::ExistingFiles);

        QStringList distanceFilters;

        distanceFilters << "Distance-files (*.distances)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Distances.setNameFilters(distanceFilters);

        fileDialog_Sync.setFileMode(QFileDialog::ExistingFiles);

        QStringList syncFilters;

        syncFilters << "Sync-files (*.sync)"
                << "Txt-files (*.txt)"
                << "Any files (*)";

        fileDialog_Sync.setNameFilters(syncFilters);

        fileDialog_All.setFileMode(QFileDialog::ExistingFiles);

        QStringList allFilesFilters;

        allFilesFilters << "UBX log files (*.ubx)"
                << "Raw log files (*.raw)"
                << "Tag-files (*.tags)"
                << "Txt-files (*.txt)"
                << "Sync-files (*.sync)"
                << "Any files (*)";

        fileDialog_All.setNameFilters(allFilesFilters);

        fileDialog_PointCloud.setFileMode(QFileDialog::Directory);

        fileDialog_MovieScript.setFileMode(QFileDialog::AnyFile);
        fileDialog_MovieScript.setDefaultSuffix("MovieScript");

        QStringList movieScriptFilters;

        movieScriptFilters << "moviescript files (*.moviescript)"
                << "Any files (*)";

        fileDialog_MovieScript.setNameFilters(movieScriptFilters);


        fileDialog_Transformation_Load.setFileMode(QFileDialog::ExistingFile);

        QStringList transformationFilters;

        transformationFilters << "Transformation files (*.Transformation)"
                << "Any files (*)";

        fileDialog_Transformation_Load.setNameFilters(transformationFilters);


        fileDialog_Transformation_Save.setFileMode(QFileDialog::AnyFile);
        fileDialog_Transformation_Save.setDefaultSuffix("Transformation");

        fileDialog_Transformation_Save.setNameFilters(transformationFilters);

        for (unsigned int presetIndex = 0; presetIndex < (sizeof(transformationPresets) / sizeof(transformationPresets[0])); presetIndex++)
        {
            ui->comboBox_Presets->addItem(transformationPresets[presetIndex].name);
        }

        onShowInitializationsDone = true;
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

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverA_clicked()
{
    rovers[0].relposnedMessages.clear();
    addLogLine("Rover A RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearRELPOSNEDData_RoverB_clicked()
{
    rovers[1].relposnedMessages.clear();
    addLogLine("Rover B RELPOSNED-data cleared.");
}

void PostProcessingForm::on_pushButton_ClearTagData_clicked()
{
    tags.clear();
    addLogLine("Tag data cleared.");
}

void PostProcessingForm::addRELPOSNEDData_Rover(const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        if (fileDialog_UBX.exec())
        {
            QStringList fileNames = fileDialog_UBX.selectedFiles();

            if (fileNames.size() != 0)
            {
                fileDialog_UBX.setDirectory(QFileInfo(fileNames[0]).path());
            }

            addRELPOSNEDData_Rover(fileNames, roverId);
        }
    }
}

void PostProcessingForm::addRELPOSNEDData_Rover(const QStringList fileNames, const unsigned int roverId)
{
    if (roverId < sizeof(rovers) / sizeof(rovers[0]))
    {
        addLogLine("Reading files into rover " + getRoverIdentString(roverId) + " relposned-data...");

        currentRELPOSNEDReadingData.relposnedMessages =  &rovers[roverId].relposnedMessages;

        addRELPOSNEDFileData(fileNames);

        currentRELPOSNEDReadingData.relposnedMessages = nullptr;
    }
}

void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverA_clicked()
{
    addRELPOSNEDData_Rover(0);
}


void PostProcessingForm::on_pushButton_AddRELPOSNEDData_RoverB_clicked()
{
    addRELPOSNEDData_Rover(1);
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

                QObject::connect(&ubloxProcessor, SIGNAL(nmeaSentenceReceived(const NMEAMessage&)),
                                 this, SLOT(ubloxProcessor_nmeaSentenceReceived(const NMEAMessage&)));

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

void PostProcessingForm::ubloxProcessor_nmeaSentenceReceived(const NMEAMessage& nmeaSentence)
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

void PostProcessingForm::addTagData(const QStringList& fileNames)
{
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

            bool uptimeColumnExists;

            if (!headerLine.compare("Time\tiTOW\tTag\tText", Qt::CaseInsensitive))
            {
                addLogLine("Warning: File's \"" + fileInfo.fileName() + "\" doesn't have \"Uptime\"-column (old format). Using iTOWS as uptimes. Distances and sync-data may not be valid.");
                uptimeColumnExists = false;
            }
            else if (!headerLine.compare("Time\tiTOW\tTag\tText\tUptime", Qt::CaseInsensitive))
            {
                uptimeColumnExists = true;
            }
            else
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have supported header. Skipped.");
                tagFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;
            int firstDuplicateTagLine = 0;
            int lastDuplicateTagLine = 0;

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if ((subItems.count() < 4) ||
                        ((subItems.count() < 5) && uptimeColumnExists))
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                Tag newTag;

                bool iTOWConvOk;
                newTag.iTOW = subItems[1].toInt(&iTOWConvOk);

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

                bool uptimeConvOk;
                qint64 uptime;

                if (uptimeColumnExists)
                {
                    uptime = subItems[4].toLongLong(&uptimeConvOk);

                    if (!uptimeConvOk)
                    {
                        discardedLines++;
                        addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (uptime) to 64-bit integer. Line skipped.");
                        continue;
                    }
                }
                else
                {
                    // Old format -> Use iTOW as uptime
                    uptime = newTag.iTOW;
                }

                newTag.sourceFile = fileName;
                newTag.sourceFileLine = lineNumber;
                newTag.ident = subItems[2];
                newTag.text = subItems[3];

                if (tags.find(uptime) != tags.end())
                {
                    QList<Tag> simultaneousItems = tags.values(uptime);

                    bool skip = false;

                    for (int i = 0; i < simultaneousItems.size(); i++)
                    {
                        if (simultaneousItems.at(i).ident == newTag.ident)
                        {
                            discardedLines++;
                            if (!firstDuplicateTagLine)
                            {
                                firstDuplicateTagLine = lineNumber;
                            }

                            lastDuplicateTagLine = lineNumber;
                            skip = true;
                            continue;
                        }
                    }

                    if (skip)
                    {
                        continue;
                    }
                }

                if (firstDuplicateTagLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateTagLine) + "-" +
                               QString::number(lastDuplicateTagLine) +
                               ": Duplicate tag(s). Line(s) skipped.");

                    firstDuplicateTagLine = 0;
                }

                tags.insertMulti(uptime, newTag);

                numberOfTags ++;
            }

            if (firstDuplicateTagLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateTagLine) + "-" +
                           QString::number(lastDuplicateTagLine) +
                           ": Duplicate tag(s). Line(s) skipped.");

                firstDuplicateTagLine = 0;
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


void PostProcessingForm::on_pushButton_AddTagData_clicked()
{
    if (fileDialog_Tags.exec())
    {
        QStringList fileNames = fileDialog_Tags.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Tags.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addTagData(fileNames);
    }

}

void PostProcessingForm::on_pushButton_GeneratePointClouds_clicked()
{
    Eigen::Matrix4d transformMatrix;

    if (!generateTransformationMatrix(transformMatrix))
    {
        return;
    }

    Eigen::Transform<double, 3, Eigen::Affine> transform;
    transform = transformMatrix;

    if (fileDialog_PointCloud.exec())
    {
        QDir dir = fileDialog_PointCloud.directory();

        if (!dir.exists())
        {
            addLogLine("Error: Directory \"" + dir.path() + "\" doesn't exist. Point cloud files not created.");
            return;
        }

        addLogLine("Processing...");

        // Some locals to prevent excessive typing:
        QString tagIdent_BeginNewObject = ui->lineEdit_TagIndicatingBeginningOfNewObject->text();
        QString tagIdent_BeginPoints = ui->lineEdit_TagIndicatingBeginningOfObjectPoints->text();
        QString tagIdent_EndPoints = ui->lineEdit_TagIndicatingEndOfObjectPoints->text();
        double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        bool includeNormals = ui->checkBox_IncludeNormals->checkState();

//        QMultiMap<qint64, Tag_New>::const_iterator currentTagIterator;

        bool objectActive = false;

        qint64 beginningUptime = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QFile* outFile = nullptr;
        QTextStream* outStream = nullptr;

        qint64 uptime = -1;
        Tag beginningTag;

        while (tags.upperBound(uptime) != tags.end())
        {
            uptime = tags.upperBound(uptime).key();

            QList<Tag> tagItems = tags.values(uptime);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const Tag& currentTag = tagItems[i];

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
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
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
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
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
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": File \"" + fileName + "\" can't be created. Ending previous object, but not beginning new. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;

                        delete outFile;
                        outFile = nullptr;
                        continue;
                    }

                    outStream = new QTextStream(outFile);
                    objectActive = true;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                    pointsWritten = 0;
                }
                else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: Begin points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
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
                else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    // Tag type: end points

                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag without beginning tag. Skipped.");
                        continue;
                    }

                    const Tag& endingTag = currentTag;

                    if (endingTag.sourceFile != beginningTag.sourceFile)
                    {
                        addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                                   beginningTag.sourceFile + "\", line " +
                                   QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                                   endingTag.sourceFile + "\", line " +
                                   QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                        continue;
                    }

                    bool constDistancesOnly = true;

                    QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(beginningUptime);

                    while ((distIter != distances.end()) && (distIter.key() < uptime))
                    {
                        if (distIter.value().type == DistanceItem::Type::MEASURED)
                        {
                            constDistancesOnly = false;
                            break;
                        }

                        distIter++;
                    }

                    int pointsBetweenTags = 0;

                    if (constDistancesOnly)
                    {
                        distIter = distances.upperBound(beginningUptime);

                        if (distIter != distances.end())
                        {
                            distIter--;

                            if ((distIter == distances.end()) ||
                                    (distIter.value().type != DistanceItem::Type::CONSTANT))
                            {
                                addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                           QString::number(currentTag.sourceFileLine)+
                                           ", uptime " + QString::number(uptime) +
                                           ", iTOW " + QString::number(currentTag.iTOW) +
                                           ": Points between tags having only constant distances without preceeding constant distance. Skipped.");
                                continue;
                            }

                            stylusTipDistanceFromRoverA = distIter.value().distance;
                        }

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.upperBound(beginningTag.iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.upperBound(beginningTag.iTOW);

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = rovers[0].relposnedMessages.upperBound(currentTag.iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = rovers[1].relposnedMessages.upperBound(currentTag.iTOW);

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
                                Eigen::Vector3d roverAPosNED(
                                        relposIterator_RoverA.value().relPosN,
                                        relposIterator_RoverA.value().relPosE,
                                        relposIterator_RoverA.value().relPosD);

                                Eigen::Vector3d roverBPosNED(
                                        relposIterator_RoverB.value().relPosN,
                                        relposIterator_RoverB.value().relPosE,
                                        relposIterator_RoverB.value().relPosD);

                                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                                // Convert to XYZ-coordinates
                                Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
                                Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
                                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;
                                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                                QString lineOut;

                                if (includeNormals)
                                {
                                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                                }
                                else
                                {
                                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                                }

                                outStream->operator<<(lineOut + "\n");

                                pointsWritten++;

                                pointsBetweenTags++;
                                relposIterator_RoverA++;
                                relposIterator_RoverB++;
                            }
                        }
                    }
                    else
                    {
                        // Distances found, sync point creation to them.
                        // Rover coordinates are interpolated according to distance timestamps.

                        distIter = distances.upperBound(beginningUptime);

                        while ((distIter != distances.end()) && (distIter.key() < uptime))
                        {
                            if (distIter.value().type == DistanceItem::Type::CONSTANT)
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Constant distance between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }
                            else if (distIter.value().type == DistanceItem::Type::MEASURED)
                            {
                                // Try to find next and previous rover coordinates
                                // for this uptime

                                qint64 distanceUptime = distIter.key();
                                // TODO: Add/subtract fine tune sync value here if needed

                                QMap<qint64, RoverSyncItem>::const_iterator roverAUptimeIter = rovers[0].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverA;

                                if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverAUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverAUptimeIter--;
                                    if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverAUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(upperSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[0].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[0].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                QMap<qint64, RoverSyncItem>::const_iterator roverBUptimeIter = rovers[1].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverB;

                                if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverBUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverBUptimeIter--;
                                    if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverBUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(upperSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[1].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[1].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                stylusTipDistanceFromRoverA = distIter.value().distance;

                                // TODO: Refine this quick hack or at least make the distance configurable!
                                // Skip distances that are too far away
                                // (measurement module seems to emit "outliers" now and then)
                                if (stylusTipDistanceFromRoverA > 2)
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Distance between RoverA and tip too high (" +
                                               QString::number(stylusTipDistanceFromRoverA) +
                                               " m). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                Eigen::Vector3d roverAPosNED(
                                        interpolated_RoverA.relPosN,
                                        interpolated_RoverA.relPosE,
                                        interpolated_RoverA.relPosD);

                                Eigen::Vector3d roverBPosNED(
                                        interpolated_RoverB.relPosN,
                                        interpolated_RoverB.relPosE,
                                        interpolated_RoverB.relPosD);

                                Eigen::Vector3d roverBToANED = roverAPosNED- roverBPosNED;
                                Eigen::Vector3d roverBToANEDNormalized = roverBToANED.normalized();
                                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToANEDNormalized * stylusTipDistanceFromRoverA;

                                // Convert to XYZ-coordinates
                                Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
                                Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
                                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;
                                Eigen::Vector3d roverBToAVecNormalizedXYZ = (roverAPosXYZ - roverBPosXYZ).normalized();

                                QString lineOut;

                                if (includeNormals)
                                {
                                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(0), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(1), 'f', 4) +
                                            "\t" + QString::number(-roverBToAVecNormalizedXYZ(2), 'f', 4);
                                }
                                else
                                {
                                    lineOut = QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4);
                                }

                                outStream->operator<<(lineOut + "\n");
                                pointsWritten++;
                                pointsBetweenTags++;
                            }
                            else
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Unknown distance type between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }

                            distIter++;
                        }
                    }

                    if (pointsBetweenTags == 0)
                    {
                        addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                                   QString::number(beginningTag.sourceFileLine) +
                                   ", uptime " + QString::number(beginningUptime) +
                                   ", iTOW " + QString::number(beginningTag.iTOW) + ", ending tag line " +
                                   QString::number(endingTag.sourceFileLine) +
                                   ", uptime " + QString::number(currentTag.iTOW) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ", File \"" + endingTag.sourceFile + "\""
                                   " No points between tags.");
                    }

                    beginningUptime = -1;
                }
            }
        }

        if (beginningUptime != -1)
        {
            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
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
            addLogLine("Closing file \"" + outFile->fileName() + "\". Points written: " + QString::number(pointsWritten));
            outFile->close();
            delete outFile;
        }

        addLogLine("Point cloud files generated.");
    }
}


void PostProcessingForm::on_pushButton_StartReplay_clicked()
{
    bool convOk;
    firstUptimeToReplay = ui->lineEdit_Uptime_Min->text().toLongLong(&convOk);
    lastReplayedUptime_ms = firstUptimeToReplay - 1;
    if (!convOk)
    {
        addLogLine("Invalid uptime range for replay, min.");
        ui->lineEdit_Uptime_Min->setFocus();
        return;
    }

    lastUptimeToReplay = ui->lineEdit_Uptime_Max->text().toLongLong(&convOk);
    if (!convOk)
    {
        addLogLine("Invalid uptime range for replay, max.");
        ui->lineEdit_Uptime_Max->setFocus();
        return;
    }

    if (firstUptimeToReplay >= lastUptimeToReplay)
    {
        addLogLine("Invalid uptime range for replay, min>max.");
        ui->lineEdit_Uptime_Min->setFocus();
        return;
    }

    if (getLastUptime() < 0)
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
        ui->lineEdit_Uptime_Min->setEnabled(false);
        ui->lineEdit_Uptime_Max->setEnabled(false);

        stopReplayRequest = false;

        // emit initial distance (/ fallback if no distances in file)
        DistanceItem initialDistanceItem;
        initialDistanceItem.distance = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        initialDistanceItem.type = DistanceItem::CONSTANT;
        emit replayData_Distance(1, initialDistanceItem);

        handleReplay(true);
    }
}

void PostProcessingForm::on_replayTimerTimeout()
{
    handleReplay(false);
}


void PostProcessingForm::handleReplay(bool firstRound)
{
    qint64 nextUptime_ms = getNextUptime(lastReplayedUptime_ms);

    if (stopReplayRequest)
    {
        addLogLine("Replay stopped. Last replayed uptime: " + QString::number(lastReplayedUptime_ms));
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(true);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->lineEdit_Uptime_Min->setEnabled(true);
        ui->lineEdit_Uptime_Max->setEnabled(true);

        stopReplayRequest = false;
    } else if ((nextUptime_ms >= 0) && (lastReplayedUptime_ms <= lastUptimeToReplay))
    {
//        qint64 nextUptime = getNextUptime(lastReplayedUptime);

        if ((!rovers[0].roverSyncData.empty()) && (rovers[0].roverSyncData.upperBound(lastReplayedUptime_ms) != rovers[0].roverSyncData.end()))
        {
            nextUptime_ms = rovers[0].roverSyncData.upperBound(lastReplayedUptime_ms).key();
        }

        if ((!rovers[1].roverSyncData.empty()) && (rovers[1].roverSyncData.upperBound(lastReplayedUptime_ms) != rovers[1].roverSyncData.end()) &&
                (rovers[1].roverSyncData.upperBound(lastReplayedUptime_ms).key() < nextUptime_ms))
        {
            nextUptime_ms = rovers[1].roverSyncData.upperBound(lastReplayedUptime_ms).key();
        }

        if ((!tags.empty()) && (tags.upperBound(lastReplayedUptime_ms) != tags.end()) &&
                (tags.upperBound(lastReplayedUptime_ms).key() < nextUptime_ms))
        {
            nextUptime_ms = tags.upperBound(lastReplayedUptime_ms).key();
        }

        if ((!distances.empty()) && (distances.upperBound(lastReplayedUptime_ms) != distances.end()) &&
                (distances.upperBound(lastReplayedUptime_ms).key() < nextUptime_ms))
        {
            nextUptime_ms = distances.upperBound(lastReplayedUptime_ms).key();
        }

        if (rovers[0].roverSyncData.find(nextUptime_ms) != rovers[0].roverSyncData.end())
        {
            RoverSyncItem syncItem = rovers[0].roverSyncData[nextUptime_ms];

            if (rovers[0].relposnedMessages.find(syncItem.iTOW) != rovers[0].relposnedMessages.end())
            {
                // Make local copy to add time stamp / frame duration.

                UBXMessage_RELPOSNED relposnedMessage = rovers[0].relposnedMessages[syncItem.iTOW];

                relposnedMessage.messageStartTime = nextUptime_ms;
                relposnedMessage.messageEndTime = nextUptime_ms + syncItem.frameTime;

                emit replayData_Rover(relposnedMessage, 0);
            }
            else
            {
                addLogLine("Warning: File \"" + syncItem.sourceFile + "\", line " +
                           QString::number(syncItem.sourceFileLine)+
                           ",  uptime " + QString::number(nextUptime_ms) +
                           ",  iTOW " + QString::number(syncItem.iTOW) +
                           ": No matching rover A-data found. Skipped.");
            }
        }

        if (rovers[1].roverSyncData.find(nextUptime_ms) != rovers[1].roverSyncData.end())
        {
            RoverSyncItem syncItem = rovers[1].roverSyncData[nextUptime_ms];

            if (rovers[1].relposnedMessages.find(syncItem.iTOW) != rovers[1].relposnedMessages.end())
            {
                // Make local copy to add time stamp / frame duration.

                UBXMessage_RELPOSNED relposnedMessage = rovers[1].relposnedMessages[syncItem.iTOW];

                relposnedMessage.messageStartTime = nextUptime_ms;
                relposnedMessage.messageEndTime = nextUptime_ms + syncItem.frameTime;

                emit replayData_Rover(relposnedMessage, 1);
            }
            else
            {
                addLogLine("Warning: File \"" + syncItem.sourceFile + "\", line " +
                           QString::number(syncItem.sourceFileLine)+
                           ",  uptime " + QString::number(nextUptime_ms) +
                           ",  iTOW " + QString::number(syncItem.iTOW) +
                           ": No matching rover B-data found. Skipped.");
            }
        }

        if (distances.find(nextUptime_ms) != distances.end())
        {
            emit replayData_Distance(nextUptime_ms, distances[nextUptime_ms]);
        }

        if (tags.find(nextUptime_ms) != tags.end())
        {
            QList<Tag> tagItems = tags.values(nextUptime_ms);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                emit replayData_Tag(nextUptime_ms, tagItems[i]);
            }
        }

        lastReplayedUptime_ms = nextUptime_ms;

        // Go on to next uptime

        nextUptime_ms = getNextUptime(nextUptime_ms);

        if (nextUptime_ms >= 0)
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

            qint64 uptimeDifference_ms = nextUptime_ms - lastReplayedUptime_ms;

            if (uptimeDifference_ms > (ui->doubleSpinBox_LimitInterval->value() * 1000))
            {
                addLogLine("Warning: Time between messages limited to max value (" +
                           QString::number(ui->doubleSpinBox_LimitInterval->value(), 'g', 3) + " s) between uptimes " +
                           QString::number(lastReplayedUptime_ms) + " and " +
                           QString::number(nextUptime_ms) + ".");
                uptimeDifference_ms = ui->doubleSpinBox_LimitInterval->value() * 1000;
            }

            qint64 expectedWaitTime_ns;

            if (ui->doubleSpinBox_ReplaySpeed->value() >= 1000)
            {
                // Max speed
                expectedWaitTime_ns = 0;
            }
            else
            {
                expectedWaitTime_ns = static_cast<qint64>((1000000. * uptimeDifference_ms) / ui->doubleSpinBox_ReplaySpeed->value());
            }

            if ((timerTotalError_ns >= 1e9) && (ui->doubleSpinBox_ReplaySpeed->value() <= 1))
            {
                addLogLine("Warning: Replay timer total error exceeded 1s (computer was in sleep or otherwise laggy?), timer reset.");
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

            qint64 replayRange_Min = getFirstUptime();
            if (firstUptimeToReplay > replayRange_Min)
            {
                replayRange_Min = firstUptimeToReplay;
            }

            qint64 replayRange_Max = getLastUptime();
            if (lastUptimeToReplay < replayRange_Max)
            {
                replayRange_Max = lastUptimeToReplay;
            }

            int progress = (lastReplayedUptime_ms - replayRange_Min) * 100 / (replayRange_Max - replayRange_Min);

            ui->progress_ReplayProgress->setValue(progress);

            QTimer::singleShot(waitTime_ms, this, SLOT(on_replayTimerTimeout()));
        }
        else
        {
            // Replay is finished

#if 0
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
#endif

            addLogLine("Replay finished.");
            ui->progress_ReplayProgress->setValue(0);
            ui->pushButton_StartReplay->setEnabled(true);
            ui->pushButton_ContinueReplay->setEnabled(false);
            ui->pushButton_StopReplay->setEnabled(false);
            ui->lineEdit_Uptime_Min->setEnabled(true);
            ui->lineEdit_Uptime_Max->setEnabled(true);
        }
    }
    else
    {
        addLogLine("Replay finished unexpectedly. Did you clear some data during replay/pause?");
        ui->progress_ReplayProgress->setValue(0);
        ui->pushButton_StartReplay->setEnabled(true);
        ui->pushButton_ContinueReplay->setEnabled(false);
        ui->pushButton_StopReplay->setEnabled(false);
        ui->lineEdit_Uptime_Min->setEnabled(true);
        ui->lineEdit_Uptime_Max->setEnabled(true);
    }

}

qint64 PostProcessingForm::getFirstUptime()
{
    qint64 firstUptime = std::numeric_limits<qint64>::max();

    if ((!rovers[0].roverSyncData.isEmpty()) && (rovers[0].roverSyncData.firstKey() < firstUptime))
    {
        firstUptime = rovers[0].roverSyncData.firstKey();
    }

    if ((!rovers[1].roverSyncData.isEmpty()) && (rovers[1].roverSyncData.firstKey() < firstUptime))
    {
        firstUptime = rovers[1].roverSyncData.firstKey();
    }

    if ((!distances.isEmpty()) && (distances.firstKey() < firstUptime))
    {
        firstUptime = distances.firstKey();
    }

    if ((!tags.isEmpty()) && (tags.firstKey() < firstUptime))
    {
        firstUptime = tags.firstKey();
    }

    if (firstUptime == std::numeric_limits<qint64>::max())
    {
        firstUptime = -1;
    }

    return firstUptime;
}

qint64 PostProcessingForm::getLastUptime()
{
    qint64 lastUptime = -1;

    if ((!rovers[0].roverSyncData.isEmpty()) && (rovers[0].roverSyncData.lastKey() > lastUptime))
    {
        lastUptime = rovers[0].roverSyncData.lastKey();
    }

    if ((!rovers[1].roverSyncData.isEmpty()) && (rovers[1].roverSyncData.lastKey() > lastUptime))
    {
        lastUptime = rovers[1].roverSyncData.lastKey();
    }

    if ((!distances.isEmpty()) && (distances.lastKey() > lastUptime))
    {
        lastUptime = distances.lastKey();
    }

    if ((!tags.isEmpty()) && (tags.lastKey() > lastUptime))
    {
        lastUptime = tags.lastKey();
    }

    return lastUptime;
}

qint64 PostProcessingForm::getNextUptime(const qint64 uptime)
{
    qint64 nextUptime = std::numeric_limits<qint64>::max();

    if ((!rovers[0].roverSyncData.empty()) && (rovers[0].roverSyncData.upperBound(uptime) != rovers[0].roverSyncData.end()) &&
            (rovers[0].roverSyncData.upperBound(uptime).key() < nextUptime))
    {
        nextUptime = rovers[0].roverSyncData.upperBound(uptime).key();
    }

    if ((!rovers[1].roverSyncData.empty()) && (rovers[1].roverSyncData.upperBound(uptime) != rovers[1].roverSyncData.end()) &&
            (rovers[1].roverSyncData.upperBound(uptime).key() < nextUptime))
    {
        nextUptime = rovers[1].roverSyncData.upperBound(uptime).key();
    }

    if ((!tags.empty()) && (tags.upperBound(uptime) != tags.end()) &&
            ((tags.upperBound(uptime).key() < nextUptime) || (nextUptime == -1)))
    {
        nextUptime = tags.upperBound(uptime).key();
    }

    if ((!distances.empty()) && (distances.upperBound(uptime) != distances.end()) &&
            ((distances.upperBound(uptime).key() < nextUptime) || (nextUptime == -1)))
    {
        nextUptime = distances.upperBound(uptime).key();
    }

    if (nextUptime == std::numeric_limits<qint64>::max())
    {
        return -1;
    }
    else
    {
        return nextUptime;
    }
}


void PostProcessingForm::on_pushButton_StopReplay_clicked()
{
    stopReplayRequest = true;
}

void PostProcessingForm::on_pushButton_ContinueReplay_clicked()
{
    if (getLastUptime() < 0)
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
        ui->lineEdit_Uptime_Min->setEnabled(false);
        ui->lineEdit_Uptime_Max->setEnabled(false);

        stopReplayRequest = false;
        handleReplay(true);
    }
}

void PostProcessingForm::on_pushButton_Movie_GenerateScript_clicked()
{
    Eigen::Matrix4d transformMatrix;

    if (!generateTransformationMatrix(transformMatrix))
    {
        return;
    }

    Eigen::Transform<double, 3, Eigen::Affine> transform;
    transform = transformMatrix;

    Eigen::Matrix3d transform_NoTranslation = transformMatrix.block<3,3>(0,0);

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
        double stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Min = ui->spinBox_Movie_ITOW_Points_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Max = ui->spinBox_Movie_ITOW_Points_Max->value();
        unsigned int expectedITOWAlignment = ui->spinBox_ExpectedITOWAlignment->value();

//        QMap<qint64, Tag_New>::const_iterator currentTagIterator;

        bool objectActive = false;

        qint64 beginningUptime = -1;
        int pointsWritten = 0;

        bool ignoreBeginningAndEndingTags = false;

        QString objectName = "N/A";

        qint64 uptime = -1;
        Tag beginningTag;

        while (tags.upperBound(uptime) != tags.end())
        {
            uptime = tags.upperBound(uptime).key();

            QList<Tag> tagItems = tags.values(uptime);

            // Since "The items that share the same key are available from most recently to least recently inserted."
            // (taken from QMultiMap's doc), iterate in "reverse order" here

            for (int i = tagItems.size() - 1; i >= 0; i--)
            {
                const Tag& currentTag = tagItems[i];

                if (!(currentTag.ident.compare(tagIdent_BeginNewObject)))
                {
                    objectActive = false;

                    if (currentTag.text.length() == 0)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": New object without a name. Ending previous object, but not beginning new nor creating a new line. Ignoring subsequent beginning and ending tags.");

                        ignoreBeginningAndEndingTags = true;
                        objectName = "N/A";

                        continue;
                    }

                    addLogLine("Object \"" + currentTag.text + "\"...");

                    objectActive = true;
                    objectName = currentTag.text;
                    ignoreBeginningAndEndingTags = false;
                    beginningUptime = -1;
                    pointsWritten = 0;
                }
                else if ((!(currentTag.ident.compare(tagIdent_BeginPoints))) && (!ignoreBeginningAndEndingTags))
                {
                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Beginning tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime != -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": Duplicate beginning tag. Skipped.");
                        continue;
                    }

                    beginningUptime = uptime;
                    beginningTag = currentTag;
                }
                else if ((!(currentTag.ident.compare(tagIdent_EndPoints)))  && (!ignoreBeginningAndEndingTags))
                {
                    if (!objectActive)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag outside object. Skipped.");
                        continue;
                    }

                    if (beginningUptime == -1)
                    {
                        addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                   QString::number(currentTag.sourceFileLine)+
                                   ", uptime " + QString::number(uptime) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ": End tag without beginning tag. Skipped.");
                        continue;
                    }

                    const Tag& endingTag = currentTag;
    //                const Tag_New beginningTag = tags_New[beginningUptime];

                    if (endingTag.sourceFile != beginningTag.sourceFile)
                    {
                        addLogLine("Warning: Starting and ending tags belong to different files. Starting tag file \"" +
                                   beginningTag.sourceFile + "\", line " +
                                   QString::number(beginningTag.sourceFileLine) + " ending tag file: " +
                                   endingTag.sourceFile + "\", line " +
                                   QString::number(endingTag.sourceFileLine) + ". Ending tag ignored.");
                        continue;
                    }

                    bool constDistancesOnly = true;

                    QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(beginningUptime);

                    while ((distIter != distances.end()) && (distIter.key() < uptime))
                    {
                        if (distIter.value().type == DistanceItem::Type::MEASURED)
                        {
                            constDistancesOnly = false;
                            break;
                        }

                        distIter++;
                    }

                    int pointsBetweenTags = 0;

                    if (constDistancesOnly)
                    {
                        distIter = distances.upperBound(beginningUptime);

                        if (distIter != distances.end())
                        {
                            distIter--;

                            if ((distIter == distances.end()) ||
                                    (distIter.value().type != DistanceItem::Type::CONSTANT))
                            {
                                addLogLine("Warning: File \"" + currentTag.sourceFile + "\", line " +
                                           QString::number(currentTag.sourceFileLine)+
                                           ", uptime " + QString::number(uptime) +
                                           ", iTOW " + QString::number(currentTag.iTOW) +
                                           ": Points between tags having only constant distances without preceeding constant distance. Skipped.");
                                continue;
                            }

                            stylusTipDistanceFromRoverA = distIter.value().distance;
                        }

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.upperBound(rovers[0].relposnedMessages.upperBound(beginningUptime).value().iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.upperBound(rovers[1].relposnedMessages.upperBound(beginningUptime).value().iTOW);

                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA_EndTag = rovers[0].relposnedMessages.upperBound(currentTag.iTOW);
                        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB_EndTag = rovers[1].relposnedMessages.upperBound(currentTag.iTOW);

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
                                    Eigen::Vector3d roverAPosNED(
                                            relposIterator_RoverA.value().relPosN,
                                            relposIterator_RoverA.value().relPosE,
                                            relposIterator_RoverA.value().relPosD);

                                    Eigen::Vector3d roverBPosNED(
                                            relposIterator_RoverB.value().relPosN,
                                            relposIterator_RoverB.value().relPosE,
                                            relposIterator_RoverB.value().relPosD);

                                    Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                                    Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                                    Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

                                    Eigen::Vector3d stylusTipAccNED(
                                                relposIterator_RoverA.value().accN,
                                                relposIterator_RoverA.value().accE,
                                                relposIterator_RoverA.value().accD);

                                    // Use accuracies of rover A (used for stylus tip accuracy)
                                    // Could calculate some kind of "worst case" scenario using both rovers,
                                    // but probably errors are mostly common to both of them.
                                    Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

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
                                            "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                            "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                            "\t" + objectName;

                                    textStream << (lineOut + "\n");

                                    pointsWritten++;

                                    pointsBetweenTags++;
                                }
                                relposIterator_RoverA++;
                                relposIterator_RoverB++;
                            }
                        }
                    } // if (constDistancesOnly)
                    else
                    {
                        // Distances found, sync point creation to them.
                        // Rover coordinates are interpolated according to distance timestamps.

                        distIter = distances.upperBound(beginningUptime);

                        while ((distIter != distances.end()) && (distIter.key() < uptime))
                        {
                            if (distIter.value().type == DistanceItem::Type::CONSTANT)
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Constant distance between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }
                            else if (distIter.value().type == DistanceItem::Type::MEASURED)
                            {
                                // Try to find next and previous rover coordinates
                                // for this uptime

                                qint64 distanceUptime = distIter.key();
                                // TODO: Add/subtract fine tune sync value here if needed

                                QMap<qint64, RoverSyncItem>::const_iterator roverAUptimeIter = rovers[0].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverA;

                                if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverAUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverAUptimeIter--;
                                    if (roverAUptimeIter != rovers[0].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverAUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(upperSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[0].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[0].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover A iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[0].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[0].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover A sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                QMap<qint64, RoverSyncItem>::const_iterator roverBUptimeIter = rovers[1].roverSyncData.lowerBound(distanceUptime);
                                UBXMessage_RELPOSNED interpolated_RoverB;

                                if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                {
                                    const RoverSyncItem upperSyncItem = roverBUptimeIter.value();
                                    RoverSyncItem lowerSyncItem;
                                    roverBUptimeIter--;
                                    if (roverBUptimeIter != rovers[1].roverSyncData.end())
                                    {
                                        lowerSyncItem = roverBUptimeIter.value();
                                    }
                                    else
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B sync data (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(upperSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    if (rovers[1].relposnedMessages.find(lowerSyncItem.iTOW) == rovers[1].relposnedMessages.end())
                                    {
                                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                                   QString::number(distIter.value().sourceFileLine)+
                                                   ", uptime " + QString::number(distIter.key()) +
                                                   ": Can not find corresponding rover B iTOW (higher limit). Skipped.");
                                        distIter++;
                                        continue;
                                    }

                                    qint64 timeDiff = distanceUptime - roverAUptimeIter.key();

                                    interpolated_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(rovers[1].relposnedMessages.find(lowerSyncItem.iTOW).value(),
                                                            rovers[1].relposnedMessages.find(upperSyncItem.iTOW).value(), lowerSyncItem.iTOW + timeDiff);
                                }
                                else
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Can not find corresponding rover B sync data (upper limit). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                stylusTipDistanceFromRoverA = distIter.value().distance;

                                // TODO: Refine this quick hack or at least make the distance configurable!
                                // Skip distances that are too far away
                                // (measurement module seems to emit "outliers" now and then)
                                if (stylusTipDistanceFromRoverA > 2)
                                {
                                    addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                               QString::number(distIter.value().sourceFileLine)+
                                               ", uptime " + QString::number(distIter.key()) +
                                               ": Distance between RoverA and tip too high (" +
                                               QString::number(stylusTipDistanceFromRoverA) +
                                               " m). Skipped.");
                                    distIter++;
                                    continue;
                                }

                                Eigen::Vector3d roverAPosNED(
                                        interpolated_RoverA.relPosN,
                                        interpolated_RoverA.relPosE,
                                        interpolated_RoverA.relPosD);

                                Eigen::Vector3d roverBPosNED(
                                        interpolated_RoverB.relPosN,
                                        interpolated_RoverB.relPosE,
                                        interpolated_RoverB.relPosD);

                                Eigen::Vector3d roverBToAVecNormalized = (roverAPosNED - roverBPosNED).normalized();

                                Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalized * stylusTipDistanceFromRoverA;
                                Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

                                Eigen::Vector3d stylusTipAccNED(
                                            interpolated_RoverA.accN,
                                            interpolated_RoverA.accE,
                                            interpolated_RoverA.accD);

                                // Use accuracies of rover A (used for stylus tip accuracy)
                                // Could calculate some kind of "worst case" scenario using both rovers,
                                // but probably errors are mostly common to both of them.
                                Eigen::Vector3d stylusTipAccXYZ = transform_NoTranslation * stylusTipAccNED;

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
                                        "\t" + QString::number(interpolated_RoverA.iTOW) +
                                        "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                                        "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                                        "\t" + objectName;

                                textStream << (lineOut + "\n");

                                pointsWritten++;
                                pointsBetweenTags++;
                            }
                            else
                            {
                                addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                           QString::number(distIter.value().sourceFileLine)+
                                           ", uptime " + QString::number(distIter.key()) +
                                           ": Unknown distance type between measured ones. Skipped.");
                                distIter++;
                                continue;
                            }

                            distIter++;
                        }
                    }


                    if (pointsBetweenTags == 0)
                    {
                        addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", beginning tag line " +
                                   QString::number(beginningTag.sourceFileLine) +
                                   ", iTOW " + QString::number(rovers[0].roverSyncData.upperBound(beginningUptime).value().iTOW) + ", ending tag line " +
                                   QString::number(endingTag.sourceFileLine) +
                                   ", iTOW " + QString::number(currentTag.iTOW) +
                                   ", File \"" + endingTag.sourceFile + "\""
                                   " No points between tags.");
                    }

                    beginningTag.iTOW = -1;
                    beginningUptime = -1;
                }
            }
        }

        if (beginningTag.iTOW != -1)
        {
            addLogLine("Warning: File \"" + beginningTag.sourceFile + "\", line " +
                       QString::number(beginningTag.sourceFileLine) +
                       ", iTOW " + QString::number(beginningTag.iTOW) +
                       " (beginning tag): File ended before end tag. Points after beginning tag ignored.");
        }

        stylusTipDistanceFromRoverA = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Fallback->value();

        addLogLine("Processing script...");
        textStream << "// Frame type\tiTOW"
                      "\tTip_X\tTip_Y\tTip_Z"
                      "\tRoverA_X\tRoverA_Y\tRoverA_Z"
                      "\tRoverB_X\tRoverB_Y\tRoverB_Z"
                      "\tTip_acc_X\tTip_Acc_Y\tTip_Acc_Z"
                      "\tRoverA_acc_X\tRoverA_Acc_Y\tRoverA_Acc_Z"
                      "\tRoverB_acc_X\tRoverB_Acc_Y\tRoverB_Acc_Z"
                      "\tCamera_X\tCamera_Y\tCamera_Z"
                      "\tLookAt_X\tLookAt_Y\tLookAt_X\tTipPositionValidity\n";

        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = ui->spinBox_Movie_ITOW_Script_Min->value();
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = ui->spinBox_Movie_ITOW_Script_Max->value();

        double fps = ui->doubleSpinBox_Movie_FPS->value();

        iTOWRange_Script_Min -= iTOWRange_Script_Min % expectedITOWAlignment; // Round to previous aligned ITOW

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverA = rovers[0].relposnedMessages.lowerBound(iTOWRange_Script_Min);
        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator_RoverB = rovers[1].relposnedMessages.lowerBound(iTOWRange_Script_Min);

        UBXMessage_RELPOSNED::ITOW startingITOW = (relposIterator_RoverA.value().iTOW > relposIterator_RoverB.value().iTOW) ?
                    relposIterator_RoverA.value().iTOW : relposIterator_RoverB.value().iTOW;

        startingITOW -= startingITOW % expectedITOWAlignment; // Should not be needed, but just to be sure...

        int frameCounter = 0;

        // Some variables for camera:
        double cameraNShift = ui->doubleSpinBox_Movie_Camera_N->value();
        double cameraEShift = ui->doubleSpinBox_Movie_Camera_E->value();
        double cameraDShift = ui->doubleSpinBox_Movie_Camera_D->value();

        double lookAtNShift = ui->doubleSpinBox_Movie_LookAt_N->value();
        double lookAtEShift = ui->doubleSpinBox_Movie_LookAt_E->value();
        double lookAtDShift = ui->doubleSpinBox_Movie_LookAt_D->value();

        UBXMessage_RELPOSNED::ITOW iTOW = (frameCounter * 1000) / fps + startingITOW;

        UBXMessage_RELPOSNED::ITOW lastRoverANagITOW = -1;
        UBXMessage_RELPOSNED::ITOW lastRoverBNagITOW = -1;

        while ((iTOW <= iTOWRange_Script_Max) &&
               (rovers[0].relposnedMessages.upperBound(iTOW) != rovers[0].relposnedMessages.end()) &&
               (rovers[1].relposnedMessages.upperBound(iTOW) != rovers[1].relposnedMessages.end()))
        {
            QString frameType;

            UBXMessage_RELPOSNED relposned_RoverA;
            UBXMessage_RELPOSNED relposned_RoverB;

            qint64 uptime = -1;

            if ((rovers[0].relposnedMessages.find(iTOW) != rovers[0].relposnedMessages.end()) &&
                    (rovers[1].relposnedMessages.find(iTOW) != rovers[1].relposnedMessages.end()))
            {
                frameType = "F_Key";
                relposned_RoverA = rovers[0].relposnedMessages.find(iTOW).value();
                relposned_RoverB = rovers[1].relposnedMessages.find(iTOW).value();

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA = rovers[0].reverseSync.find(iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB = rovers[1].reverseSync.find(iTOW);

                if ((reverseIter_RoverA == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = iTOW;
                }

                if ((reverseIter_RoverB == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = iTOW;
                }

                if ((reverseIter_RoverA != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverB != rovers[1].reverseSync.end()))
                {
                    uptime = (reverseIter_RoverA.value() + reverseIter_RoverB.value()) / 2;
                }
            }
            else
            {
                frameType = "F_Interp";

                UBXMessage_RELPOSNED interpAStart = (rovers[0].relposnedMessages.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpAEnd = rovers[0].relposnedMessages.upperBound(iTOW).value();

                UBXMessage_RELPOSNED interpBStart = (rovers[1].relposnedMessages.upperBound(iTOW) - 1).value();
                UBXMessage_RELPOSNED interpBEnd = rovers[1].relposnedMessages.upperBound(iTOW).value();

                relposned_RoverA = UBXMessage_RELPOSNED::interpolateCoordinates(interpAStart, interpAEnd, iTOW);
                relposned_RoverB = UBXMessage_RELPOSNED::interpolateCoordinates(interpBStart, interpBEnd, iTOW);

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_Start = rovers[0].reverseSync.find(interpAStart.iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverA_End = rovers[0].reverseSync.find(interpAEnd.iTOW);

                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_Start = rovers[1].reverseSync.find(interpBStart.iTOW);
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>::const_iterator reverseIter_RoverB_End = rovers[1].reverseSync.find(interpBEnd.iTOW);

                if ((reverseIter_RoverA_Start == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != interpAStart.iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(interpAStart.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = interpAStart.iTOW;
                }

                if ((reverseIter_RoverA_End == rovers[0].reverseSync.end()) &&
                        (lastRoverANagITOW != interpAEnd.iTOW))
                {
                    addLogLine("Warning: Uptime for rover A iTOW \"" + QString::number(interpAEnd.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverANagITOW = interpAEnd.iTOW;
                }

                if ((reverseIter_RoverB_Start == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != interpBStart.iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(interpBStart.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = interpBStart.iTOW;
                }

                if ((reverseIter_RoverB_End == rovers[1].reverseSync.end()) &&
                        (lastRoverBNagITOW != interpBEnd.iTOW))
                {
                    addLogLine("Warning: Uptime for rover B iTOW \"" + QString::number(interpBEnd.iTOW) +
                               " not found in sync data. Distance can not be synced.");

                    lastRoverBNagITOW = interpBEnd.iTOW;
                }

                if ((reverseIter_RoverA_Start != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverA_End != rovers[0].reverseSync.end()) &&
                        (reverseIter_RoverB_Start != rovers[1].reverseSync.end()) &&
                        (reverseIter_RoverB_End != rovers[1].reverseSync.end()))
                {
                    double startITOWAvg = (interpAStart.iTOW + interpBStart.iTOW) / 2;
                    double endITOWAvg = (interpAEnd.iTOW + interpBEnd.iTOW) / 2;

                    double fraction = static_cast<double>((iTOW - startITOWAvg)) /
                            (endITOWAvg - startITOWAvg);

                    uptime = (reverseIter_RoverA_Start.value() + reverseIter_RoverB_Start.value()) / 2 +
                            fraction *
                            (((reverseIter_RoverA_End.value() - reverseIter_RoverA_Start.value()) +
                             (reverseIter_RoverB_End.value() - reverseIter_RoverB_Start.value())) / 2);
                }
            }

            bool distanceValid = false;

            QMap<qint64, DistanceItem>::const_iterator distIter = distances.upperBound(uptime);

            if ((distIter != distances.cend()) && (uptime != -1))
            {
                QMap<qint64, DistanceItem>::const_iterator nextDistIter = distIter;

                if (distIter != distances.cbegin())
                {
                    distIter--;

                    QMap<qint64, DistanceItem>::const_iterator prevDistIter = distIter;

                    if (prevDistIter.value().type == DistanceItem::Type::CONSTANT)
                    {
                        distanceValid = true;
                        stylusTipDistanceFromRoverA = prevDistIter.value().distance;
                    }
                    else if ((prevDistIter.value().type == DistanceItem::Type::MEASURED) &&
                             (nextDistIter.value().type == DistanceItem::Type::MEASURED))
                    {
                        // Interpolate between measured distances

                        double fraction = static_cast<double>((uptime - prevDistIter.key())) /
                                (nextDistIter.key() - prevDistIter.key());

                        stylusTipDistanceFromRoverA = prevDistIter.value().distance +
                                fraction * (nextDistIter.value().distance - prevDistIter.value().distance);

                        if ((uptime - prevDistIter.key()) < 500)
                        {
                            distanceValid = true;
                        }
                    }
                    else if (prevDistIter.value().type == DistanceItem::Type::MEASURED)
                    {
                        // No more measured distances -> Use the last one

                        stylusTipDistanceFromRoverA = prevDistIter.value().distance;

                        if ((uptime - prevDistIter.key()) < 500)
                        {
                            distanceValid = true;
                        }
                    }
                    else
                    {
                        addLogLine("Warning: File \"" + distIter.value().sourceFile + "\", line " +
                                   QString::number(distIter.value().sourceFileLine)+
                                   ", uptime " + QString::number(distIter.key()) +
                                   ": Unknown distance type between measured ones. Skipped.");
                    }
                }
            }

            Eigen::Vector3d roverAPosNED(
                    relposned_RoverA.relPosN,
                    relposned_RoverA.relPosE,
                    relposned_RoverA.relPosD);

            Eigen::Vector3d roverBPosNED(
                    relposned_RoverB.relPosN,
                    relposned_RoverB.relPosE,
                    relposned_RoverB.relPosD);

            Eigen::Vector3d roverBToAVecNormalizedNED = (roverAPosNED - roverBPosNED).normalized();

            Eigen::Vector3d stylusTipPosNED = roverAPosNED + roverBToAVecNormalizedNED * stylusTipDistanceFromRoverA;

            Eigen::Vector3d roverAAccNED(
                        relposned_RoverA.accN,
                        relposned_RoverA.accE,
                        relposned_RoverA.accD);

            Eigen::Vector3d roverBAccNED(
                        relposned_RoverB.accN,
                        relposned_RoverB.accE,
                        relposned_RoverB.accD);

            Eigen::Vector3d roverAPosXYZ = transform * roverAPosNED;
            Eigen::Vector3d roverBPosXYZ = transform * roverBPosNED;
            Eigen::Vector3d stylusTipPosXYZ = transform * stylusTipPosNED;

            Eigen::Vector3d roverAAccXYZ = transform_NoTranslation * roverAAccNED;
            Eigen::Vector3d roverBAccXYZ = transform_NoTranslation * roverBAccNED;

            // Use accuracies of rover A (used for stylus tip accuracy)
            // Could calculate some kind of "worst case" scenario using both rovers,
            // but probably errors are mostly common to both of them.
            Eigen::Vector3d stylusTipAccXYZ = roverAAccXYZ;

            Eigen::Vector3d downVecNED(0,0,1);
            Eigen::Vector3d stylusForwardAxis = roverBToAVecNormalizedNED;
            Eigen::Vector3d stylusRightAxis = -(roverBToAVecNormalizedNED.cross(downVecNED).normalized());
            Eigen::Vector3d stylusDownAxis = roverBToAVecNormalizedNED.cross(stylusRightAxis).normalized();

            Eigen::Vector3d cameraPosNED = roverAPosNED +
                    stylusForwardAxis * cameraNShift +
                    stylusRightAxis * cameraEShift +
                    stylusDownAxis * cameraDShift;

            Eigen::Vector3d cameraPosXYZ = transform * cameraPosNED;

            Eigen::Vector3d lookAtPosNED = roverAPosNED +
                    stylusForwardAxis * lookAtNShift +
                    stylusRightAxis * lookAtEShift +
                    stylusDownAxis * lookAtDShift;

            Eigen::Vector3d lookAtPosXYZ = transform * lookAtPosNED;

            QString stylusTipPositionValidityString;

            if (distanceValid)
            {
                stylusTipPositionValidityString = "Valid";
            }
            else
            {
                stylusTipPositionValidityString = "Invalid";
            }

            QString lineOut =
                    frameType +
                    "\t" + QString::number(iTOW) +
                    "\t" + QString::number(stylusTipPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(stylusTipPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(stylusTipPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverAPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverBPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(stylusTipAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverAAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(0), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(1), 'f', 4) +
                    "\t" + QString::number(roverBAccXYZ(2), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(cameraPosXYZ(2), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(0), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(1), 'f', 4) +
                    "\t" + QString::number(lookAtPosXYZ(2), 'f', 4) +
                    "\t" + stylusTipPositionValidityString;

            textStream << (lineOut + "\n");

            frameCounter++;
            iTOW = (frameCounter * 1000) / fps + startingITOW;
        }
    }
    addLogLine("Movie script generated.");
}

void PostProcessingForm::on_pushButton_ClearDistanceData_clicked()
{
    distances.clear();
    addLogLine("Distance data cleared.");
}

void PostProcessingForm::addDistanceData(const QStringList& fileNames)
{
    double distanceCorrection = ui->doubleSpinBox_StylusTipDistanceFromRoverA_Correction->value();

    addLogLine("Reading distances...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile distanceFile;
        distanceFile.setFileName(fileName);
        if (distanceFile.open(QIODevice::ReadOnly))
        {
            int numberOfDistances = 0;

            QTextStream textStream(&distanceFile);

            int64_t fileLength = distanceFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                distanceFile.close();
                continue;
            }

            QString headerLine = textStream.readLine();

            if (headerLine.compare("Time\tDistance\tType\tUptime(Start)\tFrame time", Qt::CaseInsensitive))
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Skipped.");
                distanceFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;

            int firstDuplicateUptimeLine = 0;
            int lastDuplicateUptimeLine = 0;

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if (subItems.count() < 5)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                bool distanceConvOk;

                DistanceItem newDistanceItem;
                newDistanceItem.distance = subItems[1].toDouble(&distanceConvOk);

                if (!distanceConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 2 (distance) to double. Line skipped.");
                    continue;
                }

                if (!subItems[2].compare("constant", Qt::CaseInsensitive))
                {
                    newDistanceItem.type = DistanceItem::Type::CONSTANT;
                }
                else if (!subItems[2].compare("measured", Qt::CaseInsensitive))
                {
                    newDistanceItem.type = DistanceItem::Type::MEASURED;
                    newDistanceItem.distance += distanceCorrection;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Distance type not either \"constant\" nor \"measured\". Line skipped");
                    continue;
                }

                bool uptimeConvOk;

                qint64 uptime = subItems[3].toLongLong(&uptimeConvOk);

                if (!uptimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 4 (Uptime(Start)) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool frametimeConvOk;

                newDistanceItem.frameDuration = subItems[4].toLongLong(&frametimeConvOk);

                if (!frametimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Frame time) to 64-bit integer. Frame time set to 0.");
                    newDistanceItem.frameDuration = 0;
                }

                if (distances.find(uptime) != distances.end())
                {
                    if (!firstDuplicateUptimeLine)
                    {
                        firstDuplicateUptimeLine = lineNumber;
                    }

                    lastDuplicateUptimeLine = lineNumber;

                    discardedLines++;
                    continue;
                }

                if (firstDuplicateUptimeLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateUptimeLine) + "-"  +
                               QString::number(lastDuplicateUptimeLine) +
                               ": Distance(s) with duplicate uptime(s). Line(s) skipped.");

                    firstDuplicateUptimeLine = 0;
                }

                newDistanceItem.sourceFile = fileName;
                newDistanceItem.sourceFileLine = lineNumber;

                distances[uptime] = newDistanceItem;

                numberOfDistances ++;
            }

            if (firstDuplicateUptimeLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateUptimeLine) + "-"  +
                           QString::number(lastDuplicateUptimeLine) +
                           ": Distance(s) with duplicate uptime(s). Line(s) skipped.");
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid distances: " +
                       QString::number(numberOfDistances) +
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


void PostProcessingForm::on_pushButton_AddDistanceData_clicked()
{
    if (fileDialog_Distances.exec())
    {
        QStringList fileNames = fileDialog_Distances.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Distances.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addDistanceData(fileNames);
    }
}

void PostProcessingForm::on_pushButton_ClearSyncData_clicked()
{
    rovers[0].roverSyncData.clear();
    rovers[1].roverSyncData.clear();

    rovers[0].reverseSync.clear();
    rovers[1].reverseSync.clear();

    addLogLine("Sync data cleared.");
}

void PostProcessingForm::addSyncData(const QStringList& fileNames)
{
    addLogLine("Reading sync data...");

    for (const auto& fileName : fileNames)
    {
        QFileInfo fileInfo(fileName);
        addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

        QFile syncFile;
        syncFile.setFileName(fileName);
        if (syncFile.open(QIODevice::ReadOnly))
        {
            int numberOfSyncItems = 0;

            QTextStream textStream(&syncFile);

            int64_t fileLength = syncFile.size();

            if (fileLength > 0x7FFFFFFFLL)
            {
                addLogLine("Error: File \"" + fileInfo.fileName() + "\" is too big. Skipped.");
                syncFile.close();
                continue;
            }

            QString headerLine = textStream.readLine();

            if (headerLine.compare("Time\tSource\tType\tiTOW\tUptime(Start)\tFrame time", Qt::CaseInsensitive))
            {
                addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Skipped.");
                syncFile.close();
                continue;
            }

            int lineNumber = 1;
            int discardedLines = 0;

            int firstDuplicateSyncItemLine = 0;
            int lastDuplicateSyncItemLine = 0;

            while (!textStream.atEnd())
            {
                lineNumber++;

                QString line = textStream.readLine();

                QStringList subItems = line.split("\t");

                if (subItems.count() < 6)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Not enough tab-separated items. Line skipped.");
                    continue;
                }

                RoverSyncItem newSyncItem;

                QMap<qint64, RoverSyncItem>* roverContainer = nullptr;
                QMap<UBXMessage_RELPOSNED::ITOW, qint64>* reverseContainer = nullptr;

                if (!subItems[1].compare("rover a", Qt::CaseInsensitive))
                {
                    roverContainer = &rovers[0].roverSyncData;
                    reverseContainer = &rovers[0].reverseSync;
                }
                else if (!subItems[1].compare("rover b", Qt::CaseInsensitive))
                {
                    roverContainer = &rovers[1].roverSyncData;
                    reverseContainer = &rovers[1].reverseSync;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Source not either \"rover a\" nor \"rover b\". Line skipped");
                    continue;
                }

                if (!subItems[2].compare("RELPOSNED", Qt::CaseInsensitive))
                {
                    newSyncItem.messageType = RoverSyncItem::MSGTYPE_UBX_RELPOSNED;
                }
                else
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Type not \"RELPOSNED\" (currently only supported type). Line skipped");
                    continue;
                }

                bool iTOWConvOk;

                newSyncItem.iTOW = subItems[3].toInt(&iTOWConvOk);

                if (!iTOWConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 4 (iTOW) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool uptimeConvOk;

                qint64 uptime = subItems[4].toLongLong(&uptimeConvOk);

                if (!uptimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Uptime(Start)) to 64-bit integer. Line skipped.");
                    continue;
                }

                bool frametimeConvOk;

                newSyncItem.frameTime = subItems[5].toLongLong(&frametimeConvOk);

                if (!frametimeConvOk)
                {
                    discardedLines++;
                    addLogLine("Warning: Line " + QString::number(lineNumber) + ": Can't convert column 5 (Frame time) to 64-bit integer. Frame time set to 0.");
                    newSyncItem.frameTime = 0;
                }

                newSyncItem.sourceFile = fileName;
                newSyncItem.sourceFileLine = lineNumber;

                if (roverContainer->find(uptime) != roverContainer->end())
                {
                    discardedLines++;

                    if (!firstDuplicateSyncItemLine)
                    {
                        firstDuplicateSyncItemLine = lineNumber;
                    }

                    lastDuplicateSyncItemLine = lineNumber;
                    continue;
                }

                if (firstDuplicateSyncItemLine)
                {
                    addLogLine("Warning: Line(s) " + QString::number(firstDuplicateSyncItemLine) + "-" +
                               QString::number(lastDuplicateSyncItemLine) +
                               ": Duplicate rover sync item(s). Line(s) skipped.");

                    firstDuplicateSyncItemLine = 0;
                }

                roverContainer->insert(uptime, newSyncItem);
                reverseContainer->insert(newSyncItem.iTOW, uptime);

                numberOfSyncItems ++;
            }

            if (firstDuplicateSyncItemLine)
            {
                addLogLine("Warning: Line(s) " + QString::number(firstDuplicateSyncItemLine) + "-" +
                           QString::number(lastDuplicateSyncItemLine) +
                           ": Duplicate rover sync item(s). Line(s) skipped.");
            }

            addLogLine("File \"" + fileInfo.fileName() + "\" processed. Valid sync items: " +
                       QString::number(numberOfSyncItems) +
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


void PostProcessingForm::on_pushButton_AddSyncData_clicked()
{
    if (fileDialog_Sync.exec())
    {
        QStringList fileNames = fileDialog_Sync.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Sync.setDirectory(QFileInfo(fileNames[0]).path());
        }

        addSyncData(fileNames);
    }
}

void PostProcessingForm::on_pushButton_GenerateSyncDataBasedOnITOWS_clicked()
{
    rovers[0].roverSyncData.clear();
    rovers[1].roverSyncData.clear();

    rovers[0].reverseSync.clear();
    rovers[1].reverseSync.clear();

    addLogLine("Previous sync data cleared.");

    int itemCount = 0;
    int lineNumber = 2; // Header at first line

    for (const auto& relposnedData : rovers[0].relposnedMessages)
    {
        RoverSyncItem fakeSyncItem;

        fakeSyncItem.sourceFile = "None";
        fakeSyncItem.sourceFileLine = lineNumber;
        fakeSyncItem.messageType = RoverSyncItem::MSGTYPE_UBX_RELPOSNED;
        fakeSyncItem.iTOW = relposnedData.iTOW;
        fakeSyncItem.frameTime = 0;
        rovers[0].roverSyncData.insert(fakeSyncItem.iTOW, fakeSyncItem);
        rovers[0].reverseSync.insert(fakeSyncItem.iTOW, fakeSyncItem.iTOW);

        lineNumber += 2;    // "Interleaved" data
        itemCount++;
    }

    lineNumber = 3; // "Interleaved" data

    for (const auto& relposnedData : rovers[1].relposnedMessages)
    {
        RoverSyncItem fakeSyncItem;

        fakeSyncItem.sourceFile = "None";
        fakeSyncItem.sourceFileLine = lineNumber;
        fakeSyncItem.messageType = RoverSyncItem::MSGTYPE_UBX_RELPOSNED;
        fakeSyncItem.iTOW = relposnedData.iTOW;
        fakeSyncItem.frameTime = 0;
        rovers[1].roverSyncData.insertMulti(fakeSyncItem.iTOW, fakeSyncItem);
        rovers[1].reverseSync.insert(fakeSyncItem.iTOW, fakeSyncItem.iTOW);

        lineNumber += 2;    // "Interleaved" data
        itemCount++;
    }

    addLogLine(QString::number(itemCount) + " sync items created.");
}

bool PostProcessingForm::generateTransformationMatrix(Eigen::Matrix4d& outputMatrix)
{
    Eigen::Matrix4d translation_NED;

    for (int i = 0; i < 4; i++)
    {
        for (int k= 0; k < 4; k++)
        {
            translation_NED(i, k) = 0;
        }

        translation_NED(i, i) = 1;
    }

    translation_NED(0, 3) = ui->doubleSpinBox_Translation_N->value();
    translation_NED(1, 3) = ui->doubleSpinBox_Translation_E->value();
    translation_NED(2, 3) = ui->doubleSpinBox_Translation_D->value();

    Eigen::Matrix4d prelimTransform;

    for (int i = 0; i < 4; i++)
    {
        for (int k= 0; k < 4; k++)
        {
            bool ok;
            prelimTransform(i, k) = ui->tableWidget_TransformationMatrix->item(i, k)->text().toDouble(&ok);

            if (!ok)
            {
                addLogLine("Error: Row " + QString::number(i + 1) +
                           ", column " + QString::number(k + 1) +
                           " of transformation matrix not convertible to a (double precision) floating point value. "
                           "Unable to perform NED->XYZ-coordinate conversion.");

                return false;
            }
        }
    }

    outputMatrix = prelimTransform * translation_NED;


    return true;
}

void PostProcessingForm::on_pushButton_ClearAllFileData_clicked()
{
    this->on_pushButton_ClearRELPOSNEDData_RoverA_clicked();
    this->on_pushButton_ClearRELPOSNEDData_RoverB_clicked();
    this->on_pushButton_ClearTagData_clicked();
    this->on_pushButton_ClearDistanceData_clicked();
    this->on_pushButton_ClearSyncData_clicked();
}

void PostProcessingForm::addAllData(const bool includeTransformation)
{
    if (fileDialog_All.exec())
    {
        QStringList selectedFileNames = fileDialog_All.selectedFiles();

        if (selectedFileNames.size() != 0)
        {
            fileDialog_All.setDirectory(QFileInfo(selectedFileNames[0]).path());
        }

        QStringList baseFileNames;

        for (const auto& fileName : selectedFileNames)
        {
            struct
            {
                QString endsWith;
                QString formatName;
            } const fileNameEndings [] =
            {
                { "_base.NMEA", "NMEA (base)" },
                { "_base.raw", "raw (base)" },
                { "_base.RTCM", "RTCM (base)" },
                { "_base.ubx", "ubx (base)" },

                { "_RoverA.NMEA", "NMEA (Rover A)" },
                { "_RoverA.raw", "raw (Rover A)" },
                { "_RoverA.ubx", "ubx (Rover A)" },
                { "_RoverA_RELPOSNED.ubx", "ubx (relposned, Rover A)" },

                { "_RoverB.NMEA", "NMEA (Rover B)" },
                { "_RoverB.raw", "raw (Rover B)" },
                { "_RoverB.ubx", "ubx (Rover B)" },
                { "_RoverB_RELPOSNED.ubx", "ubx (relposned, Rover B)" },

                { "_tags.tags", "tags (""double extension""" },
                { ".tags", "tags (""single extension""" },

                { ".distances", "distances" },

                { ".sync", "sync" },
                { ".transformation", "transformation" },
            };

            bool formatKnown = false;

            for (unsigned int endingIndex = 0; endingIndex < (sizeof(fileNameEndings) / sizeof(fileNameEndings[0])); endingIndex++)
            {
                if (fileName.endsWith(fileNameEndings[endingIndex].endsWith, Qt::CaseInsensitive))
                {
                    QString baseFileName = fileName;
                    baseFileName.resize(baseFileName.length() - fileNameEndings[endingIndex].endsWith.length());

                    addLogLine(QString("File selection """) + fileName + """ recognized as a format """ +
                               fileNameEndings[endingIndex].formatName + """. Base filename: """ +
                               baseFileName + """");

                    baseFileNames.append(baseFileName);

                    formatKnown = true;
                    break;
                }
            }

            if (!formatKnown)
            {
                addLogLine(QString("Warning: file selection """) + fileName +
                           """ not recognized as any supported format. Skipping... ");
            }
        }

        QStringList fileNames = getAppendedFileNames(baseFileNames, "_RoverA_RELPOSNED.ubx");
        addRELPOSNEDData_Rover(fileNames, 0);

        fileNames = getAppendedFileNames(baseFileNames, "_RoverB_RELPOSNED.ubx");
        addRELPOSNEDData_Rover(fileNames, 1);

        fileNames = getAppendedFileNames(baseFileNames, "_tags.tags");
        addTagData(fileNames);

        fileNames = getAppendedFileNames(baseFileNames, ".distances");
        addDistanceData(fileNames);

        fileNames = getAppendedFileNames(baseFileNames, ".sync");
        addSyncData(fileNames);

        if (includeTransformation)
        {
            fileNames = getAppendedFileNames(baseFileNames, ".Transformation");

            if (fileNames.count() == 0)
            {
                addLogLine("Warning: No file(s) selected. Transformation not read.");
            }
            else
            {
                if (fileNames.count() != 1)
                {
                    addLogLine("Warning: Multiple files selected. Transformation read only using the first one ("""+
                               fileNames[0] + """)");
                }

                loadTransformation(fileNames[0]);
            }
        }
    }
}


void PostProcessingForm::on_pushButton_AddAll_clicked()
{
    addAllData(false);
}

QStringList PostProcessingForm::getAppendedFileNames(const QStringList& fileNames, const QString appendix)
{
    QStringList appendedList;

    for (auto fileName : fileNames)
    {
        appendedList.append(fileName + appendix);
    }

    return appendedList;
}

static QString getTransformationFileHeaderLine(void)
{
    QString line = "Translation_N\tTranslation_E\tTranslation_D";

    for (int row = 0; row < 4; row++)
    {
        for (int column = 0; column < 4; column++)
        {
            line += "\tTransf R" + QString::number(row + 1) + ", C" + QString::number(column + 1);
        }
    }

    return line;
}

void PostProcessingForm::loadTransformation(const QString fileName)
{
    QFileInfo fileInfo(fileName);
    addLogLine("Opening file \"" + fileInfo.fileName() + "\"...");

    QFile transformationFile;
    transformationFile.setFileName(fileName);
    if (transformationFile.open(QIODevice::ReadOnly))
    {
        QTextStream textStream(&transformationFile);

        QString headerLine = textStream.readLine();

        if (headerLine.compare(getTransformationFileHeaderLine(), Qt::CaseInsensitive))
        {
            addLogLine("Error: File's \"" + fileInfo.fileName() + "\" doesn't have correct header. Data not read.");
            transformationFile.close();
            return;
        }

        QString dataLine = textStream.readLine();

        QStringList dataLineItems = dataLine.split("\t");

        if (dataLineItems.count() != (16 + 3))
        {
            addLogLine("Error: File's \"" + fileInfo.fileName() + "\" data line doesn't have correct number of items (19). Data not read.");
            transformationFile.close();
            return;
        }
        else
        {
            double convertedItems[3];
            bool convOk = true;

            for (int itemIndex = 0; itemIndex < (3); itemIndex++)
            {
                convertedItems[itemIndex] = dataLineItems[itemIndex].toDouble(&convOk);

                if (!convOk)
                {
                    addLogLine("Error: File's \"" + fileInfo.fileName() +
                               "\" data line item #" + QString::number(itemIndex + 1) +
                               "can't be converted to double. Data not read.");

                    transformationFile.close();

                    break;
                }
            }

            if (convOk)
            {
                ui->doubleSpinBox_Translation_N->setValue(convertedItems[0]);
                ui->doubleSpinBox_Translation_E->setValue(convertedItems[1]);
                ui->doubleSpinBox_Translation_D->setValue(convertedItems[2]);

                for (int row = 0; row < 4; row++)
                {
                    for (int column = 0; column < 4; column++)
                    {
                        ui->tableWidget_TransformationMatrix->item(row, column)->setText(dataLineItems[3 + (row * 4) + column]);
                    }
                }
                addLogLine("File read.");
            }
        }
    }
    else
    {
        addLogLine("Error: can't open file \"" + fileInfo.fileName() + "\". Data not read.");
    }
}

void PostProcessingForm::on_pushButton_LoadTransformation_clicked()
{
    if (fileDialog_Transformation_Load.exec())
    {
        QStringList fileNames = fileDialog_Transformation_Load.selectedFiles();

        if (fileNames.size() != 0)
        {
            fileDialog_Transformation_Load.setDirectory(QFileInfo(fileNames[0]).path());

            loadTransformation(fileNames[0]);
        }
        else
        {
            addLogLine("Warning: No transformation file selected. Data not read.");
        }
    }
}

void PostProcessingForm::on_pushButton_SaveTransformation_clicked()
{
    if (fileDialog_Transformation_Save.exec())
    {
        QStringList fileNameList = fileDialog_Transformation_Save.selectedFiles();

        if (fileNameList.length() != 1)
        {
            addLogLine("Error: Multiple file selection not supported. Transformation not saved.");
            return;
        }

        QFile transformationFile;

        transformationFile.setFileName(fileNameList[0]);

        if (transformationFile.exists())
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
                addLogLine("Transformation not saved.");
                return;
            }
        }

        if (!transformationFile.open(QIODevice::WriteOnly))
        {
            addLogLine("Error: Can't open transformation file.");
            return;
        }

        QTextStream textStream(&transformationFile);

        textStream << getTransformationFileHeaderLine() << "\n";

        textStream << QString::number(ui->doubleSpinBox_Translation_N->value()) << "\t"
                      << QString::number(ui->doubleSpinBox_Translation_E->value()) << "\t"
                      << QString::number(ui->doubleSpinBox_Translation_D->value());

        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                textStream << "\t" << ui->tableWidget_TransformationMatrix->item(row, column)->text();
            }
        }

        textStream << "\n";
    }
}

void PostProcessingForm::on_pushButton_AddAllIncludingTransform_clicked()
{
    addAllData(true);
}

void PostProcessingForm::on_pushButton_Preset_clicked()
{
    int presetIndex = ui->comboBox_Presets->currentIndex();

    if ((presetIndex < 0) || (presetIndex > int(sizeof(transformationPresets) / sizeof(transformationPresets[0]))))
    {
        addLogLine("Error: index out of bounds (this should never happen...)");
    }
    else
    {
        for (int row = 0; row < 4; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                ui->tableWidget_TransformationMatrix->item(row, column)->setText(QString::number(transformationPresets[presetIndex].values[row][column]));
            }
        }

        ui->tableWidget_TransformationMatrix->item(3, 3)->setText(QString::number(transformationPresets[presetIndex].values[3][3]));
    }
}

QString PostProcessingForm::getRoverIdentString(const unsigned int roverId)
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
