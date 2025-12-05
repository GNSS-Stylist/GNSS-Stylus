/*
    rovertrackgenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2025-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
#include <limits> // QtEndian (below) seems to need this... See https://bugreports.qt.io/browse/QTBUG-90395
#include <QtEndian>

#include "rovertrackgenerator.h"


void RoverTrackGenerator::generateTrack(const Params& params)
{
    this->params = params;

    file.setFileName(params.fileName);

    if (file.exists())
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
            emit infoMessage("Generating of rover track cancelled.");
            return;
        }
    }

//    Eigen::Transform<double, 3, Eigen::Affine> transform_XYZToNED_NoTranslation;
//    transform_XYZToNED_NoTranslation = (*params.transform_NEDToXYZ).linear().transpose();

    if (!file.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open rover track file.");
        return;
    }

    writeHeader();

    numberOfPointsWritten = 0;

//    UBXMessage_RELPOSNED::ITOW iTOWMismatchStart = -1;
//    unsigned int iTOWMismatchCount = 0;

    unsigned int warningCount = 0;

    QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterator;

    relposIterator = params.rover->relposnedMessages.lowerBound(params.iTOWRange_Script_Min);

    while (relposIterator != params.rover->relposnedMessages.end())
    {
        if (warningCount >= 1000)
        {
            emit errorMessage("Maximum number of warnings (1000) reached. "
                       "Please check your data.");

//            iTOWMismatchCount = 0;
            break;
        }

        const UBXMessage_RELPOSNED& relPosNED = relposIterator.value();

        if (relPosNED.iTOW > params.iTOWRange_Script_Max)
        {
            break;
        }

        unsigned int itemTime;

        if (params.timeFormat == Params::TimeFormat::TF_UPTIME)
        {
            itemTime = params.rover->reverseSync.find(relPosNED.iTOW).value();
        }
        else
        {
            itemTime = relPosNED.iTOW;
        }

        writeRELPOSNEDItem(relPosNED, itemTime);
        numberOfPointsWritten++;

        for (int i = 0; i < (params.frameSkip + 1); i++)
        {
            relposIterator++;
            if (relposIterator == params.rover->relposnedMessages.end())
            {
                break;
            }
        }
    }

    finalizeFile();

    if (file.isOpen())
    {
        file.close();
    }

    emit infoMessage("Rover track generated. Number of items: " + QString::number(numberOfPointsWritten));
}


void RoverTrackGenerator::writeHeader(void)
{
    QByteArray eol = params.endOfLine;

    QByteArray dataToWrite = "ply" + eol;

    if (params.binary)
    {
        dataToWrite += "format binary_little_endian 1.0" + eol;
    }
    else
    {
        dataToWrite += "format ascii 1.0" + eol;
    }

    dataToWrite += "comment Rover track-file created with GNSS-Stylus on (dd.mm.yyyy hh:mm): " + QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm").toLatin1() + eol;
    dataToWrite += "comment ITOW range for the script, min: " + QString::number(params.iTOWRange_Script_Min).toLatin1() + eol;
    dataToWrite += "comment ITOW range for the script, max: " + QString::number(params.iTOWRange_Script_Max).toLatin1() + eol;
    dataToWrite += "comment Frame skip: " + QString::number(params.frameSkip).toLatin1() + eol;

    dataToWrite += "comment AccuracyBasis:";
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            dataToWrite += " " + QString::number(params.transform_NEDToXYZ->matrix()(row, col), 'f', 3).toLatin1();
        }
    }
    dataToWrite += eol;

    // rows "comment pad" and "element vertex (N/A) added here this way to allow updating them later with simple overwriting some bytes.
    // Vertex count is not known when creating the file so it needs to be updated as one of the last steps when finalizing the file.
    // (Seems that Meshlab actually allows '0'-padded length, but the original paper doesn't say anything about padding, so better to be safe).
    dataToWrite += "comment pad";
    vertexCountFirstByte = dataToWrite.length();
    dataToWrite += "     " + eol + "element vertex (N/A)";
    vertexCountLastByte = dataToWrite.length();
    dataToWrite += eol;

    if (params.coordsFormat != Params::CF_NONE)
    {
        QByteArray coordFormatString = getCoordFormatString(params.coordsFormat);
        dataToWrite += "property " + coordFormatString + " x" + eol;
        dataToWrite += "property " + coordFormatString + " y" + eol;
        dataToWrite += "property " + coordFormatString + " z" + eol;
    }

    switch (params.timeFormat)
    {
    case Params::TF_NONE:
        break;
    case Params::TF_UPTIME:
    case Params::TF_ITOW:
        dataToWrite += "property uint time" + eol;
        break;
    default:
        qFatal("Unimplemented time format.");
        break;
    }

    if (params.accuracyFormat != Params::CF_NONE)
    {
        QByteArray coordFormatString = getCoordFormatString(params.coordsFormat);
        dataToWrite += "property " + coordFormatString + " accn" + eol;
        dataToWrite += "property " + coordFormatString + " acce" + eol;
        dataToWrite += "property " + coordFormatString + " accd" + eol;
    }

    switch (params.flagsFormat)
    {
    case Params::FF_NONE:
        break;
    case Params::FF_RAW:
        dataToWrite += "property ushort flags" + eol;
        break;
    case Params::FF_CARR_SOLN:
        dataToWrite += "property uchar flags_carrsoln" + eol;
        break;
    default:
        qFatal("Unimplemented flags format.");
        break;
    }

    dataToWrite += "end_header" + eol;

    file.write(dataToWrite);
}

static inline void writeDouble(QFile& file, const double src)
{
    char buf[8];
    qToLittleEndian(src, buf);
    file.write(buf, 8);
}

static inline void writeFloat(QFile& file, const float src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

static inline void writeChar(QFile& file, const char src)
{
    file.write(&src, 1);
}

static inline void writeShort(QFile& file, const short src)
{
    char buf[2];
    qToLittleEndian(src, buf);
    file.write(buf, 2);
}

static inline void writeInt(QFile& file, const int src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

void RoverTrackGenerator::writeRELPOSNEDItem(const UBXMessage_RELPOSNED& relPosNED, const unsigned int time)
{
    Eigen::Vector3d nedVector = Eigen::Vector3d(relPosNED.relPosN, relPosNED.relPosE, relPosNED.relPosD);
    Eigen::Vector3d xyzVector = *params.transform_NEDToXYZ * nedVector;

    if (params.binary)
    {
        switch (params.coordsFormat)
        {
        case Params::CF_NONE:
            break;

        case Params::CF_FLOAT:
            writeFloat(file, xyzVector.x());
            writeFloat(file, xyzVector.y());
            writeFloat(file, xyzVector.z());
            break;

        case Params::CF_DOUBLE:
            writeDouble(file, xyzVector.x());
            writeDouble(file, xyzVector.y());
            writeDouble(file, xyzVector.z());
            break;
        }

        switch (params.timeFormat)
        {
        case Params::TF_NONE:
            break;

        case Params::TF_ITOW:
        case Params::TF_UPTIME:
            writeInt(file, time);
            break;
        }        

        switch (params.accuracyFormat)
        {
        case Params::CF_NONE:
            break;

        case Params::CF_FLOAT:
            writeFloat(file, relPosNED.accN);
            writeFloat(file, relPosNED.accE);
            writeFloat(file, relPosNED.accD);
            break;

        case Params::CF_DOUBLE:
            writeDouble(file, relPosNED.accN);
            writeDouble(file, relPosNED.accE);
            writeDouble(file, relPosNED.accD);
            break;
        }

        switch (params.flagsFormat)
        {
        case Params::FF_NONE:
            break;
        case Params::FF_RAW:
            writeShort(file, relPosNED.flags);
            break;
        case Params::FF_CARR_SOLN:
            writeChar(file, relPosNED.flag_carrSoln);
            break;
        }
    } // if (binary)
    else
    {
        QString lineOut;

        switch (params.coordsFormat)
        {
        case Params::CF_NONE:
            break;
        case Params::CF_FLOAT:
        case Params::CF_DOUBLE:
            lineOut = QString::number(xyzVector.x(), 'f', params.numberOfDecimals_Coords) +
                      " " + QString::number(xyzVector.y(), 'f', params.numberOfDecimals_Coords) +
                      " " + QString::number(xyzVector.z(), 'f', params.numberOfDecimals_Coords);
        }

        switch (params.timeFormat)
        {
        case Params::TF_NONE:
            break;
        case Params::TF_ITOW:
        case Params::TF_UPTIME:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(time);
            break;
        }

        switch (params.accuracyFormat)
        {
        case Params::CF_NONE:
            break;

        case Params::CF_FLOAT:
        case Params::CF_DOUBLE:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(relPosNED.accN, 'f', params.numberOfDecimals_Accuracy) +
                      " " + QString::number(relPosNED.accE, 'f', params.numberOfDecimals_Accuracy) +
                      " " + QString::number(relPosNED.accD, 'f', params.numberOfDecimals_Accuracy);
            break;
        }

        switch (params.flagsFormat)
        {
        case Params::FF_NONE:
            break;
        case Params::FF_RAW:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(relPosNED.flags);
            break;
        case Params::FF_CARR_SOLN:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut += QString::number(relPosNED.flag_carrSoln);
            break;
        }

        lineOut += params.endOfLine;
        QByteArray bytesToWrite = lineOut.toLatin1();
        file.write(bytesToWrite);
    }
}


void RoverTrackGenerator::finalizeFile(void)
{
    // Need to write number of vertices as one of the last steps as it isn't known before.
    unsigned int pointCount = numberOfPointsWritten;
    QByteArray stringToWrite = params.endOfLine + "element vertex " + QString::number(pointCount).toLatin1();

    int numOfBytesToWrite = vertexCountLastByte - vertexCountFirstByte;
    while (stringToWrite.length() < numOfBytesToWrite)
    {
        stringToWrite = QByteArray(" ") + stringToWrite;
    }

    file.seek(vertexCountFirstByte);
    file.write(stringToWrite);
}


QByteArray RoverTrackGenerator::getCoordFormatString(const Params::CoordsFormat format)
{
    QByteArray coordsFormatString;

    switch (format)
    {
    case Params::CF_NONE:
        coordsFormatString = "N/A";
        break;
    case Params::CF_FLOAT:
        coordsFormatString = "float";
        break;
    case Params::CF_DOUBLE:
        coordsFormatString = "double";
        break;
    default:
        qFatal("Unimplemented coordinate format.");
        break;
    }

    return coordsFormatString;
}
