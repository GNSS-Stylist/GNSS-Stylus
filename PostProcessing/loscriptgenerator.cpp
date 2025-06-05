/*
    loscriptgenerator.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-present Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "loscriptgenerator.h"


void LOScriptGenerator::generateScript(const Params& params)
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
            emit infoMessage("Generating of location/orientation script cancelled.");
            return;
        }
    }

    Eigen::Transform<double, 3, Eigen::Affine> transform_XYZToNED_NoTranslation;
    transform_XYZToNED_NoTranslation = (*params.transform_NEDToXYZ).linear().transpose();

    if (!file.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open location/orientation script file.");
        return;
    }

    writeHeader();

    UBXMessage_RELPOSNED::ITOW currentITOW = params.iTOWRange_Script_Min;

    numberOfPointsWritten = 0;

    UBXMessage_RELPOSNED::ITOW iTOWMismatchStart = -1;
    unsigned int iTOWMismatchCount = 0;

    unsigned int warningCount = 0;

    while (currentITOW <= params.iTOWRange_Script_Max)
    {
        if (warningCount >= 1000)
        {
            emit errorMessage("Maximum number of warnings (1000) reached. "
                       "Please check your data.");

            iTOWMismatchCount = 0;
            break;
        }

        QMap<UBXMessage_RELPOSNED::ITOW, UBXMessage_RELPOSNED>::const_iterator relposIterators[3];

        relposIterators[0] = params.rovers[0].relposnedMessages.lowerBound(currentITOW);
        relposIterators[1] = params.rovers[1].relposnedMessages.lowerBound(currentITOW);
        relposIterators[2] = params.rovers[2].relposnedMessages.lowerBound(currentITOW);

        bool endOfData = false;

        for (unsigned int i = 0; i < 3; i++)
        {
            if (relposIterators[i] == params.rovers[i].relposnedMessages.end())
            {
                // No more data
                endOfData = true;
            }
        }

        if (endOfData)
        {
            break;
        }

        UBXMessage_RELPOSNED::ITOW lowestNextRoverITOW = 1e9;

        for (unsigned int i = 0; i < 3; i++)
        {
            if (relposIterators[i].value().iTOW < lowestNextRoverITOW)
            {
                lowestNextRoverITOW = relposIterators[i].value().iTOW;
            }
        }

        bool roverITOWSInSync = true;

        for (unsigned int i = 0; i < 3; i++)
        {
            if (relposIterators[i].value().iTOW != lowestNextRoverITOW)
            {
                roverITOWSInSync = false;
            }
        }

        if (!roverITOWSInSync)
        {
            if (iTOWMismatchCount == 0)
            {
                // First mismatch in this set

                iTOWMismatchStart = lowestNextRoverITOW;
                iTOWMismatchCount = 1;
            }
            else
            {
                iTOWMismatchCount++;
            }

            currentITOW = lowestNextRoverITOW + 1;
            continue;
        }
        else if (iTOWMismatchCount != 0)
        {
            emit warningMessage("Mismatch in rover iTOWs, range: \"" +
                       QString::number(iTOWMismatchStart) + " - " + QString::number(lowestNextRoverITOW - 1) +
                       ", number of discarded iTOWS: " + QString::number(iTOWMismatchCount));

            iTOWMismatchCount = 0;
            warningCount++;
        }

        Eigen::Vector3d points[3];

        for (unsigned int i = 0; i < 3; i++)
        {
            points[i] = Eigen::Vector3d(relposIterators[i].value().relPosN, relposIterators[i].value().relPosE, relposIterators[i].value().relPosD);
        }

        if (!params.loSolver->setPoints(points))
        {
            emit warningMessage("Error setting points. ITOW: \"" +
                       QString::number(lowestNextRoverITOW) +
                       ", error code: " + QString::number(params.loSolver->getLastError()));

            currentITOW = lowestNextRoverITOW + 1;
            warningCount++;
            continue;
        }

        Eigen::Transform<double, 3, Eigen::Affine> loTransformNED;

        if (!params.loSolver->getTransformMatrix(loTransformNED))
        {
            emit warningMessage("Error calculating transform matrix. ITOW: \"" +
                       QString::number(lowestNextRoverITOW) +
                       ", error code: " + QString::number(params.loSolver->getLastError()));

            currentITOW = lowestNextRoverITOW + 1;
            warningCount++;
            continue;
        }

        Eigen::Transform<double, 3, Eigen::Affine> finalMatrix = *params.transform_NEDToXYZ * loTransformNED * *params.transform_Generated * transform_XYZToNED_NoTranslation;

        unsigned int loItemTime;

        if (params.timeFormat == Params::TimeFormat::TF_UPTIME)
        {
            qint64 timeSum = 0;
            unsigned int timeValues = 0;
            bool fail = false;

            for (int i = 0; i < 3; i++)
            {
                if (params.rovers[i].reverseSync.find(lowestNextRoverITOW) != params.rovers[i].reverseSync.end())
                {
                    timeSum += params.rovers[i].reverseSync.find(lowestNextRoverITOW).value();
                    timeValues++;
                }
                else
                {
                    fail = true;
                    break;
                }
            }

            if (fail)
            {
                emit warningMessage("Can not find reverse sync (ITOW -> uptime) for all rovers. ITOW: \"" +
                           QString::number(lowestNextRoverITOW));

                currentITOW = lowestNextRoverITOW + 1;
                warningCount++;
                continue;
            }

            timeSum /= timeValues;

            loItemTime = timeSum;
        }
        else
        {
            loItemTime = lowestNextRoverITOW;
        }

        writeLOItem(finalMatrix, loItemTime);
        numberOfPointsWritten++;

        currentITOW = lowestNextRoverITOW + 1;
    }

    if (iTOWMismatchCount != 0)
    {
        emit warningMessage("Mismatch in rover iTOWs in the end of rover data, first ITOW: \"" +
                   QString::number(iTOWMismatchStart) +
                   ", number of discarded iTOWS: " + QString::number(iTOWMismatchCount));

        warningCount++;
    }

    finalizeFile();

    if (file.isOpen())
    {
        file.close();
    }

    emit infoMessage("Location/orientation script generated. Number of items: " + QString::number(numberOfPointsWritten));
}


void LOScriptGenerator::writeHeader(void)
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

    dataToWrite += "comment LOScript-file created with GNSS-Stylus on (dd.mm.yyyy hh:mm): " + QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm").toLatin1() + eol;
    dataToWrite += "comment ITOW range for the script, min: " + QString::number(params.iTOWRange_Script_Min).toLatin1() + eol;
    dataToWrite += "comment ITOW range for the script, max: " + QString::number(params.iTOWRange_Script_Max).toLatin1() + eol;

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

    switch (params.orientationFormat)
    {
    case Params::OF_NONE:
        break;
    case Params::OF_BASIS:
        dataToWrite += "property float basis_xx" + eol +
                       "property float basis_xy" + eol +
                       "property float basis_xz" + eol +
                       "property float basis_yx" + eol +
                       "property float basis_yy" + eol +
                       "property float basis_yz" + eol +
                       "property float basis_zx" + eol +
                       "property float basis_zy" + eol +
                       "property float basis_zz" + eol;
        break;
    case Params::OF_QUATERNION:
        dataToWrite += "property float quat_x" + eol +
                       "property float quat_y" + eol +
                       "property float quat_z" + eol +
                       "property float quat_w" + eol;
        break;
    default:
        qFatal("Unimplemented orientation format.");
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


static inline void writeInt(QFile& file, const int src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

void LOScriptGenerator::writeLOItem(const Eigen::Transform<double, 3, Eigen::Affine>& finalMatrix, const unsigned int time)
{
    if (params.binary)
    {
        switch (params.coordsFormat)
        {
        case Params::CF_NONE:
            break;

        case Params::CF_FLOAT:
            writeFloat(file, finalMatrix(0, 3));
            writeFloat(file, finalMatrix(1, 3));
            writeFloat(file, finalMatrix(2, 3));
            break;

        case Params::CF_DOUBLE:
            writeDouble(file, finalMatrix(0, 3));
            writeDouble(file, finalMatrix(1, 3));
            writeDouble(file, finalMatrix(2, 3));
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

        switch (params.orientationFormat)
        {
        case Params::OF_NONE:
            break;
        case Params::OF_BASIS:
            writeFloat(file, finalMatrix(0, 0));
            writeFloat(file, finalMatrix(1, 0));
            writeFloat(file, finalMatrix(2, 0));

            writeFloat(file, finalMatrix(0, 1));
            writeFloat(file, finalMatrix(1, 1));
            writeFloat(file, finalMatrix(2, 1));

            writeFloat(file, finalMatrix(0, 2));
            writeFloat(file, finalMatrix(1, 2));
            writeFloat(file, finalMatrix(2, 2));
            break;

        case Params::OF_QUATERNION:
            Eigen::Quaterniond quat(finalMatrix.rotation());
            writeFloat(file, quat.x());
            writeFloat(file, quat.y());
            writeFloat(file, quat.z());
            writeFloat(file, quat.w());
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
            lineOut = QString::number(finalMatrix(0, 3), 'f', params.numberOfDecimals_Coords) +
                      " " + QString::number(finalMatrix(1, 3), 'f', params.numberOfDecimals_Coords) +
                      " " + QString::number(finalMatrix(2, 3), 'f', params.numberOfDecimals_Coords);
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

        switch (params.orientationFormat)
        {
        case Params::OF_NONE:
            break;
        case Params::OF_BASIS:
            if (lineOut.length() != 0) lineOut += " ";
            lineOut +=
                    " " + QString::number(finalMatrix(0, 0), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(1, 0), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(2, 0), 'f', params.numberOfDecimals_Basis) +

                    " " + QString::number(finalMatrix(0, 1), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(1, 1), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(2, 1), 'f', params.numberOfDecimals_Basis) +

                    " " + QString::number(finalMatrix(0, 2), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(1, 2), 'f', params.numberOfDecimals_Basis) +
                    " " + QString::number(finalMatrix(2, 2), 'f', params.numberOfDecimals_Basis);
            break;
        case Params::OF_QUATERNION:
            Eigen::Quaterniond quat(finalMatrix.rotation());
            lineOut += QString::number(quat.x(), 'f', params.numberOfDecimals_Quaternion) +
                       " " + QString::number(quat.y(), 'f', params.numberOfDecimals_Quaternion) +
                       " " + QString::number(quat.z(), 'f', params.numberOfDecimals_Quaternion) +
                       " " + QString::number(quat.w(), 'f', params.numberOfDecimals_Quaternion);
        }

        lineOut += params.endOfLine;
        QByteArray bytesToWrite = lineOut.toLatin1();
        file.write(bytesToWrite);
    }
}


void LOScriptGenerator::finalizeFile(void)
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


QByteArray LOScriptGenerator::getCoordFormatString(const Params::CoordsFormat format)
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
