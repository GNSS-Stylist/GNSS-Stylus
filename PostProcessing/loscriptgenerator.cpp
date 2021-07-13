/*
    loscriptgenerator.cpp (part of GNSS-Stylus)
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

#include <QMessageBox>
#include <QPushButton>

#include "loscriptgenerator.h"


void LOScriptGenerator::generateScript(const Params& params)
{
    QFile loScriptFile;

    loScriptFile.setFileName(params.fileName);

    if (loScriptFile.exists())
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

    if (!loScriptFile.open(QIODevice::WriteOnly))
    {
        emit errorMessage("Can't open location/orientation script file.");
        return;
    }

    QTextStream textStream(&loScriptFile);

    emit infoMessage("Processing location/orientation script...");

    // Add some metadata to make possible changes in the future easier
    textStream << "META\tHEADER\tGNSS location/orientation script\n";
    textStream << "META\tVERSION\t1.0.1\n";
    textStream << "META\tFORMAT\tASCII\n";
    textStream << "META\tCONTENT\tDEFAULT\n";

    QString timeStampColumnText = "iTOW";

    if (params.timeStampFormat == Params::TimeStampFormat::TSF_UPTIME)
    {
        textStream << "META\tTIMESTAMPS\tUPTIME\n";
        timeStampColumnText = "Uptime";
    }
    textStream << "META\tEND\n";

    textStream << timeStampColumnText + "\t"
                  "Origin_X\tOrigin_Y\tOrigin_Z\t"
                  "Basis_XX\tBasis_XY\tBasis_XZ\t"
                  "Basis_YX\tBasis_YY\tBasis_YZ\t"
                  "Basis_ZX\tBasis_ZY\tBasis_ZZ\n";

    //iTOWRange_Script_Min -= iTOWRange_Script_Min % expectedITOWAlignment; // Round to previous aligned ITOW

    UBXMessage_RELPOSNED::ITOW currentITOW = params.iTOWRange_Script_Min;

    unsigned int itemCount = 0;

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

        QString timeString;

        if (params.timeStampFormat == Params::TimeStampFormat::TSF_UPTIME)
        {
            qint64 timeSum = 0;
            bool fail = false;

            for (int i = 0; i < 3; i++)
            {
                if (params.rovers[i].reverseSync.find(lowestNextRoverITOW) != params.rovers[i].reverseSync.end())
                {
                    timeSum += params.rovers[i].reverseSync.find(lowestNextRoverITOW).value();
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

                warningCount++;
            }

            timeSum /= 3;

            timeString = QString::number(timeSum);
        }
        else
        {
            timeString = QString::number(lowestNextRoverITOW);
        }

        const int originDecimals = 4;
        const int unitVectorDecimals = 6;

        QString lineOut =
                timeString +

                "\t" + QString::number(finalMatrix(0, 3), 'f', originDecimals) +
                "\t" + QString::number(finalMatrix(1, 3), 'f', originDecimals) +
                "\t" + QString::number(finalMatrix(2, 3), 'f', originDecimals) +

                "\t" + QString::number(finalMatrix(0, 0), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(1, 0), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(2, 0), 'f', unitVectorDecimals) +

                "\t" + QString::number(finalMatrix(0, 1), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(1, 1), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(2, 1), 'f', unitVectorDecimals) +

                "\t" + QString::number(finalMatrix(0, 2), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(1, 2), 'f', unitVectorDecimals) +
                "\t" + QString::number(finalMatrix(2, 2), 'f', unitVectorDecimals);
                textStream << (lineOut + "\n");

        currentITOW = lowestNextRoverITOW + 1;

        itemCount++;
    }

    if (iTOWMismatchCount != 0)
    {
        emit warningMessage("Mismatch in rover iTOWs in the end of rover data, first ITOW: \"" +
                   QString::number(iTOWMismatchStart) +
                   ", number of discarded iTOWS: " + QString::number(iTOWMismatchCount));

        warningCount++;
    }

    emit infoMessage("Location/orientation script generated. Number of rows: " + QString::number(itemCount));
}
