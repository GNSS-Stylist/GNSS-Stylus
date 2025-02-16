/*
    loscriptgenerator.h (part of GNSS-Stylus)
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

#ifndef LOSCRIPTGENERATOR_H
#define LOSCRIPTGENERATOR_H

#include <QObject>

#include "postprocessingform.h"

class LOScriptGenerator : public QObject
{
    Q_OBJECT

public:
    class Params
    {
    public:

        enum CoordsFormat
        {
            CF_NONE = 0,    // Do not write these coords
            CF_FLOAT,       // Metres
            CF_DOUBLE,      // Metres
        };

        enum TimeFormat
        {
            TF_NONE = 0,
            TF_ITOW,
            TF_UPTIME
        };

        enum OrientationFormat
        {
            OF_NONE = 0,
            OF_BASIS,
            OF_QUATERNION,
        };

        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        Eigen::Transform<double, 3, Eigen::Affine>* transform_Generated = nullptr;
        LOSolver* loSolver;
        const PostProcessingForm::Rover* rovers = nullptr;

        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = 0;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = 1e9;

        QString fileName;
        bool binary = false;
        CoordsFormat coordsFormat = CF_FLOAT;
        QByteArray endOfLine = "\n";
        TimeFormat timeFormat = TF_ITOW;
        OrientationFormat orientationFormat = OF_BASIS;

        int numberOfDecimals_Coords = 4;
        int numberOfDecimals_Basis = 6;
        int numberOfDecimals_Quaternion = 6;
    };

    void generateScript(const Params& params);

private:
    Params params;

    QFile file;
    void writeHeader(void);
    void finalizeFile(void);

    void writeLOItem(const Eigen::Transform<double, 3, Eigen::Affine>& matrix, const unsigned int time);
    QByteArray getCoordFormatString(const Params::CoordsFormat format);

    unsigned int vertexCountFirstByte = 0;
    unsigned int vertexCountLastByte = 0;
    unsigned int numberOfPointsWritten = 0;

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

};


#endif // LOSCRIPTGENERATOR_H
