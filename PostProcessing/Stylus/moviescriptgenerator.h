/*
    moviescriptgenerator.h (part of GNSS-Stylus)
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
 * @file moviescriptgenerator.h
 * @brief Declaration for a class generating a "movie script" that can be used with Processing-sketch (for example)
 */

#ifndef MOVIESCRIPTGENERATOR_H
#define MOVIESCRIPTGENERATOR_H

#include "../postprocessingform.h"

namespace Stylus
{

class MovieScriptGenerator : public QObject
{
    Q_OBJECT

public:

    class Params
    {
    public:
        QString fileName = "";
        const Eigen::Transform<double, 3, Eigen::Affine>* transform = nullptr;
        QString tagIdent_BeginNewObject = "New object";
        QString tagIdent_BeginPoints = "RMB";
        QString tagIdent_EndPoints = "LMB";
        double initialStylusTipDistanceFromRoverA = 0;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Min = 0;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Lines_Max = 1e9;
        unsigned int expectedITOWAlignment = 125;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = 0;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = 1e9;
        double fps = 30;
        double cameraNShift = 0;
        double cameraEShift = 0;
        double cameraDShift = 0;
        double lookAtNShift = 1;
        double lookAtEShift = 0;
        double lookAtDShift = 0;

        const QMultiMap<qint64, PostProcessingForm::Tag>* tags = nullptr;
        const QMap<qint64, PostProcessingForm::DistanceItem>* distances = nullptr;
        const PostProcessingForm::Rover* rovers = nullptr;
    };

    void GenerateMovieScript(const Params& params);

private:

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
};

}; // namespace Stylus

#endif // MOVIESCRIPTGENERATOR_H
