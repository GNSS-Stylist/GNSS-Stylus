/*
    loscriptgenerator.h (part of GNSS-Stylus)
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

        enum TimeStampFormat
        {
            TSF_ITOW = 0,
            TSF_UPTIME
        };

    public:
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        Eigen::Transform<double, 3, Eigen::Affine>* transform_Generated = nullptr;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Min = 0;
        UBXMessage_RELPOSNED::ITOW iTOWRange_Script_Max = 1e9;

        QString fileName;
        TimeStampFormat timeStampFormat = TSF_ITOW;
        LOSolver* loSolver;

        const PostProcessingForm::Rover* rovers = nullptr;
    };

    void generateScript(const Params& params);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

};


#endif // LOSCRIPTGENERATOR_H
