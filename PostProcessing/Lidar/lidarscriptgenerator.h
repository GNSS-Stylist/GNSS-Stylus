/*
    lidarscriptgenerator.h (part of GNSS-Stylus)
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

#ifndef LIDARSCRIPTGENERATOR_H
#define LIDARSCRIPTGENERATOR_H

#include "../postprocessingform.h"
#include "../lointerpolator.h"
#include "lidarscriptgeneratorthread.h"
#include "../asynclidarscriptfilewriter.h"

namespace Lidar
{

class LidarScriptGenerator : public QObject
{
    Q_OBJECT
public:
    class Params
    {
    public:
        QString baseFileName;
        bool dontWriteFiles = false;
        qint64 uptime_Min = 0;
        qint64 uptime_Max = 1e18;

        int maxWorkUnitDuration = 1000;
        int numOfWorkerThreads = 1;

        LidarScriptGeneratorThread::ConstData threadConstData;
        AsyncLidarScriptFileWriter::Params fileParams;
    };

    void generateLidarScript(const Params& params);

private:
    UBXMessage_RELPOSNED::ITOW getITOW(const QMap<qint64, UBXMessage_RELPOSNED::ITOW>* averagedSync, const quint64 &uptime_ms);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
};

}; // namespace Lidar

#endif // LIDARSCRIPTGENERATOR_H
