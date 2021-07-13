/*
    lidarscriptgenerator.h (part of GNSS-Stylus)
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

#ifndef LIDARSCRIPTGENERATOR_H
#define LIDARSCRIPTGENERATOR_H

#include "../postprocessingform.h"

namespace Lidar
{

class LidarScriptGenerator : public QObject
{
    Q_OBJECT
public:
    class Params
    {
    public:
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_BeforeRotation = nullptr;
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_AfterRotation = nullptr;
        QString fileName;
        QString tagIdent_BeginNewObject = "New object";
        QString tagIdent_BeginPoints = "RMB";
        QString tagIdent_EndPoints = "LMB";
        int timeShift = 0;
        const Eigen::Vector3d* boundingSphere_Center;
        double boundingSphere_Radius = 1e12;
        qint64 uptime_Min = 0;
        qint64 uptime_Max = 1e18;

        const QMultiMap<qint64, PostProcessingForm::Tag>* tags = nullptr;
        const PostProcessingForm::Rover* rovers = nullptr;
        const QMap<qint64, PostProcessingForm::LidarRound>* lidarRounds = nullptr;
        const RPLidarPlausibilityFilter::Settings* lidarFilteringSettings = nullptr;
        PostProcessingForm::LOInterpolator* loInterpolator = nullptr;
    };

    void generateLidarScript(const Params& params);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
};

}; // namespace Lidar

#endif // LIDARSCRIPTGENERATOR_H
