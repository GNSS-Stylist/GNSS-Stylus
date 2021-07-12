/*
    pointcloudgeneratorstylus.h (part of GNSS-Stylus)
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

#ifndef POINTCLOUDGENERATORSTYLUS_H
#define POINTCLOUDGENERATORSTYLUS_H

#include "../postprocessingform.h"


namespace Stylus
{

class PointCloudGenerator : public QObject
{
    Q_OBJECT

public:
    class Params
    {
    public:
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        QDir directory;
        QString tagIdent_BeginNewObject = "New object";
        QString tagIdent_BeginPoints = "RMB";
        QString tagIdent_EndPoints = "LMB";
        double initialStylusTipDistanceFromRoverA = 0;
        bool includeNormals = false;

        const QMultiMap<qint64, PostProcessingForm::Tag>* tags = nullptr;
        const QMap<qint64, PostProcessingForm::DistanceItem>* distances = nullptr;
        const PostProcessingForm::Rover* rovers = nullptr;

    };

    void generatePointClouds(const Params& params);

private:
    bool generatePointCloudPointSet(const Params& params,
                                    const PostProcessingForm::Tag& beginningTag,
                                    const PostProcessingForm::Tag& endingTag,
                                    const qint64 beginningUptime, const qint64 endingUptime,
                                    QTextStream* outStream,
                                    int& pointsWritten);

public:

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

};

}; // namespace Stylus

#endif // POINTCLOUDGENERATORSTYLUS_H
