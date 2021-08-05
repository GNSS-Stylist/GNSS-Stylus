/*
    meshlabrastercameragenerator.h (part of GNSS-Stylus)
    Copyright (C) 2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef MESHLABRASTERCAMERAGENERATOR_H
#define MESHLABRASTERCAMERAGENERATOR_H

#include <QStringList>
#include <QDir>

#include "postprocessingform.h"

class MeshLabRasterCameraGenerator : public QObject
{
    Q_OBJECT
public:
    class Params
    {
    public:
        const QStringList* lines;
        const Eigen::Transform<double, 3, Eigen::Affine>* transform_NEDToXYZ = nullptr;
        Eigen::Transform<double, 3, Eigen::Affine>* transform_Generated = nullptr;

        const PostProcessingForm::Rover* rovers = nullptr;
        PostProcessingForm::LOInterpolator* loInterpolator = nullptr;
    };

    class Item
    {
    public:
        int lineNumber = -1;
        int firstCol = -1;
        int lastCol = -1;
        QString text;

        Item(const QString& text, const int lineNumber = -1, const int firstCol = -1, const int lastCol = -1);
        Item() {};
    };

    class Issue
    {
    public:
        Item item;
        QString text;
    };

    MeshLabRasterCameraGenerator();

    QString generate(const Params& params);
    void init(void);

private:

    QString xmlRasterItemFormatString;

    QDir baseDir;
    QDir relativeDir;

    int timeShift;     //!< Time (ms) to add to one calculated from EXIF time differences when calculating location/orientation of the camera.

    QDateTime referenceImageDateTime;
    qint64 referenceImageUptime;
    UBXMessage_RELPOSNED::ITOW referenceImageITOW;

    QString outString;

    void processCommand(const QVector<Item>& command, const Params &params);
//    QVector<double> convertItemsToDoubles(const QVector<Item>& command, const unsigned int startItem, const unsigned int numOfItems);
    void checkArgumentCount(const QVector<Item>& command, const int argsNumMin, int argsNumMax = -1);
    QString fullPath(QString fileName = "");

    void genericPathCmd(const QVector<Item>& command, QDir& dir, QString pathTitle);
    void genericStringCmd(const QVector<Item>& command, QString& string);

    QDateTime readEXIFDateTimeFromString(QString dateTimeString);
    QDateTime readEXIFDateTimeFromFile(const QString fileName);

    void cmd_XMLWriteString(const QVector<Item>& command);
    void cmd_XMLRasterItemFormatString(const QVector<Item>& command);

    void cmd_BasePath(const QVector<Item>& command);
    void cmd_RelativePath(const QVector<Item>& command);

    void cmd_ReferenceTimeImage(const QVector<Item>& command);
    void cmd_ProcessStills(const QVector<Item>& command, const Params &params);
    void cmd_TimeShift(const QVector<Item>& command);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
};

#endif // MESHLABRASTERCAMERAGENERATOR_H
