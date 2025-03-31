/*
    pointfanfilewriter.h (part of GNSS-Stylus)
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

#ifndef POINTFANFILEWRITER_H
#define POINTFANFILEWRITER_H

#include <QByteArray>
#include <QFile>
#include <limits> // QtEndian (below) seems to need this... See https://bugreports.qt.io/browse/QTBUG-90395
#include <QtEndian>

#include "Eigen/Geometry"
#include "fantriangle.h"

class PointFanFileWriter
{
public:
    class Params
    {
    public:
        QByteArray endOfLine = "\n";
        bool writeCorners = false;
        bool writeFaces = false;
        bool invertedFaces = false;
    };

    PointFanFileWriter(const Params &params);
    ~PointFanFileWriter();
    bool openFile(const QString &fileName);
    bool writePoint(const Eigen::Vector3d& point, const Eigen::Vector3d& normal);
    bool addTriangle(const FanTriangle& triangle);
    void finalizeFile(void);
    bool getError(QString& errorString);

private:
    bool writeHeader(void);

    Params params;
    QFile file;
    bool fileOpenedSuccesfully = false;

    unsigned int numberOfPointsWritten = 0;

    QVector<FanTriangle> triangles;

    unsigned int plyVertexCountFirstByte = 0;
    unsigned int plyVertexCountLastByte = 0;

    unsigned int plyFaceCountFirstByte = 0;
    unsigned int plyFaceCountLastByte = 0;

    inline void writeDouble(const double src);
    inline void writeFloat(const float src);
    inline void writeInt(const int src);
    inline void writeUChar(const unsigned char src);
};

inline void PointFanFileWriter::writeDouble(const double src)
{
    char buf[8];
    qToLittleEndian(src, buf);
    file.write(buf, 8);
}

inline void PointFanFileWriter::writeFloat(const float src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

inline void PointFanFileWriter::writeInt(const int src)
{
    char buf[4];
    qToLittleEndian(src, buf);
    file.write(buf, 4);
}

inline void PointFanFileWriter::writeUChar(const unsigned char src)
{
    file.write((const char*)&src, 1);
}

#endif // POINTFANFILEWRITER_H
