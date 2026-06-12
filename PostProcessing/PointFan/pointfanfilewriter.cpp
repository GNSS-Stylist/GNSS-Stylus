/*
    pointfanfilewriter.cpp (part of GNSS-Stylus)
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

#include <QDateTime>

#include "pointfanfilewriter.h"

PointFanFileWriter::PointFanFileWriter(const Params &params)
{
    this->params = params;
}

PointFanFileWriter::~PointFanFileWriter()
{
    if (fileOpenedSuccesfully)
    {
        finalizeFile();
    }
}

bool PointFanFileWriter::openFile(const QString &fileName)
{
    file.setFileName(fileName);

    if (file.open(QIODevice::WriteOnly))
    {
        fileOpenedSuccesfully = true;
        writeHeader();
        return true;
    }

    return false;
}

bool PointFanFileWriter::writeHeader(void)
{
    // Not bothering here with file format (ascii/floats/doubles etc).
    // Just use doubles for coords and float for normals
    // and use binary format.

    QByteArray eol = params.endOfLine;

    QByteArray dataToWrite = "ply" + eol;

    dataToWrite += "format binary_little_endian 1.0" + eol;

    dataToWrite += "comment File created with GNSS-Stylus on (dd.mm.yyyy hh:mm): " + QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm").toLatin1() + eol;

    // rows "comment pad" and "element vertex (N/A) added here this way to allow updating them later with simple overwriting some bytes.
    // Vertex count is not known when creating the file so it needs to be updated as one of the last steps when finalizing the file.
    // (Seems that Meshlab actually allows '0'-padded length, but the original paper doesn't say anything about padding, so better to be safe).
    dataToWrite += "comment pad";
    plyVertexCountFirstByte = dataToWrite.length();
    dataToWrite += "     " + eol + "element vertex (N/A)";
    plyVertexCountLastByte = dataToWrite.length();
    dataToWrite += eol;

    dataToWrite += "property double x" + eol;
    dataToWrite += "property double y" + eol;
    dataToWrite += "property double z" + eol;

    dataToWrite += "property float nx" + eol;
    dataToWrite += "property float ny" + eol;
    dataToWrite += "property float nz" + eol;

    if (params.writeQuality)
    {
        dataToWrite += "property float quality" + eol;
    }

    if (params.writeCorners || params.writeFaces)
    {
        // Read above ("element vertex") how writing of counts are supposed to work here
        dataToWrite += "comment pad";
        plyFaceCountFirstByte = dataToWrite.length();
        dataToWrite += "     " + eol + "element face (N/A)";
        plyFaceCountLastByte = dataToWrite.length();
        dataToWrite += eol;
        dataToWrite += "property list uchar int vertex_index" + eol;
    }

    dataToWrite += "end_header" + eol;

    file.write(dataToWrite);

    return true;
}

bool PointFanFileWriter::writePoint(const Eigen::Vector3d& point, const Eigen::Vector3d& normal)
{
    if (!fileOpenedSuccesfully)
    {
        return false;
    }

    writeDouble(point.x());
    writeDouble(point.y());
    writeDouble(point.z());

    writeFloat(normal.x());
    writeFloat(normal.y());
    writeFloat(normal.z());

    if (params.writeQuality)
    {
        writeFloat(params.qualityValue);
    }

    numberOfPointsWritten++;

    return true;
}

bool PointFanFileWriter::addTriangle(const FanTriangle& triangle)
{
    triangles.push_back(triangle);
    return true;
}

void PointFanFileWriter::finalizeFile(void)
{
    unsigned int triangleVerticesStartIndex = numberOfPointsWritten;

    if ((params.writeCorners) || (params.writeFaces))
    {
        // Add corner vertices
        for (auto& triangle:triangles)
        {
            for (int i = 0; i < 3; i++)
            {
                writePoint(triangle.getVertices()[i], triangle.getNormal());
            }
        }
    }

    if (params.writeFaces)
    {
        unsigned int triangleVertexIndex = triangleVerticesStartIndex;

        // Add triangles
        for (int triangleIndex = 0; triangleIndex < triangles.size(); triangleIndex++)
        {
            writeUChar(3);
            if (params.invertedFaces)
            {
                writeInt(triangleVertexIndex);
                writeInt(triangleVertexIndex + 2);
                writeInt(triangleVertexIndex + 1);
                triangleVertexIndex += 3;
            }
            else
            {
                writeInt(triangleVertexIndex++);
                writeInt(triangleVertexIndex++);
                writeInt(triangleVertexIndex++);
            }
        }
    }

    // Need to write number of vertices as one of the last steps as it isn't known before.
    unsigned int pointCount = numberOfPointsWritten;
    QByteArray stringToWrite = params.endOfLine + "element vertex " + QString::number(pointCount).toLatin1();

    int numOfBytesToWrite = plyVertexCountLastByte - plyVertexCountFirstByte;
    while (stringToWrite.length() < numOfBytesToWrite)
    {
        stringToWrite = QByteArray(" ") + stringToWrite;
    }

    file.seek(plyVertexCountFirstByte);
    file.write(stringToWrite);

    if (params.writeFaces)
    {
        // Need to write number of faces as one of the last steps as it isn't known before.
        unsigned int faceCount = triangles.size();
        stringToWrite = params.endOfLine + "element face " + QString::number(faceCount).toLatin1();

        numOfBytesToWrite = plyFaceCountLastByte - plyFaceCountFirstByte;
        while (stringToWrite.length() < numOfBytesToWrite)
        {
            stringToWrite = QByteArray(" ") + stringToWrite;
        }

        file.seek(plyFaceCountFirstByte);
        file.write(stringToWrite);
    }

    file.close();
    fileOpenedSuccesfully = false;
}

