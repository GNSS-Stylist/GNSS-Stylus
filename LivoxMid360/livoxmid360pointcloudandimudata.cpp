/*
    livoxmid360pointcloudandimudata.cpp (part of GNSS-Stylus)
    Copyright (C) 2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "livoxmid360pointcloudandimudata.h"
#include <QtEndian>
#include <QtMath>

namespace LivoxMid360
{


PointCloudAndIMUDataHeader::PointCloudAndIMUDataHeader()
{
    initHeaderFields(STATUS_UNINITIALIZED);
}

PointCloudAndIMUDataHeader::PointCloudAndIMUDataHeader(const QNetworkDatagram& datagram)
{
    ParseDatagramHeader(datagram);
}

bool PointCloudAndIMUDataHeader::ParseDatagramHeader(const QNetworkDatagram& datagram)
{
    if (datagram.data().size() < 36)
    {
        initHeaderFields(STATUS_ERROR_DATAGRAM_LENGTH_BELOW_MINIMUM);
        return false;
    }

    const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());

    quint8 protocolVersion = datagramData[0];
    if (protocolVersion != 0)
    {
        initHeaderFields(STATUS_ERROR_PROTOCOL_VERSION);
        return false;
    }

    int datagramSize = datagram.data().size();
    quint16 datagramLength_Datagram = qFromLittleEndian<quint16>(&datagramData[1]);

    if (datagramSize != datagramLength_Datagram)
    {
        initHeaderFields(STATUS_ERROR_DATAGRAM_LENGTH_MISMATCH);
        return false;
    }

    /* crc32 in the datagram (starting from byte 24) seems to be always 0?!?!? Uhhh...
    quint32 dataCRC32_Calc = fastCRC32.crc32(&datagramData[28], datagramLength_Datagram - 28);
    quint32 dataCRC32_Datagram = qFromLittleEndian<quint32>(&datagramData[24]);

    if (dataCRC32_Calc != dataCRC32_Datagram)
    {
        initHeaderFields(STATUS_ERROR_CRC);
        return false;
    }
    */

    time_interval = qFromLittleEndian<quint16>(&datagramData[3]);
    dot_num = qFromLittleEndian<quint16>(&datagramData[5]);
    udp_cnt = qFromLittleEndian<quint16>(&datagramData[7]);
    frame_cnt = datagramData[8];

    if (datagramData[10] > DATA_TYPE_POINTS_SPHERICAL)
    {
        initHeaderFields(STATUS_ERROR_UNKNOWN_DATA_TYPE);
        return false;
    }
    data_type = DataType(datagramData[10]);

    if (datagramData[11] > TIME_SYNC_GPS)
    {
        initHeaderFields(STATUS_ERROR_UNKNOWN_TIME_TYPE);
        return false;
    }
    time_type = TimeSyncType(datagramData[11]);

    timestamp = qFromLittleEndian<quint64>(&datagramData[28]);

    int itemLength;

    switch (data_type)
    {
    case DATA_TYPE_IMU:
        itemLength = 24;
        break;

    case DATA_TYPE_POINTS_CARTESIAN_32BIT:
        itemLength = 14;
        break;

    case DATA_TYPE_POINTS_CARTESIAN_16BIT:
        itemLength = 8;
        break;

    case DATA_TYPE_POINTS_SPHERICAL:
        itemLength = 10;
        break;

    default:
        // This should never be entered
        initHeaderFields(STATUS_ERROR_UNKNOWN_DATA_TYPE);
        return false;
    }

    int payloadLength = dot_num * itemLength;

    if ((payloadLength + 36) != datagramLength_Datagram)
    {
        initHeaderFields(STATUS_ERROR_DATAGRAM_LENGTH_MISMATCH);
        return false;
    }

    status = STATUS_VALID;
    return true;
}

void PointCloudAndIMUDataHeader::initHeaderFields(const PointCloudAndIMUDataHeader::MessageDataStatus newStatus)
{
    status = newStatus;
    time_interval = 0;
    dot_num = 0;
    udp_cnt = 0;
    frame_cnt = 0;
    data_type = DATA_TYPE_INVALID;
    time_type = TIME_SYNC_NONE;
    timestamp = 0;
}

PointCloudData::PointCloudData()
{
    initHeaderFields(STATUS_UNINITIALIZED);
}

PointCloudData::PointCloudData(const QNetworkDatagram& datagram)
{
    parseDatagram(datagram);
}

PointCloudData::PointCloudData(const PointCloudAndIMUDataHeader& header, const QNetworkDatagram& datagram) // Copies header fields without further error checking(!) and extracts points from the datagram.
{
    this->status = header.status;
    this->time_interval = header.time_interval;
    this->dot_num = header.dot_num;
    this->udp_cnt = header.udp_cnt;
    this->frame_cnt = header.frame_cnt;
    this->data_type = header.data_type;
    this->time_type = header.time_type;
    this->timestamp = header.timestamp;

    if ((data_type != DATA_TYPE_POINTS_CARTESIAN_32BIT) &&
         (data_type != DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
         (data_type != DATA_TYPE_POINTS_SPHERICAL))
    {
        status = STATUS_ERROR_SUBCLASS;
        return;
    }

    extractPoints(datagram);
}

bool PointCloudData::parseDatagram(const QNetworkDatagram& datagram)
{
    ParseDatagramHeader(datagram);

    if ((data_type != DATA_TYPE_POINTS_CARTESIAN_32BIT) &&
        (data_type != DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
        (data_type != DATA_TYPE_POINTS_SPHERICAL))
    {
        status = STATUS_ERROR_SUBCLASS;
        points.clear();
        return false;
    }

    return extractPoints(datagram);
}

bool PointCloudData::extractPoints(const QNetworkDatagram& datagram)
{
    points.clear();

    if ((status != STATUS_VALID) ||
        ((data_type != DATA_TYPE_POINTS_CARTESIAN_32BIT) &&
        (data_type != DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
        (data_type != DATA_TYPE_POINTS_SPHERICAL)))
    {
        return false;
    }

    // No other checks here, it is expected here that the length/type etc have been checked already!

    const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());

    switch (data_type)
    {
    case DATA_TYPE_POINTS_CARTESIAN_32BIT:
        extractPoints_Cartesian_32Bit(&datagramData[36], dot_num);
        break;

    case DATA_TYPE_POINTS_CARTESIAN_16BIT:
        extractPoints_Cartesian_16Bit(&datagramData[36], dot_num);
        break;

    case DATA_TYPE_POINTS_SPHERICAL:
        extractPoints_Spherical(&datagramData[36], dot_num);
        break;

    default:
        // This should never be entered
        initHeaderFields(STATUS_ERROR_UNKNOWN_DATA_TYPE);
        return false;
    }

    return true;
}

void PointCloudData::extractPoints_Cartesian_32Bit(const unsigned char* const data, const int numOfPoints)
{
    Point pointToAdd;
    const int itemLength = 14;

    for (int i = 0; i < numOfPoints; i++)
    {
        int startOffset = i * itemLength;

        qint32 temp32 = qFromLittleEndian<qint32>(&data[startOffset]);
        pointToAdd.x = temp32 * 0.001;

        temp32 = qFromLittleEndian<qint32>(&data[startOffset + 4]);
        pointToAdd.y = temp32 * 0.001;

        temp32 = qFromLittleEndian<qint32>(&data[startOffset +8]);
        pointToAdd.z = temp32 * 0.001;

        pointToAdd.reflectivity = data[startOffset + 12];
        pointToAdd.Properties_other = Confidence((data[startOffset + 13] >> 4) & 3);
        pointToAdd.Properties_dust = Confidence((data[startOffset + 13] >> 2) & 3);
        pointToAdd.Properties_glue = Confidence((data[startOffset + 13] >> 0) & 3);

        points.push_back(pointToAdd);
    }
}

void PointCloudData::extractPoints_Cartesian_16Bit(const unsigned char* const data, const int numOfPoints)
{
    Point pointToAdd;
    const int itemLength = 8;

    for (int i = 0; i < numOfPoints; i++)
    {
        int startOffset = i * itemLength;

        qint16 temp16 = qFromLittleEndian<qint16>(&data[startOffset]);
        pointToAdd.x = temp16 * 0.01;

        temp16 = qFromLittleEndian<qint16>(&data[startOffset + 2]);
        pointToAdd.y = temp16 * 0.01;

        temp16 = qFromLittleEndian<qint16>(&data[startOffset + 4]);
        pointToAdd.z = temp16 * 0.01;

        pointToAdd.reflectivity = data[startOffset + 6];
        pointToAdd.Properties_other = Confidence((data[startOffset + 7] >> 4) & 3);
        pointToAdd.Properties_dust = Confidence((data[startOffset + 7] >> 2) & 3);
        pointToAdd.Properties_glue = Confidence((data[startOffset +7] >> 0) & 3);

        points.push_back(pointToAdd);
    }
}

void PointCloudData::extractPoints_Spherical(const unsigned char* const data, const int numOfPoints)
{
    Point pointToAdd;
    const int itemLength = 10;

    for (int i = 0; i < numOfPoints; i++)
    {
        int startOffset = i * itemLength;

        quint32 rawDepth = qFromLittleEndian<quint32>(&data[startOffset]);
        quint16 rawTheta = qFromLittleEndian<quint16>(&data[startOffset + 4]);
        quint16 rawPhi = qFromLittleEndian<quint16>(&data[startOffset + 4]);

        double depth = rawDepth * 0.001;
        double theta = qDegreesToRadians(rawTheta * 0.01);
        double phi = qDegreesToRadians(rawPhi * 0.01);

        pointToAdd.x = depth * sin(theta) * cos(phi);
        pointToAdd.y = depth * sin(theta) * sin(phi);
        pointToAdd.z = depth * cos(theta);

        pointToAdd.reflectivity = data[startOffset + 8];
        pointToAdd.Properties_other = Confidence((data[startOffset + 9] >> 4) & 3);
        pointToAdd.Properties_dust = Confidence((data[startOffset + 9] >> 2) & 3);
        pointToAdd.Properties_glue = Confidence((data[startOffset +9] >> 0) & 3);

        points.push_back(pointToAdd);
    }
}


























IMUData::IMUData()
{
    initHeaderFields(STATUS_UNINITIALIZED);
    initIMUFields();
}

IMUData::IMUData(const QNetworkDatagram& datagram)
{
    parseDatagram(datagram);
}

IMUData::IMUData(const PointCloudAndIMUDataHeader& header, const QNetworkDatagram& datagram) // Copies header fields without further error checking(!) and extracts points from the datagram.
{
    this->status = header.status;
    this->time_interval = header.time_interval;
    this->dot_num = header.dot_num;
    this->udp_cnt = header.udp_cnt;
    this->frame_cnt = header.frame_cnt;
    this->data_type = header.data_type;
    this->time_type = header.time_type;
    this->timestamp = header.timestamp;

    if (data_type != DATA_TYPE_IMU)
    {
        status = STATUS_ERROR_SUBCLASS;
        initIMUFields();
        return;
    }

    extractIMUData(datagram);
}

bool IMUData::parseDatagram(const QNetworkDatagram& datagram)
{
    ParseDatagramHeader(datagram);

    if (data_type != DATA_TYPE_IMU)
    {
        status = STATUS_ERROR_SUBCLASS;
        initIMUFields();
        return false;
    }

    return extractIMUData(datagram);
}

bool IMUData::extractIMUData(const QNetworkDatagram& datagram)
{
    const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());

    const int offset = 36;

    imuData.gyro_x = qFromLittleEndian<float>(&datagramData[offset]);
    imuData.gyro_y = qFromLittleEndian<float>(&datagramData[offset + 4]);
    imuData.gyro_z = qFromLittleEndian<float>(&datagramData[offset + 8]);

    imuData.acc_x = qFromLittleEndian<float>(&datagramData[offset + 12]);
    imuData.acc_y = qFromLittleEndian<float>(&datagramData[offset + 16]);
    imuData.acc_z = qFromLittleEndian<float>(&datagramData[offset + 20]);

    return true;
}

void IMUData::initIMUFields(void)
{
    imuData.gyro_x = 0;
    imuData.gyro_y = 0;
    imuData.gyro_z = 0;

    imuData.acc_x = 0;
    imuData.acc_y = 0;
    imuData.acc_z = 0;
}

} // namespace LivoxMid360
