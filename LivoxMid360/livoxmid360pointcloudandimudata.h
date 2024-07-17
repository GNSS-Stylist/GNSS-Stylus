/*
    livoxmid360pointcloudandimudata.h (part of GNSS-Stylus)
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

#ifndef LIVOXMID360POINTCLOUDANDIMUDATA_H
#define LIVOXMID360POINTCLOUDANDIMUDATA_H

#include <QNetworkDatagram>
#include "FastCRC/FastCRC.h"

namespace LivoxMid360
{

class PointCloudAndIMUDataHeader
{
public:
    typedef enum
    {
        STATUS_UNINITIALIZED = -1,
        STATUS_INVALID = 0,
        STATUS_VALID = 1,

        STATUS_ERROR_DATAGRAM_LENGTH_BELOW_MINIMUM = 100, // Datagram length below 36 bytes
        STATUS_ERROR_DATAGRAM_LENGTH_MISMATCH, // Length in header does not match datagram length

        STATUS_ERROR_CRC = 200,     // CRC mismatch (crc32)

        STATUS_ERROR_PROTOCOL_VERSION = 300,// Protocol version is not supported
        STATUS_ERROR_UNKNOWN_DATA_TYPE,     // Data type is not supported (should be point or IMU)
        STATUS_ERROR_UNKNOWN_TIME_TYPE,     // Timestamp type is not supported
        STATUS_ERROR_DOT_NUM_MISMATCH,      // dot_num (along with the type) does not match datagram length

        // SubClass (Point/IMU) errors:
        STATUS_ERROR_SUBCLASS = 400,        // Tried to "cast" IMU data to points or vice versa

    } MessageDataStatus;

    typedef enum
    {
        DATA_TYPE_INVALID = 0xFF,
        DATA_TYPE_IMU = 0,
        DATA_TYPE_POINTS_CARTESIAN_32BIT,
        DATA_TYPE_POINTS_CARTESIAN_16BIT,
        DATA_TYPE_POINTS_SPHERICAL,
    } DataType;

    typedef enum
    {
        TIME_SYNC_NONE = 0,
        TIME_SYNC_PTP = 1,
        TIME_SYNC_GPS = 2
    } TimeSyncType;

    // Naming here follows the MID-360 communication protocol definitions
    // ( https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html )
    MessageDataStatus status;
    quint16 time_interval;
    quint16 dot_num;
    quint16 udp_cnt;
    quint8 frame_cnt;
    DataType data_type;
    TimeSyncType time_type;
    quint64 timestamp;

    PointCloudAndIMUDataHeader();
    PointCloudAndIMUDataHeader(const QNetworkDatagram& datagram);
    bool ParseDatagramHeader(const QNetworkDatagram& datagram);

protected:
    void initHeaderFields(const PointCloudAndIMUDataHeader::MessageDataStatus newStatus = STATUS_INVALID);
    FastCRC32 fastCRC32;
};

class PointCloudData : public PointCloudAndIMUDataHeader
{
    public:

    typedef enum
    {
        CONFIDENCE_HIGH = 0,
        CONFIDENCE_MEDIUM,
        CONFIDENCE_LOW,
        CONFIDENCE_RESERVED
    } Confidence;

    class Point
    {
    public:
        float x;    // metres
        float y;    // metres
        float z;    // metres
        quint8 reflectivity;
        quint8 properties;
        Confidence getProperties_other(void) { return Confidence((properties >> 4) & 3); };
        Confidence getProperties_dust(void) { return Confidence((properties >> 2) & 3); };
        Confidence getProperties_glue(void) { return Confidence((properties >> 0) & 3); };
    };

    QVector<Point> points;

    PointCloudData();
    PointCloudData(const QNetworkDatagram& datagram);
    PointCloudData(const PointCloudAndIMUDataHeader& header, const QNetworkDatagram& datagram); // Copies header fields without further error checking(!) and extracts points from the datagram.
    bool parseDatagram(const QNetworkDatagram& datagram);

private:
    bool extractPoints(const QNetworkDatagram& datagram);
    void extractPoints_Cartesian_32Bit(const unsigned char* const data, const int numOfPoints);
    void extractPoints_Cartesian_16Bit(const unsigned char* const data, const int numOfPoints);
    void extractPoints_Spherical(const unsigned char* const data, const int numOfPoints);
};

class IMUData : public PointCloudAndIMUDataHeader
{
public:
    class IMU
    {
    public:
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
    };

    IMU imuData;

    IMUData();
    IMUData(const QNetworkDatagram& datagram);
    IMUData(const PointCloudAndIMUDataHeader& header, const QNetworkDatagram& datagram); // Copies header fields without further error checking(!) and extracts IMU data from the datagram.
    bool parseDatagram(const QNetworkDatagram& datagram);

private:
    void initIMUFields(void);
    bool extractIMUData(const QNetworkDatagram& datagram);
};

} // namespace LivoxMid360

#endif // LIVOXMID360POINTCLOUDANDIMUDATA_H
