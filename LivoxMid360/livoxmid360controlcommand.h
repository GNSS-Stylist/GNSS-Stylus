/*
    livoxmid360controlcommand.h (part of GNSS-Stylus)
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

#ifndef LIVOXMID360CONTROLCOMMAND_H
#define LIVOXMID360CONTROLCOMMAND_H

#include <QNetworkDatagram>
#include "FastCRC/FastCRC.h"

namespace LivoxMid360
{

class ControlCommand
{
public:
    typedef enum
    {
        STATUS_UNINITIALIZED = -1,
        STATUS_INVALID = 0,
        STATUS_VALID = 1,

        STATUS_ERROR_DATAGRAM_LENGTH_BELOW_MINIMUM = 100, // Datagram length below 24 bytes
        STATUS_ERROR_DATAGRAM_LENGTH_MISMATCH, // Length in header does not match datagram length

        STATUS_ERROR_CRC16 = 200,   // Frame header CRC mismatch
        STATUS_ERROR_CRC32,         // Frame data CRC mismatch

        STATUS_ERROR_STARTING_BYTE = 300,   // Datagram starting byte is not 0xAA
        STATUS_ERROR_PROTOCOL_VERSION,      // Protocol version is not supported
        STATUS_ERROR_CMD_TYPE,              // cmd_type is not supported (should be REQ/ACK)
        STATUS_ERROR_SENDER_TYPE,           // Sender type is not host computer or lidar

        STATUS_ERROR_CMD_ID = 400,          // cmd_id doesn't match when casting to a subclass (like LivoxMid360::PushInfo)

        // Push info:
        STATUS_ERROR_PUSH_INFO_PREMATURE_END_OF_DATA = 500,   // Data extended over the end of the datagram when parsing the keys
        STATUS_ERROR_PUSH_INFO_PCL_DATA_TYPE_INVALID,
        STATUS_ERROR_PUSH_INFO_DETECT_MODE_INVALID,
        STATUS_ERROR_PUSH_INFO_CUR_WORK_STATE_INVALID,
        STATUS_ERROR_PUSH_INFO_TIME_SYNC_TYPE_INVALID,

    } MessageDataStatus;

    typedef enum
    {
        // Only cmd_ids supported defined here
        PUSH_LIDAR_INFORMATION = 0x0102,
        SET_GPS_TIMESTAMP = 0x0202
    } Cmd_id;

    typedef enum
    {
        CMD_TYPE_INVALID = 0xFF,
        CMD_TYPE_REQ = 0x00,
        CMD_TYPE_ACK = 0x01,
    } CmdType;

    typedef enum
    {
        SENDER_TYPE_INVALID = 0xFF,
        SENDER_TYPE_HOST = 0x00,
        SENDER_TYPE_LIDAR = 0x01,
    } SenderType;

    ControlCommand();
    ControlCommand(const QNetworkDatagram& datagram);
    bool parseDatagramHeader(const QNetworkDatagram& datagram);

    // Naming here follows the MID-360 communication protocol definitions
    // ( https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html )
    MessageDataStatus status;
    quint32 seq_num;
    quint16 cmd_id;
    CmdType cmd_type;
    SenderType sender_type;

protected:
    void initHeaderFields(const ControlCommand::MessageDataStatus newStatus = STATUS_INVALID);
    FastCRC16 fastCRC16;
    FastCRC32 fastCRC32;
};

class PushLidarInformation : public ControlCommand
{
public:
    PushLidarInformation();
    PushLidarInformation(const QNetworkDatagram& datagram);
    bool parseDatagram(const QNetworkDatagram& datagram);

    typedef enum
    {
        PCL_DATA_TYPE_CARTESIAN_32BIT = 0x01,
        PCL_DATA_TYPE_CARTESIAN_16BIT = 0x02,
        PCL_DATA_TYPE_SPHERICAL = 0x03,
    } PclDataType;

    typedef enum
    {
        DETECT_MODE_NORMAL = 0x00,
        DETECT_MODE_SENSITIVE = 0x01
    } DetectMode;

    typedef enum
    {
        WORK_STATE_SAMPLING = 0x01,
        WORK_STATE_IDLE = 0x02,
        WORK_STATE_ERROR = 0x04,
        WORK_STATE_SELFCHECK = 0x05,
        WORK_STATE_MOTORSTARUP = 0x06,
        WORK_STATE_UPGRADE = 0x08,
        WORK_STATE_READY = 0x09,
    } WorkState;

    typedef enum
    {
        TIME_SYNC_NONE = 0,
        TIME_SYNC_PTP = 1,
        TIME_SYNC_GPS = 2
    } TimeSyncType;


    // Naming here follows the MID-360 communication protocol definitions
    // ( https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html )
    // Also only keys required somewhere are added here, so:
    // TODO: Add keys as needed.

    bool pcl_data_type_valid;
    PclDataType pcl_data_type;

    bool lidar_ipcfg_valid;
    char lidar_ipcfg[12];

    bool state_info_host_ipcfg_valid;
    char state_info_host_ipcfg[8];

    bool pointcloud_host_ipcfg_valid;
    char pointcloud_host_ipcfg[8];

    bool imu_host_ipcfg_valid;
    char imu_host_ipcfg[8];

    bool detect_mode_valid;
    DetectMode detect_mode;

    bool imu_data_en_valid;
    bool imu_data_en;

    bool sn_valid;
    QString sn;

    bool cur_work_state_valid;
    WorkState cur_work_state;

    bool core_temp_valid;
    qint32 core_temp;   // 0.01 deg C

    bool local_time_now_valid;
    quint64 local_time_now;

    bool last_sync_time_valid;
    quint64 last_sync_time;

    bool time_offset_valid;
    qint64 time_offset;

    bool time_sync_type_valid;
    TimeSyncType  time_sync_type;

private:
    void initKeyValidities(void);
};

class SetGPSTimestamp : public ControlCommand
{
public:
    static QByteArray createSetCommand(const quint64 time, const quint32 seq_num);

    // TODO: Add ACK-handler
};


} // Namespace

Q_DECLARE_METATYPE(LivoxMid360::ControlCommand);
Q_DECLARE_METATYPE(LivoxMid360::PushLidarInformation);

#endif // LIVOXMID360CONTROLCOMMAND_H
