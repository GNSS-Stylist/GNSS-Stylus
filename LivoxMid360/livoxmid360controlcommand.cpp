/*
    livoxmid360controlcommand.cpp (part of GNSS-Stylus)
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

#include "livoxmid360controlcommand.h"
#include <QtEndian>


namespace LivoxMid360
{

ControlCommand::ControlCommand()
{
    initHeaderFields(STATUS_UNINITIALIZED);
}

ControlCommand::ControlCommand(const QNetworkDatagram& datagram)
{
    parseDatagramHeader(datagram);
}

void ControlCommand::initHeaderFields(const ControlCommand::MessageDataStatus newStatus)
{
    status = newStatus;
    seq_num = 0;
    cmd_id = 0;
    cmd_type = CMD_TYPE_INVALID;
    sender_type = SENDER_TYPE_INVALID;
}

bool ControlCommand::parseDatagramHeader(const QNetworkDatagram& datagram)
{
    if (datagram.data().size() < 24)
    {
        initHeaderFields(STATUS_ERROR_DATAGRAM_LENGTH_BELOW_MINIMUM);
        return false;
    }

    const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());
    int datagramSize = datagram.data().size();

    quint16 headerCRC16_Calc = fastCRC16.ccitt(datagramData, 18);
    quint16 headerCRC16_Datagram = qFromLittleEndian<quint16>(&datagramData[18]);

    if (headerCRC16_Calc != headerCRC16_Datagram)
    {
        initHeaderFields(STATUS_ERROR_CRC16);
        return false;
    }

    quint16 datagramLength_Datagram = qFromLittleEndian<quint16>(&datagramData[2]);

    if (datagramSize != datagramLength_Datagram)
    {
        initHeaderFields(STATUS_ERROR_DATAGRAM_LENGTH_MISMATCH);
        return false;
    }

    quint32 dataCRC32_Calc = fastCRC32.crc32(&datagramData[24], datagramLength_Datagram - 24);
    quint32 dataCRC32_Datagram = qFromLittleEndian<quint32>(&datagramData[20]);

    if (dataCRC32_Calc != dataCRC32_Datagram)
    {
        initHeaderFields(STATUS_ERROR_CRC32);
        return false;
    }

    seq_num = qFromLittleEndian<quint32>(&datagramData[4]);
    cmd_id = qFromLittleEndian<quint16>(&datagramData[8]);

    quint8 temp8 = datagramData[10];

    //if ((temp8 < CMD_TYPE_REQ) || (temp8 > CMD_TYPE_ACK))
    if (temp8 > CMD_TYPE_ACK)
    {
        initHeaderFields(STATUS_ERROR_CMD_TYPE);
        return false;
    }

    cmd_type = CmdType(temp8);

    temp8 = datagramData[11];

    // if ((temp8 < SENDER_TYPE_HOST) || (temp8 > SENDER_TYPE_LIDAR))
    if (temp8 > SENDER_TYPE_LIDAR)
    {
        initHeaderFields(STATUS_ERROR_SENDER_TYPE);
        return false;
    }

    sender_type = SenderType(temp8);

    status = STATUS_VALID;
    return true;
}

PushLidarInformation::PushLidarInformation()
{
    initKeyValidities();
}

void PushLidarInformation::initKeyValidities(void)
{
    pcl_data_type_valid = false;
    lidar_ipcfg_valid = false;
    state_info_host_ipcfg_valid = false;
    pointcloud_host_ipcfg_valid = false;
    imu_host_ipcfg_valid = false;
    detect_mode_valid = false;
    imu_data_en_valid = false;
    sn_valid = false;
    cur_work_state_valid = false;
    core_temp_valid = false;
    local_time_now_valid = false;
    last_sync_time_valid = false;
    time_offset_valid = false;
    time_sync_type_valid = false;
}

PushLidarInformation::PushLidarInformation(const QNetworkDatagram& datagram)
{
    parseDatagram(datagram);
}

bool PushLidarInformation::parseDatagram(const QNetworkDatagram& datagram)
{
    initKeyValidities();

    if (!parseDatagramHeader(datagram))
    {
        return false;
    }

    if (cmd_id != PUSH_LIDAR_INFORMATION)
    {
        status = STATUS_ERROR_CMD_ID;
        return false;
    }

    const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());
    int datagramSize = datagram.data().size();

    int i = 24;

    while (i < datagramSize)
    {
        if (datagramSize < i + 4)
        {
            status = STATUS_ERROR_PUSH_INFO_PREMATURE_END_OF_DATA;
            // initKeyValidities(); Leave parsed keys
            return false;
        }

        quint16 key = qFromLittleEndian<quint16>(&datagramData[i]);
        quint16 keyLength = qFromLittleEndian<quint16>(&datagramData[i + 2]);
        i += 4;

        if (datagramSize < i + keyLength)
        {
            status = STATUS_ERROR_PUSH_INFO_PREMATURE_END_OF_DATA;
            // initKeyValidities(); Leave parsed keys
            return false;
        }

        switch(key)
        {
        case 0x0000:    // pcl_data_type
        {
            quint8 temp8 = datagramData[i];

            if ((temp8 < PCL_DATA_TYPE_CARTESIAN_32BIT) || (temp8 > PCL_DATA_TYPE_SPHERICAL))
            {
                status = STATUS_ERROR_PUSH_INFO_PCL_DATA_TYPE_INVALID;
                return false;
            }

            pcl_data_type = PclDataType(temp8);
            pcl_data_type_valid = true;

            break;
        }

        case 0x0004:    // lidar_ipcfg
        {
            for (int ii = 0; ii < 12; ii++)
            {
                lidar_ipcfg[ii] = datagramData[i + ii];
            }
            lidar_ipcfg_valid = true;

            break;
        }

        case 0x0005:    // state_info_host_ipcfg
        {
            for (int ii = 0; ii < 8; ii++)
            {
                state_info_host_ipcfg[ii] = datagramData[i + ii];
            }
            state_info_host_ipcfg_valid = true;

            break;
        }

        case 0x0006:    // pointcloud_host_ipcfg
        {
            for (int ii = 0; ii < 8; ii++)
            {
                pointcloud_host_ipcfg[ii] = datagramData[i + ii];
            }
            pointcloud_host_ipcfg_valid = true;

            break;
        }

        case 0x0007:    // imu_host_ipcfg
        {
            for (int ii = 0; ii < 8; ii++)
            {
                imu_host_ipcfg[ii] = datagramData[i + ii];
            }
            imu_host_ipcfg_valid = true;

            break;
        }

        case 0x0018:    // detect_mode
        {
            quint8 temp8 = datagramData[i];

            // if ((temp8 < DETECT_MODE_NORMAL) || (temp8 > DETECT_MODE_SENSITIVE))
            if (temp8 > DETECT_MODE_SENSITIVE)
            {
                status = STATUS_ERROR_PUSH_INFO_DETECT_MODE_INVALID;
                return false;
            }

            detect_mode = DetectMode(temp8);
            detect_mode_valid = true;

            break;
        }

        case 0x001C:    // imu_data_en
        {
           imu_data_en = datagramData[i];
           imu_data_en_valid = true;

           break;
        }

        case 0x8000:    // sn
        {
            char tempSN[17];
            tempSN[16] = 0;

            for (int ii = 0; ii < 16; ii++)
            {
                tempSN[ii] = datagramData[i + ii];
            }

            sn = tempSN;
            sn_valid = true;

            break;
        }

        case 0x8006:    // cur_work_state
        {
            quint8 temp8 = datagramData[i];

            if ((temp8 != WORK_STATE_SAMPLING) &&
                (temp8 != WORK_STATE_IDLE) &&
                (temp8 != WORK_STATE_ERROR) &&
                (temp8 != WORK_STATE_SELFCHECK) &&
                (temp8 != WORK_STATE_MOTORSTARUP) &&
                (temp8 != WORK_STATE_UPGRADE) &&
                (temp8 != WORK_STATE_READY))
            {
                status = STATUS_ERROR_PUSH_INFO_CUR_WORK_STATE_INVALID;
                return false;
            }

            cur_work_state = WorkState(temp8);
            cur_work_state_valid = true;

            break;
        }

        case 0x8007:    // core_temp
        {
            core_temp = qFromLittleEndian<quint16>(&datagramData[i]);
            core_temp_valid = true;
            break;
        }

        case 0x8009:    // local_time_now
        {
            local_time_now = qFromLittleEndian<quint64>(&datagramData[i]);
            local_time_now_valid = true;
            break;
        }

        case 0x800A:    // last_sync_time
        {
            last_sync_time = qFromLittleEndian<quint64>(&datagramData[i]);
            last_sync_time_valid = true;
            break;
        }

        case 0x800B:    // time_offset
        {
            time_offset = qFromLittleEndian<qint64>(&datagramData[i]);
            time_offset_valid = true;
            break;
        }

        case 0x800C:    // time_sync_type
        {
            quint8 temp8 = datagramData[i];

            // if ((temp8 < TIME_SYNC_NONE) || (temp8 > TIME_SYNC_GPS))
            if (temp8 > TIME_SYNC_GPS)
            {
                status = STATUS_ERROR_PUSH_INFO_TIME_SYNC_TYPE_INVALID;
                return false;
            }

            time_sync_type = TimeSyncType(temp8);
            time_sync_type_valid = true;

            break;
        }

        default:
            // Just skip unimplemented key types
            break;


        } // switch(key)

        i += keyLength;
    }

    return true;
}

#if 1
QByteArray SetGPSTimestamp::createSetCommand(const quint64 time, const quint32 seq_num)
{
    unsigned char buf[24 + sizeof(quint8) + sizeof(quint64)];

    buf[0] = 0xAA;
    buf[1] = 0;

    quint16 length = sizeof(buf);

    *((quint16*)&buf[2]) = qToLittleEndian(length);
    *((quint32*)&buf[4]) = qToLittleEndian(seq_num);
    *((quint16*)&buf[8]) = qToLittleEndian(quint16(SET_GPS_TIMESTAMP));
    buf[10] = 0;    // REQ
    buf[11] = 0;    // Sender = host computer

    for (int i = 12; i < 18; i++)
    {
        buf[i] = 0;
    }

    FastCRC16 localFastCRC16;
    quint16 headerCRC16_Calc = localFastCRC16.ccitt(buf, 18);
    *((quint16*)&buf[18]) = qToLittleEndian(headerCRC16_Calc);

    buf[24] = 2;    // GPS time synchronization
    *((quint64*)&buf[25]) = qToLittleEndian(time);

    FastCRC32 localFastCRC32;
    quint32 dataCRC32_Calc = localFastCRC32.crc32(&buf[24], sizeof(buf) - 24);

    *((quint32*)&buf[20]) = qToLittleEndian(dataCRC32_Calc);

    QByteArray ret((char *)buf, sizeof(buf));
    return ret;
//    return QByteArray((char *)buf, sizeof(buf));
}
#endif

#if 0
QByteArray SetGPSTimestamp::createSetCommand(const quint64 time, const quint32 seq_num)
{
//    unsigned char buf[24 + sizeof(quint8) + sizeof(quint64)];
    unsigned char buf[24];

    buf[0] = 0xAA;
    buf[1] = 0;

    quint16 length = sizeof(buf);

    *((quint16*)&buf[2]) = qToLittleEndian(length);
    *((quint32*)&buf[4]) = qToLittleEndian(seq_num);
    *((quint16*)&buf[8]) = qToLittleEndian(quint16(SET_GPS_TIMESTAMP));
    buf[10] = 0;    // REQ
    buf[11] = 0;    // Sender = host computer

    for (int i = 12; i < 18; i++)
    {
        buf[i] = 0;
    }

    FastCRC16 localFastCRC16;
    quint16 headerCRC16_Calc = localFastCRC16.ccitt(buf, 18);
    *((quint16*)&buf[18]) = qToLittleEndian(headerCRC16_Calc);

    buf[24] = 2;    // GPS time synchronization
    *((quint64*)&buf[25]) = qToLittleEndian(time);

    FastCRC32 localFastCRC32;
    quint32 dataCRC32_Calc = localFastCRC32.crc32(&buf[24], sizeof(buf) - 24);

    *((quint32*)&buf[20]) = qToLittleEndian(dataCRC32_Calc);

    QByteArray ret((char *)buf, sizeof(buf));
    return ret;
    //    return QByteArray((char *)buf, sizeof(buf));
}
#endif

} // namespace








