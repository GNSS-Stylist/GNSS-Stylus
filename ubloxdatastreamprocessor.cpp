/*
    ubloxdatastreamprocessor.cpp (part of GNSS-Stylus)
    Copyright (C) 2019 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

/**
 * @file ubloxdatastreamprocessor.cpp
 * @brief Definitions for a class separating different types of messages (NMEA, UBX, RTCM) sent by u-blox devices from a stream of bytes.
 */

#include "ubloxdatastreamprocessor.h"

UBloxDataStreamProcessor::UBloxDataStreamProcessor(const unsigned int maxUBXMessageLength,
                                                   const unsigned int maxNMEASentenceLenght,
                                                   const unsigned int maxUnidentifiedDataSize)
{
    this->maxUBXMessageLength = maxUBXMessageLength;
    this->maxNMEASentenceLenght = maxNMEASentenceLenght;
    this->maxUnidentifiedDataSize = maxUnidentifiedDataSize;
//    inputBuffer.reserve(1024);
    inputBuffer.clear();
    state = WAITING_FOR_START_BYTE;
}

void UBloxDataStreamProcessor::process(const char inbyte)
{
    switch (state)
    {
    case WAITING_FOR_START_BYTE:

        if ((inbyte == '$') || (static_cast<unsigned char>(inbyte) == 0xB5) || (static_cast<unsigned char>(inbyte) == 0xD3))
        {
            if (inputBuffer.length() != 0)
            {
                // inputBuffer should be empty at this point if all bytes can be interpreted as valid ones.
                emit unidentifiedDataReceived(inputBuffer);
            }
            inputBuffer.clear();
            inputBuffer.append(inbyte);
        }
        else if (static_cast<unsigned int>(inputBuffer.length()) >= maxUnidentifiedDataSize)
        {
            emit unidentifiedDataReceived(inputBuffer);
            inputBuffer.clear();
        }

        if (inbyte == '$')
        {
            // '$' = NMEA's start-character
            state = NMEA_WAITING_FOR_CR;
        }
        else if (static_cast<unsigned char>(inbyte) == 0xB5)
        {
            // 0xB5 = UBX's start-character
            state = UBX_WAITING_FOR_SYNC_CHAR_2;
        }
        else if (static_cast<unsigned char>(inbyte) == 0xD3)
        {
            // 0xD3 = RTCM's start-character
            state = RTCM_WAITING_FOR_MESSAGE_LENGTH_1;
        }
        else
        {
            // Byte not recognized as a start character of any supported message types
            // -> Append it into inputBuffer (will be emitted as unidentified data when recognized data is received)
            inputBuffer.append(inbyte);
        }
        break;

    case UBX_WAITING_FOR_SYNC_CHAR_2:
        if (static_cast<unsigned char>(inbyte) == 0x62)
        {
            inputBuffer.append(inbyte);
            state = UBX_WAITING_FOR_MESSAGE_CLASS;
        }
        else
        {
            emit ubxParseError("No UBX sync char 2 after sync char 1.");
            inputBuffer.clear();
            state = WAITING_FOR_START_BYTE;
        }
        break;

    case UBX_WAITING_FOR_MESSAGE_CLASS:
        inputBuffer.append(inbyte);
        state = UBX_WAITING_FOR_MESSAGE_ID;
        break;

    case UBX_WAITING_FOR_MESSAGE_ID:
        inputBuffer.append(inbyte);
        state = UBX_WAITING_FOR_LENGTH_BYTE_1;
        break;

    case UBX_WAITING_FOR_LENGTH_BYTE_1:
        inputBuffer.append(inbyte);
        state = UBX_WAITING_FOR_LENGTH_BYTE_2;
        break;

    case UBX_WAITING_FOR_LENGTH_BYTE_2:
        inputBuffer.append(inbyte);
        ubxPayloadLength = static_cast<unsigned char>(inputBuffer[4]) + 256 * static_cast<unsigned char>(inputBuffer[5]);

        if (ubxPayloadLength > maxUBXMessageLength - 8)
        {
            inputBuffer.clear();
            state = WAITING_FOR_START_BYTE;
            emit ubxParseError("UBX message length exceeded maximum value.");
        }

        if (ubxPayloadLength != 0)
        {
            state = UBX_RECEIVING_PAYLOAD;
        }
        else
        {
            state = UBX_WAITING_FOR_CK_A;
        }
        break;

    case UBX_RECEIVING_PAYLOAD:
        inputBuffer.append(inbyte);
        if (inputBuffer.length() >= ubxPayloadLength + 6)
        {
            state = UBX_WAITING_FOR_CK_A;
        }
        break;

    case UBX_WAITING_FOR_CK_A:
        inputBuffer.append(inbyte);
        state = UBX_WAITING_FOR_CK_B;
        break;

    case UBX_WAITING_FOR_CK_B:
        inputBuffer.append(inbyte);

        {
            // Calculate checksum
            unsigned char ck_a = 0;
            unsigned char ck_b = 0;

            for (int i = 2; i < inputBuffer.length() - 2; i++)
            {
                ck_a += static_cast<unsigned char>(inputBuffer[i]);
                ck_b += ck_a;
            }

            if (((static_cast<unsigned char>(inputBuffer[inputBuffer.length() - 2])) != ck_a) ||
                    ((static_cast<unsigned char>(inputBuffer[inputBuffer.length() - 1])) != ck_b))
            {
                emit ubxParseError("UBX message checksum error.");
            }
            else
            {
                // Frame is formally valid
                UBXMessage newUbxMessage(inputBuffer);
                emit ubxMessageReceived(newUbxMessage);
            }
        }

        inputBuffer.clear();
        state = WAITING_FOR_START_BYTE;
        break;

    case NMEA_WAITING_FOR_CR:
        // TODO: Add checksum check for NMEA and emit signal if fail

        inputBuffer.append(inbyte);

        if (inputBuffer.length() >= static_cast<int>(maxNMEASentenceLenght - 1))
        {
            emit nmeaParseError("NMEA sentence exceeded maximum length.");
            inputBuffer.clear();
            state = WAITING_FOR_START_BYTE;
        }
        else if (inbyte == 13)
        {
            state = NMEA_WAITING_FOR_LF;
        }
        break;

    case NMEA_WAITING_FOR_LF:
        inputBuffer.append(inbyte);
        if (inbyte == 10)
        {
            emit nmeaSentenceReceived(inputBuffer);
        }
        else
        {
            emit nmeaParseError("No LF after CR in the end of NMEA sentence.");
        }
        inputBuffer.clear();
        state = WAITING_FOR_START_BYTE;
        break;

    case RTCM_WAITING_FOR_MESSAGE_LENGTH_1:
        inputBuffer.append(inbyte);
        state = RTCM_WAITING_FOR_MESSAGE_LENGTH_2;
        break;

    case RTCM_WAITING_FOR_MESSAGE_LENGTH_2:
        inputBuffer.append(inbyte);

        // 10 lowest bits used, big-endian
        rtcmDataLength = (static_cast<unsigned char>(inputBuffer[2]) + 256 * static_cast<unsigned char>(inputBuffer[1])) & 0x3FF;

        if (rtcmDataLength != 0)
        {
            state = RTCM_RECEIVING_PAYLOAD;
        }
        else
        {
            state = RTCM_WAITING_FOR_CRC_1;
        }
        break;

    case RTCM_RECEIVING_PAYLOAD:
        inputBuffer.append(inbyte);
        if (inputBuffer.length() >= rtcmDataLength + 3)
        {
            state = RTCM_WAITING_FOR_CRC_1;
        }
        break;

    case RTCM_WAITING_FOR_CRC_1:
        inputBuffer.append(inbyte);
        state = RTCM_WAITING_FOR_CRC_2;
        break;

    case RTCM_WAITING_FOR_CRC_2:
        inputBuffer.append(inbyte);
        state = RTCM_WAITING_FOR_CRC_3;
        break;

    case RTCM_WAITING_FOR_CRC_3:
        inputBuffer.append(inbyte);

        // TODO: Calculate and check CRC
        // Now this just naively emits data without any checking

        RTCMMessage newRTCMMessage(inputBuffer);
        emit rtcmMessageReceived(newRTCMMessage);

        inputBuffer.clear();
        state = WAITING_FOR_START_BYTE;
        break;
    }
}

void UBloxDataStreamProcessor::process(const QByteArray& data)
{
    for (int i = 0; i < data.length(); i++)
    {
        process(data[i]);
    }
}

void UBloxDataStreamProcessor::flushInputBuffer()
{
    inputBuffer.clear();
    state = WAITING_FOR_START_BYTE;
}

unsigned int UBloxDataStreamProcessor::getNumOfUnprocessedBytes(void)
{
    return static_cast<unsigned int>(inputBuffer.length());
}
