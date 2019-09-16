/*
    ubloxdatastreamprocessor.h (part of GNSS-Stylus)
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
 * @file ubloxdatastreamprocessor.h
 * @brief Declarations for a class splitting different types of messages (NMEA, UBX, RTCM) sent by u-blox devices from a stream of bytes.
 */

#ifndef UBLOXDATASTREAMPROCESSOR_H
#define UBLOXDATASTREAMPROCESSOR_H

#include <QObject>
#include <QByteArray>
#include "gnssmessage.h"

/**
 * @brief Class that takes stream of bytes and splits it to different types of messages (NMEA, UBS, RTCM)
 */
class UBloxDataStreamProcessor : public QObject
{
    Q_OBJECT

private:
    QByteArray inputBuffer;

    typedef enum
    {
        WAITING_FOR_START_BYTE = 0,

        UBX_WAITING_FOR_SYNC_CHAR_2,
        UBX_WAITING_FOR_MESSAGE_CLASS,
        UBX_WAITING_FOR_MESSAGE_ID,
        UBX_WAITING_FOR_LENGTH_BYTE_1,
        UBX_WAITING_FOR_LENGTH_BYTE_2,
        UBX_RECEIVING_PAYLOAD,
        UBX_WAITING_FOR_CK_A,
        UBX_WAITING_FOR_CK_B,

        NMEA_WAITING_FOR_CR,
        NMEA_WAITING_FOR_LF,

        RTCM_WAITING_FOR_MESSAGE_LENGTH_1,
        RTCM_WAITING_FOR_MESSAGE_LENGTH_2,
        RTCM_RECEIVING_PAYLOAD,
        RTCM_WAITING_FOR_CRC_1,
        RTCM_WAITING_FOR_CRC_2,
        RTCM_WAITING_FOR_CRC_3,
    } State;

    State state;

    unsigned short ubxPayloadLength;
    unsigned short rtcmDataLength;

    unsigned int maxNMEASentenceLenght;
    unsigned int maxUBXMessageLength;

    unsigned int maxUnidentifiedDataSize;

public:
    /**
     * @brief Constructor
     * @param maxUBXMessageLength Maximum length of UBX-message including all headers/tails (bytes)
     * @param maxNMEASentenceLenght Maximum length of NMEA-message (characters)
     */
    UBloxDataStreamProcessor(const unsigned int maxUBXMessageLength = 65536+8,
                             const unsigned int maxNMEASentenceLenght = 1024,
                             const unsigned int maxUnidentifiedDataSize = 100);
    void process(const char byte);                  //!< Processes single byte
    void process(const QByteArray& data);           //!< Processes data
    void flushInputBuffer(void);                    //!< Discards any data already in input buffer
    unsigned int getNumOfUnprocessedBytes(void);    //!< @returns number of unprocessed bytes

signals:
    void nmeaSentenceReceived(const QByteArray&);   //!< Complete NMEA-sentence has been interpreted from input stream
    void ubxMessageReceived(const UBXMessage&);     //!< Complete and formally valid UBX-message has been interpreted from input stream
    void rtcmMessageReceived(const RTCMMessage&);   //!< Complete RTCM-message has been interpreted from input stream
    void ubxParseError(const QString&);             //!< Parsing of UBX-message failed. String is descriptive string about the reason.
    void nmeaParseError(const QString&);            //!< Parsing of NMEA-message failed. String is descriptive string about the reason.
    void unidentifiedDataReceived(const QByteArray& data);  //!< Data can't be interpreted as one of the supported types (NMEA/UBX/RTCM)
};

#endif // UBLOXDATASTREAMPROCESSOR_H
