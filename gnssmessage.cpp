/*
    gnssmessage.cpp (part of GNSS-Stylus)
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
 * @file gnssmessage.cpp
 * @brief Definitions for different types of messages sent by u-blox devices.
 *
 * u-blox devices (in this case ZED-F9P) can send three types of messages
 * (UBX, NMEA and RTCM). These message types are handled inside different classes.
 */

#include "gnssmessage.h"

GNSSMessage::GNSSMessage()
{
    rawMessage.clear();
}

NMEAMessage::NMEAMessage()
{
    messageDataStatus = STATUS_UNINITIALIZED;
}

NMEAMessage::NMEAMessage(const QByteArray &messageString)
{
    // TODO: Add parsing with error/checksum-checking etc. Now this just copies the string.
    rawMessage = messageString;
    messageDataStatus = STATUS_VALID;
}

UBXMessage::UBXMessage()
{
    init();
}

UBXMessage::UBXMessage(const QByteArray& ubxRawData)
{
    rawMessage = ubxRawData;

    if (ubxRawData.length() < 8)
    {
        messageDataStatus = STATUS_ERROR_LENGTH;
    }
    else if ((static_cast<unsigned char>(ubxRawData.at(0)) != 0xB5) ||
             (static_cast<unsigned char>(ubxRawData.at(1)) != 0x62))
    {
        messageDataStatus = STATUS_ERROR_SYNC_CHAR;
    }
    else
    {
        payloadLength = static_cast<unsigned char>(ubxRawData.at(4)) | ((static_cast<unsigned char>(ubxRawData.at(5))) * 256L);

        if (payloadLength != ubxRawData.length() - 8)
        {
            messageDataStatus = STATUS_ERROR_LENGTH;
        }
        else
        {
            // Calculate checksum

            unsigned char ck_a = 0;
            unsigned char ck_b = 0;

            for (int i = 2; i < ubxRawData.length() - 2; i++)
            {
                ck_a += static_cast<unsigned char>(ubxRawData.at(i));
                ck_b += ck_a;
            }

            if (((static_cast<unsigned char>(ubxRawData.at(ubxRawData.length() - 2))) != ck_a) ||
                    ((static_cast<unsigned char>(ubxRawData.at(ubxRawData.length() - 1))) != ck_b))
            {
                messageDataStatus = STATUS_ERROR_CHECKSUM;
            }
            else
            {
                // Frame is formally valid
                messageDataStatus = STATUS_VALID;
                messageClass = static_cast<unsigned char>(ubxRawData.at(2));
                messageId = static_cast<unsigned char>(ubxRawData.at(3));
            }
        }
    }

    if (messageDataStatus != STATUS_VALID)
    {
        payloadLength = 0;
        messageClass = 0;
        messageId = 0;
    }
}

void UBXMessage::init()
{
    payloadLength = 0;
    messageClass = 0;
    messageId = 0;
}

UBXMessage_RELPOSNED::UBXMessage_RELPOSNED()
{
    initRELPOSNEDFields();
}

UBXMessage_RELPOSNED::UBXMessage_RELPOSNED(const UBXMessage &ubxMessage) : UBXMessage(ubxMessage)
{
    if (messageDataStatus == STATUS_VALID)
    {
        if (messageClass != 0x01)
        {
            messageDataStatus = STATUS_ERROR_CAST_CLASS;
        }
        else if (messageId != 0x3c)
        {
            messageDataStatus = STATUS_ERROR_CAST_ID;
        }
        else if (payloadLength != 64)
        {
            messageDataStatus = STATUS_ERROR_LENGTH;
        }
        else
        {
            // Use packed, "memory-mapped" source data in conversion to prevent byte-counting

#pragma pack(push, 1)
            //RELPOSNED-message as it is in the frame (packed)
            typedef struct
            {
                unsigned char version;          // Message version (0x01 for this version).
                unsigned char reserved1;        // Reserved.
                unsigned short refStationId;    // Reference Station ID. Must be in the range 0..4095.
                unsigned int iTOW;              // GPS time of week of the navigation epoch (ms).
                int relPosN;                    // North component of relative position vector (cm).
                int relPosE;                    // East component of relative position vector (cm).
                int relPosD;                    // Down component of relative position vector (cm).
                int relPosLength;               // Length of the relative position vector (cm).
                int relPosHeading;              // Heading of the relative position vector (1e-5 deg).
                unsigned char reserved2[4];     // Reserved.
                signed char relPosHPN;          // High-precision North component of relative position vector (0.1 mm).
                signed char relPosHPE;          // High-precision East component of relative position vector (0.1 mm).
                signed char relPosHPD;          // High-precision Down component of relative position vector (0.1 mm).
                signed char relPosHPLength;     // High-precision component of the length of the relative position vector (0.1 mm).
                unsigned int accN;              // Accuracy of relative position North component (0.1 mm).
                unsigned int accE;              // Accuracy of relative position East component (0.1 mm).
                unsigned int accD;              // Accuracy of relative position Down component (0.1 mm).
                unsigned int accLength;         // Accuracy of length of the relative position vector (0.1 mm).
                unsigned int accHeading;        // Accuracy of heading of the relative position vector (0.1 mm).
                unsigned char reserved3[4];     // Reserved.
                unsigned int flags;             // Flags.
            } UBXRawData_RELPOSNED;
#pragma pack(pop)

            UBXRawData_RELPOSNED* rawRELPOSNED = reinterpret_cast<UBXRawData_RELPOSNED*>(&rawMessage.data()[6]);

            version = rawRELPOSNED->version;
            refStationId = rawRELPOSNED->refStationId;
            iTOW = static_cast<ITOW>(rawRELPOSNED->iTOW);

            relPosN = rawRELPOSNED->relPosN / 100. + rawRELPOSNED->relPosHPN / 10e3;
            relPosE = rawRELPOSNED->relPosE / 100. + rawRELPOSNED->relPosHPE / 10e3;
            relPosD = rawRELPOSNED->relPosD / 100. + rawRELPOSNED->relPosHPD / 10e3;

            relPosLength = rawRELPOSNED->relPosLength / 100. + rawRELPOSNED->relPosHPLength / 10e3;
            relPosHeading = rawRELPOSNED->relPosHeading / 1e5;

            accN = rawRELPOSNED->accN / 10e3;
            accE = rawRELPOSNED->accE / 10e3;
            accD = rawRELPOSNED->accD / 10e3;

            accLength = rawRELPOSNED->accLength / 10e3;
            accHeading = rawRELPOSNED->accHeading / 1e5;
            flags = rawRELPOSNED->flags;

            flag_gnssFixOK = flags & (1 << 0);
            flag_diffSoln = flags & (1 << 1);
            flag_relPosValid = flags & (1 << 2);
            flag_carrSoln = static_cast<UBXRawData_RELPOSNED_CarrierPhaseSolutionStatus>((flags >> 3) & 3);
            flag_isMoving = flags & (1 << 5);
            flag_refPosMiss = flags & (1 << 6);
            flag_refObsMiss = flags & (1 << 7);
            flag_relPosHeadingValid = flags & (1 << 8);
        }
    }
}

void UBXMessage_RELPOSNED::initRELPOSNEDFields(void)
{
    version = 0;
    refStationId = 0;
    iTOW = -1;  // Negative = not valid

    relPosN = 0;
    relPosE = 0;
    relPosD = 0;

    relPosLength = 0;
    relPosHeading = 0;

    accN = 0;
    accE = 0;
    accD = 0;

    accLength = 0;
    accHeading = 0;
    flags = 0;

    flag_gnssFixOK = 0;
    flag_diffSoln = 0;
    flag_relPosValid = 0;
    flag_carrSoln = NO_SOLUTION;
    flag_isMoving = 0;
    flag_refPosMiss = 0;
    flag_refObsMiss = 0;
    flag_relPosHeadingValid = 0;
}

RTCMMessage::RTCMMessage()
{
    messageDataStatus = STATUS_UNINITIALIZED;
}

RTCMMessage::RTCMMessage(const QByteArray &rtcmData)
{
    // TODO: Add parsing with error/checksum-checking etc. Now this just copies the data.
    rawMessage = rtcmData;
    messageDataStatus = STATUS_VALID;

    if (rtcmData.length() >= 8)
    {
        messageType = static_cast<unsigned short>((static_cast<unsigned char>(rtcmData[3]) * 256 + static_cast<unsigned char>(rtcmData[4]))) >> 4;
    }
    else
    {
        messageType = 0;
    }
}

UBXMessage_RELPOSNED UBXMessage_RELPOSNED::interpolateCoordinates(const UBXMessage_RELPOSNED& startValues, const UBXMessage_RELPOSNED& endValues, const ITOW iTOW)
{
    // Only calculated fields (doubles) interpolated
    // if iTOW is out of range, values will be either start- or end-values

    UBXMessage_RELPOSNED retval;

    retval.iTOW = iTOW;
    retval.relPosN = interpolateDouble(startValues.relPosN, endValues.relPosN, startValues.iTOW, endValues.iTOW, iTOW);
    retval.relPosE = interpolateDouble(startValues.relPosE, endValues.relPosE, startValues.iTOW, endValues.iTOW, iTOW);
    retval.relPosD = interpolateDouble(startValues.relPosD, endValues.relPosD, startValues.iTOW, endValues.iTOW, iTOW);
    retval.relPosLength = interpolateDouble(startValues.relPosLength, endValues.relPosLength, startValues.iTOW, endValues.iTOW, iTOW);
    retval.accN = interpolateDouble(startValues.accN, endValues.accN, startValues.iTOW, endValues.iTOW, iTOW);
    retval.accE = interpolateDouble(startValues.accE, endValues.accE, startValues.iTOW, endValues.iTOW, iTOW);
    retval.accD = interpolateDouble(startValues.accD, endValues.accD, startValues.iTOW, endValues.iTOW, iTOW);
    retval.accLength = interpolateDouble(startValues.accLength, endValues.accLength, startValues.iTOW, endValues.iTOW, iTOW);
    retval.accHeading = interpolateDouble(startValues.accHeading, endValues.accHeading, startValues.iTOW, endValues.iTOW, iTOW);

    return retval;
}

double UBXMessage_RELPOSNED::interpolateDouble(const double startVal, const double endVal, const ITOW startITOW, const ITOW endITOW, ITOW currITOW)
{
    if (currITOW <= startITOW)
    {
        return startVal;
    }
    else if (currITOW >= endITOW)
    {
        return endITOW;
    }
    else
    {
        double fraction = static_cast<double>((currITOW - startITOW)) / (endITOW - startITOW);
        return startVal + fraction * (endVal - startVal);
    }
}

QString UBXMessage_RELPOSNED::getCarrSolnString(void)
{
    switch(flag_carrSoln)
    {
    case NO_SOLUTION:
        return "No solution";

    case FLOATING:
        return "Floating";

    case FIXED:
        return "Fixed";

//    case UNDEFINED:
    default:
        return "Undefined";
    }

}


















