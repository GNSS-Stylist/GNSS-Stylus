/*
    gnssmessage.h (part of GNSS-Stylus)
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
 * @file gnssmessage.h
 * @brief Declarations for different types of messages sent by u-blox devices.
 *
 * u-blox devices (in this case ZED-F9P) can send three types of messages
 * (UBX, NMEA and RTCM). These message types are handled inside different classes.
 */

#ifndef GNSSMESSAGE_H
#define GNSSMESSAGE_H

#include <QByteArray>
#include <QString>
//#include <QException>

/**
 * @brief Base class for all messages received from u-blox devices.
 */
class GNSSMessage
{
public:
    QByteArray rawMessage;  //!< Message contents as received from serial port. Includes all checksums and CR/LF (for NMEA) etc. Terminating null (for NMEA) is handled by QByteArray itself.
    GNSSMessage();
};

/**
 * @brief Class for NMEA-messages.
 *
 * Implementation is incomplete. No error checking and such.
 */
class NMEAMessage : public GNSSMessage
{
public:
    /**
     * @brief Default constructor that sets the message status to STATUS_UNINITIALIZED.
     */
    NMEAMessage();

    /**
     * @brief NMEAMessage Constructor that takes NMEA-string as an input.
     * @param messageString NMEA-message string, including every character (also CR/LF) but no terminating null as QByteArrays's "payload".
     */
    NMEAMessage(const QByteArray& messageString);

    /**
     * @brief Enumeration for message data status.
     */
    enum MessageDataStatus
    {
        STATUS_UNINITIALIZED = 0,   //!< String not given.
        STATUS_VALID,               //!< NMEA-sentence is valid (lenght ok, ends to CR/LF, checksum is valid).
        // TODO: Take into use        STATUS_ERROR_NO_CR_LF,       //!< Sentence is lacking CR/LF at the end.
        // TODO: Take into use        STATUS_ERROR_INVALID_CHARS,  //!< Invalid characters detected in string.
        // TODO: Take into use        STATUS_ERROR_CHECKSUM,       //!< Checksum error.
    } messageDataStatus; //!< Status of the data (rawMessage)
};

/**
 * @brief Class for UBX-messages.
 */
class UBXMessage : public GNSSMessage
{
public:
    /**
     * @brief Enum for message data status.
     */
    enum MessageDataStatus
    {
        STATUS_UNINITIALIZED = 0,       //!< Message not initialized.
        STATUS_VALID,                   //!< UBX-message is formally valid.

        STATUS_ERROR_SYNC_CHAR= 100,    //!< Sync chars not valid.
        STATUS_ERROR_LENGTH,            //!< Message length is not valid (below 8 bytes overall or length-field doesn't match data length given).
        STATUS_ERROR_CHECKSUM,          //!< Checksum error.

        STATUS_ERROR_CAST_CLASS = 200,  //!< Casting of "generic" UBXMessage into specific message failed, class mismatch.
        STATUS_ERROR_CAST_ID,           //!< Casting of "generic" UBXMessage into specific message failed, id mismatch.
    } messageDataStatus; //!< Status of the data (rawMessage)

    unsigned char messageClass;         //!< Message class (byte index 2 of UBX-frame).
    unsigned char messageId;            //!< Message id (byte index 3 of UBX-frame).
    unsigned short payloadLength;       //!< Payload length of UBX-message (indexes 4 & 5 of UBX-frame).

    void init();    //!< Initializes fields
    UBXMessage();   //!< Default constructor

    /**
     * @brief Parses raw data to UBX-message and sets messageDataStatus, messageClass, messageId and payloadLength accordingly.
     * @param ubxRawData Raw data to parse.
     */
    UBXMessage(const QByteArray& ubxRawData);
};

/**
 * @brief Class for UBX-RELPOSNED-message.
 */
class UBXMessage_RELPOSNED : public UBXMessage
{
public:
    typedef int ITOW;                   //!< Integer Time Of Week (ms). Negative values: value not valid.

    /**
     * @brief Carrier phase solution status ("RTK-solution"-state, should be "FIXED" for best accuracy).
     */
    typedef enum
    {
        NO_SOLUTION = 0,            //!< No carrier phase range solution.
        FLOATING = 1,               //!< Carrier phase range solution with floating ambiguities.
        FIXED = 2,                  //!< Carrier phase range solution with fixed ambiguities.
        UNDEFINED = 3,              //!< Undefined (bits in the flags-field can get this value, but this value is not defined).
    } UBXRawData_RELPOSNED_CarrierPhaseSolutionStatus;

    /**
     * @brief Default constructor initilizes all to "not initialized"-state.
     */
    UBXMessage_RELPOSNED();

    /**
     * @brief Contructor that tries to convert "generic" UBX-message to RELPOSNED.
     * Failure will be indicated in messageDataStatus-field.
     * @param ubxMessage "generic" UBX-message to convert from
     */
    UBXMessage_RELPOSNED(const UBXMessage& ubxMessage);

    unsigned char version;          //!< Message version.
    unsigned short refStationId;    //!< Reference Station ID. Must be in the range 0..4095.
    ITOW iTOW;                      //!< GPS time of week of the navigation epoch. See the description of iTOW for details. Negative: invalid value.

    double relPosN;                 //!< North component of relative position vector (m).
    double relPosE;                 //!< East component of relative position vector (m).
    double relPosD;                 //!< Down component of relative position vector (m).

    double relPosLength;            //!< Length of the relative position vector (m).
    double relPosHeading;           //!< Heading of the relative position vector (degrees).

    double accN;                    //!< Accuracy of relative position North component (m).
    double accE;                    //!< Accuracy of relative position East component (m).
    double accD;                    //!< Accuracy of relative position Down component (m).

    double accLength;               //!< Accuracy of length of the relative position vector (m).
    double accHeading;              //!< Accuracy of heading of the relative position vector (degrees).

    unsigned int flags;             //!< Flags.

    // Flags split to "sub-parts":
    bool flag_gnssFixOK;            //!< A valid fix (i.e within DOP & accuracy masks).
    bool flag_diffSoln;             //!< Differential corrections were applied.
    bool flag_relPosValid;          //!< Relative position components and accuracies are valid and, in moving base mode only, if baseline is valid.
    UBXRawData_RELPOSNED_CarrierPhaseSolutionStatus flag_carrSoln; //!< Carrier phase range solution status.
    bool flag_isMoving;             //!< The receiver is operating in moving base mode.
    bool flag_refPosMiss;           //!< Extrapolated reference position was used to compute moving base solution this epoch.
    bool flag_refObsMiss;           //!< Extrapolated reference observations were used to compute moving base solution this epoch.
    bool flag_relPosHeadingValid;   //!< RelPosHeading is valid.

    /**
     * @brief Function to interpolate between two RELPOSNED-coordinates using ITOWS.
     * startValues must have smaller ITOW than endValues.
     * Only values that are meaningful to interpolate are interpolated
     * (practically coordinates and accuracies that are presented as doubles in this case)
     * @param startValues starting RELPOSNED.
     * @param endValues ending RELPOSNED.
     * @param iTOW ITOW to "interpolate to". Will be limited to range defined by startValues and endValues.
     * @return RELPOSNED-message that has interpolated coordinates and accuracies.
     */
    static UBXMessage_RELPOSNED interpolateCoordinates(const UBXMessage_RELPOSNED& startValues, const UBXMessage_RELPOSNED& endValues, const ITOW iTOW);

    /**
     * @brief Function to get a descriptive string for flag_carrSoln.
     * @return Descriptive string for flag_carrSoln.
     */
    QString getCarrSolnString(void);

private:
    void initRELPOSNEDFields(void);
    static double interpolateDouble(const double startVal, const double endVal, const ITOW startITOW, const ITOW endITOW, ITOW currITOW);
};

/**
 * @brief Class for RTCM-messages.
 */
class RTCMMessage : public GNSSMessage
{
public:
    /**
     * @brief Default constructor that initialized the message to uninitialized state.
     */
    RTCMMessage();

    /**
     * @brief RTCMMessage Constructor that takes RTCM data as an input.
     * @param rtcmData RTCM data, including every byte (also 0xD3 and CRC).
     */
    RTCMMessage(const QByteArray& rtcmData);

    /**
     * @brief Enum for message data status.
     */
    enum MessageDataStatus
    {
        STATUS_UNINITIALIZED = 0,       //!< Data not given
        STATUS_VALID,                   //!< RTCM-data is valid (CRC is valid)
        // TODO: Take into use        STATUS_ERROR_CRC,       //!< CRC error
    } messageDataStatus; //!< Status of the data (rawMessage)

    unsigned short messageType;         //!< RTCM-message type (first 12 bits of data). 0 if data length < 2 bytes
};


#endif // GNSSMESSAGE_H
