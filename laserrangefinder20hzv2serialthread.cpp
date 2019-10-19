/*
    laserrangefinder20hzv2serialthread.cpp (part of GNSS-Stylus)
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
 * @file laserrangefinder20hzv2serialthread.cpp
 * @brief Definitions for a class handling serial communication of a cheap "V2" 20Hz laser rangefinder module.
 */

#include <QElapsedTimer>
#include "laserrangefinder20hzv2serialthread.h"

LaserRangeFinder20HzV2SerialThread::LaserRangeFinder20HzV2SerialThread(const QString& serialPortFileName, const double distanceOffset, const MeasurementResolution resolution)
    : QThread()
{
    this->serialPortFileName = serialPortFileName;
    this->distanceOffset = distanceOffset;
    this->resolution = resolution;

    terminateRequest = false;
    suspended = false;
}

LaserRangeFinder20HzV2SerialThread::~LaserRangeFinder20HzV2SerialThread()
{
    terminateRequest = true;
    this->wait(5000);
}

static const QString serialPortErrors[] =
{
    "NoError",
    "DeviceNotFoundError",
    "PermissionError",
    "OpenError",
    "ParityError",
    "FramingError",
    "BreakConditionError",
    "WriteError",
    "ReadError",
    "ResourceError",
    "UnsupportedOperationError",
    "UnknownError",
    "TimeoutError",
    "NotOpenError"
};

void LaserRangeFinder20HzV2SerialThread::run()
{
    enum
    {
        WAITING_FOR_FIRST_BYTE = 0,
        WAITING_FOR_SECOND_BYTE,
        WAITING_FOR_THIRD_BYTE,
        WAITING_FOR_FOURTH_BYTE,

        RECEIVING_DISTANCE,
        RECEIVING_ERROR,

        WAITING_FOR_CS_DISTANCE,
        WAITING_FOR_CS_ERROR,
    } state = WAITING_FOR_FIRST_BYTE;

    QSerialPort serialPort;

    serialPort.setPortName(serialPortFileName);
    serialPort.setBaudRate(9600);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);

    while (!terminateRequest)
    {
        // Try to open serial port
        do
        {
            if (suspended && !terminateRequest)
            {
                emit infoMessage("Suspending...");
                while (suspended && !terminateRequest)
                {
                    msleep(100);
                }
                if (!terminateRequest)
                {
                    emit infoMessage("Resuming...");
                }
            }

            emit infoMessage("Opening serial port \"" + serialPortFileName + "\"...");
            if (!serialPort.open(QIODevice::ReadWrite))
            {
                QSerialPort::SerialPortError error = serialPort.error();

                if (error < sizeof(serialPortErrors) / sizeof(serialPortErrors[0]))
                {
                    emit errorMessage("Can't open serial port\"" + serialPortFileName + "\". Reason: " + serialPortErrors[error] + ". Trying again after 1 s...");
                }
                else
                {
                    emit errorMessage("Can't open serial port\"" + serialPortFileName + "\", error " + QString::number(error) + ". Trying again after 1 s...");
                }
                sleep(1);
            }
        } while ((!serialPort.isOpen()) && !terminateRequest);

        if (!terminateRequest)
        {
            flushReceiveBuffer(&serialPort);
            emit infoMessage("Entering main loop.");
        }

        QElapsedTimer lastByteReceivedTimer;
        QElapsedTimer dataStartTimer;

        while (!terminateRequest)
        {
            char readBuffer[20];
            unsigned int bytesRead;

            unsigned long bytesToRead;

            switch (state)
            {
            default:
            case WAITING_FOR_FIRST_BYTE:
                bytesToRead = 11;
                break;
            case WAITING_FOR_SECOND_BYTE:
                bytesToRead = 10;
                break;
            case WAITING_FOR_THIRD_BYTE:
                bytesToRead = 9;
                break;
            case WAITING_FOR_FOURTH_BYTE:
                bytesToRead = 8;
                break;
            case RECEIVING_DISTANCE:
            case RECEIVING_ERROR:
                bytesToRead = 11 - receiveBuffer.length();
                break;
            case WAITING_FOR_CS_DISTANCE:
            case WAITING_FOR_CS_ERROR:
                bytesToRead = 1;
            }

            if ((resolution == RESOLUTION_01mm) && (state != WAITING_FOR_CS_ERROR) && (state != WAITING_FOR_CS_DISTANCE))
            {
                bytesToRead++;
            }

            if ((serialPort.bytesAvailable() != 0) ||  (serialPort.waitForReadyRead(1)))
            {
                // Data is ready to be read

                if (receiveBuffer.length() == 0)
                {
                    // No bytes received in this "burst" -> Store the starting time

                    dataStartTimer.start();
                }

                bytesRead = serialPort.read(readBuffer, bytesToRead);

                if (bytesRead > 0)
                {
                    for (unsigned int i = 0; i < bytesRead; i++)
                    {
                        unsigned char readByte = readBuffer[i];

                        if (state != WAITING_FOR_FIRST_BYTE)
                        {
                            receiveBuffer.append(readByte);
                        }

                        switch (state)
                        {
                        default:
                        case WAITING_FOR_FIRST_BYTE:
                            if (readByte == 0x80)
                            {
                                if (!receiveBuffer.isEmpty())
                                {
                                    emit unidentifiedDataReceived(receiveBuffer, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference());
                                    receiveBuffer.clear();
                                }
                                receiveBuffer.append(readByte);
                                state = WAITING_FOR_SECOND_BYTE;
                            }
                            break;
                        case WAITING_FOR_SECOND_BYTE:
                            if (readByte == 0x06)
                            {
                                state = WAITING_FOR_THIRD_BYTE;
                            }
                            else
                            {
                                state = WAITING_FOR_FIRST_BYTE;
                            }
                            break;
                        case WAITING_FOR_THIRD_BYTE:
                            if (readByte == 0x83)
                            {
                                state = WAITING_FOR_FOURTH_BYTE;
                            }
                            else
                            {
                                state = WAITING_FOR_FIRST_BYTE;
                            }
                            break;
                        case WAITING_FOR_FOURTH_BYTE:
                            if ((readByte >= '0') && (readByte <= '9'))
                            {
                                state = RECEIVING_DISTANCE;
                            }
                            else if (readByte == 0x45)
                            {
                                state = RECEIVING_ERROR;
                            }
                            else
                            {
                                state = WAITING_FOR_FIRST_BYTE;
                            }
                            break;
                        case RECEIVING_DISTANCE:
                            if (receiveBuffer.length() == 7)    // Place for decimal point
                            {
                                if (readByte != '.')
                                {
                                    // Decimal point not in expected place
                                    state = WAITING_FOR_FIRST_BYTE;
                                }
                            }
                            else if ((receiveBuffer.length() < 11) || ((resolution == RESOLUTION_01mm) && (receiveBuffer.length() < 12)))
                            {
                                if ((readByte < '0') || (readByte > '9'))
                                {
                                    // character not a decimal digit
                                    state = WAITING_FOR_FIRST_BYTE;
                                }
                                else if (((resolution == RESOLUTION_1mm) && (receiveBuffer.length() == 10)) ||
                                        ((resolution == RESOLUTION_01mm) && (receiveBuffer.length() == 11)))
                                {
                                    state = WAITING_FOR_CS_DISTANCE;
                                }
                            }
                            else
                            {
                                // This should not be reached
                                state = WAITING_FOR_FIRST_BYTE;
                            }
                            break;

                        case RECEIVING_ERROR:
                            if (((resolution == RESOLUTION_1mm) && (receiveBuffer.length() == 10)) ||
                                    ((resolution == RESOLUTION_01mm) && (receiveBuffer.length() == 11)))
                            {
                                state = WAITING_FOR_CS_ERROR;
                            }
                            break;

                        case WAITING_FOR_CS_DISTANCE:
                            if (checkChecksum(receiveBuffer))
                            {
                                QString decimalString = receiveBuffer.mid(3, (resolution == RESOLUTION_01mm) ? 8 : 7);
                                bool convOk = false;
                                double distance = decimalString.toDouble(&convOk);
                                distance += distanceOffset;

                                if (convOk)
                                {
                                    lastByteReceivedTimer.start();
                                    emit distanceReceived(distance, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference());
                                    receiveBuffer.clear();
                                    state = WAITING_FOR_FIRST_BYTE;
                                }
                            }
                            else
                            {
                                state = WAITING_FOR_FIRST_BYTE;
                            }

                            break;

                        case WAITING_FOR_CS_ERROR:
                            if (checkChecksum(receiveBuffer))
                            {
                                QString errorString = receiveBuffer.mid(3, (resolution == RESOLUTION_01mm) ? 8 : 7);

                                lastByteReceivedTimer.start();
                                emit errorReceived(errorString, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference());
                                receiveBuffer.clear();
                                state = WAITING_FOR_FIRST_BYTE;
                            }
                            else
                            {
                                state = WAITING_FOR_FIRST_BYTE;
                            }

                            break;

                        }
                    }
                }

                lastByteReceivedTimer.start();
            }

            if (lastByteReceivedTimer.elapsed() >= 10)
            {
                if (receiveBuffer.length() != 0)
                {
                    // Byte not received inside the timeout
                    emit unidentifiedDataReceived(receiveBuffer, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference());
                    receiveBuffer.clear();
                }

                lastByteReceivedTimer.start();
            }

            if (suspended && !terminateRequest)
            {
                emit infoMessage("Suspending...");
                while (suspended && !terminateRequest)
                {
                    msleep(100);
                }
                if (!terminateRequest)
                {
                    emit infoMessage("Resuming...");
                    flushReceiveBuffer(&serialPort);
                }
            }
        } // while (!terminateRequest)
    } // while (!terminateRequest)

    serialPort.close();

    emit infoMessage("Thread terminated.");
}

void LaserRangeFinder20HzV2SerialThread::flushReceiveBuffer(QSerialPort* serialPort)
{
    while (serialPort->bytesAvailable() != 0)
    {
        serialPort->read(256);
    }

    receiveBuffer.clear();
}

void LaserRangeFinder20HzV2SerialThread::suspend(void)
{
    suspended = true;
}

void LaserRangeFinder20HzV2SerialThread::resume(void)
{
    suspended = false;
}

void LaserRangeFinder20HzV2SerialThread::requestTerminate(void)
{
    terminateRequest = true;
}

bool LaserRangeFinder20HzV2SerialThread::checkChecksum(const QByteArray& array)
{
    // From "data sheet":
    // "CS check byte, it sums all the bytes in front,Returns the reverse, plus 1,
    // in the data returned by single measurements and successive measurements,
    // in which the quotes are part of the data,Format is ASCII sample:123.456 m
    // display 31 32 33 2E 34 35 36 ADDR Default value  80(128)"
    // ...
    // Clear, isn't it? "quotes are part of the data"... What?
    // Code below seems to work, though.

    unsigned char checksum = 0;

    for (int i = 0; i < array.length() - 1; i++)
    {
        checksum += static_cast<unsigned char>(array.at(i));
    }

    checksum = ~checksum;
    checksum++;

    if (checksum == static_cast<unsigned char>(array.at(receiveBuffer.length()-1)))
    {
        return true;
    }
    else
    {
        return false;
    }
}


