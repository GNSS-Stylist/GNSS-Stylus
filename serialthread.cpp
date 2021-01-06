/*
    serialthread.cpp (part of GNSS-Stylus)
    Copyright (C) 2019-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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
 * @file serialthread.cpp
 * @brief Definitions for a class handling serial communication.
 */

//#include <memory>
#include <QElapsedTimer>
#include "serialthread.h"

SerialThread::SerialThread(const QString& serialPortFileName,
                           const unsigned int charTimeout,
                           const unsigned int maxReadDataSize,
                           const unsigned int serialPortBPS)
    : QThread()
{
    this->serialPortFileName = serialPortFileName;
    this->charTimeout = charTimeout;
    this->maxReadDataSize = maxReadDataSize;
    this->serialPortBPS = serialPortBPS;

    terminateRequest = false;
    suspended = false;
}

SerialThread::~SerialThread()
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

void SerialThread::run()
{
    QSerialPort serialPort;

    serialPort.setPortName(serialPortFileName);
    serialPort.setBaudRate(serialPortBPS);
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
            sendMutex.lock();
            sendQueue.clear();
            sendMutex.unlock();
            emit infoMessage("Entering main loop.");
        }

        QElapsedTimer lastByteReceivedTimer;
        QElapsedTimer dataStartTimer;

        while (!terminateRequest)
        {
            char readBuffer[256];
            qint64 BytesRead;

            unsigned long bytesToRead = sizeof(readBuffer);

            if (bytesToRead > maxReadDataSize - static_cast<unsigned int>(receiveBuffer.length()))
            {
                bytesToRead = maxReadDataSize - static_cast<unsigned int>(receiveBuffer.length());
            }

            if ((serialPort.bytesAvailable() != 0) ||  (serialPort.waitForReadyRead(1)))
            {
                // Data is ready to be read

                lastByteReceivedTimer.start();

                if (receiveBuffer.length() == 0)
                {
                    // No bytes received in this "burst" -> Store the starting time

                    dataStartTimer.start();
                }

                BytesRead = serialPort.read(readBuffer, bytesToRead);

                if (BytesRead > 0)
                {
                    // Bytes received -> Add them to the buffer
                    receiveBuffer.append(readBuffer, static_cast<int>(BytesRead));

                    if (static_cast<unsigned int>(receiveBuffer.length()) >= maxReadDataSize)
                    {
                        emit dataReceived(receiveBuffer, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference(), MAX_BYTES);
                        receiveBuffer.clear();
                    }
                }
            }

            if (lastByteReceivedTimer.elapsed() >= charTimeout)
            {
                if (receiveBuffer.length() != 0)
                {
                    // Byte not received inside the timeout
                    // -> Emit any bytes already received
                    emit dataReceived(receiveBuffer, dataStartTimer.msecsSinceReference(), lastByteReceivedTimer.msecsSinceReference(), TIMEOUT);
                    receiveBuffer.clear();

                    // Also emit timeout
                    emit serialTimeout();
                }

                sendMutex.lock();

                // Send any pending data from queue
                while(!sendQueue.isEmpty())
                {
                    QByteArray sendArray = sendQueue.dequeue();
                    sendMutex.unlock();

                    serialPort.write(sendArray);

                    sendMutex.lock();
                }
                sendMutex.unlock();

                lastByteReceivedTimer.start();
            }

            if (suspended && !terminateRequest)
            {
                emit infoMessage("Suspending...");
                while (suspended && !terminateRequest)
                {
                    sendMutex.lock();
                    sendQueue.clear();
                    sendMutex.unlock();

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

void SerialThread::flushReceiveBuffer(QSerialPort* serialPort)
{
    while (serialPort->bytesAvailable() != 0)
    {
        serialPort->read(256);
    }

    receiveBuffer.clear();
}


void SerialThread::addToSendQueue(const QByteArray& dataToSend)
{
    if (!terminateRequest)
    {
        QMutexLocker locker(&sendMutex);
        sendQueue.enqueue(dataToSend);
    }
}

void SerialThread::suspend(void)
{
    suspended = true;
}

void SerialThread::resume(void)
{
    suspended = false;
}

void SerialThread::requestTerminate(void)
{
    terminateRequest = true;
}

