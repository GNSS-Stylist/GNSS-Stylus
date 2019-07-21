/*
    serialthread.cpp (part of GNSS-Stylus)
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
 * @file serialthread.cpp
 * @brief Definitions for a class handling serial communication.
 * This is currently Windows-only, but could easily be ported to other oparating systems.
 */

#include <memory>
#include "serialthread.h"

SerialThread::SerialThread(const QString& comPortFileName, const unsigned int charTimeout, const unsigned int maxReadDataSize)
    : QThread()
{
    this->comPortFileName = comPortFileName;
    this->charTimeout = charTimeout;
    this->maxReadDataSize = maxReadDataSize;

    commHandle = INVALID_HANDLE_VALUE;
    terminateRequest = false;
    suspended = false;
}

SerialThread::~SerialThread()
{
    terminateRequest = true;
    this->wait(5000);
}

void SerialThread::run()
{
    while (!terminateRequest)
    {
        // Try to open com port

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

            std::unique_ptr<wchar_t> portFileName_Wchar(new wchar_t[static_cast<unsigned int>(comPortFileName.length() + 1)]);   // +1 for null...

            int portNameLength = comPortFileName.toWCharArray(portFileName_Wchar.get());

            portFileName_Wchar.get()[portNameLength] = 0;

            emit infoMessage("Opening com port \"" + comPortFileName + "\"...");
            commHandle = CreateFile(portFileName_Wchar.get(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                    OPEN_EXISTING, 0, nullptr);

            if (commHandle == INVALID_HANDLE_VALUE)
            {
                emit errorMessage("Can't open com port\"" + comPortFileName + "\". Trying again after 1 s...");
                sleep(1);
            }
            else
            {
                DCB CommDCB;

                ZeroMemory(&CommDCB, sizeof(DCB));
                CommDCB.DCBlength = sizeof(DCB);
                GetCommState(commHandle, &CommDCB);

                CommDCB.BaudRate=115200;
                CommDCB.fBinary=1;
                CommDCB.fParity=0;
                CommDCB.fOutxCtsFlow=0;
                CommDCB.fOutxDsrFlow=0;
                CommDCB.fDtrControl=DTR_CONTROL_ENABLE;
                CommDCB.fDsrSensitivity=0;
                CommDCB.fOutX=0;
                CommDCB.fInX=0;
                CommDCB.fNull=0;
                CommDCB.fRtsControl=RTS_CONTROL_HANDSHAKE;
                CommDCB.fAbortOnError=0;
                CommDCB.ByteSize=8;
                CommDCB.Parity=NOPARITY;
                CommDCB.StopBits=ONESTOPBIT;

                // Set com port parameters
                if (!SetCommState(commHandle, &CommDCB))
                {
                    emit errorMessage("Can't set com port\"" + comPortFileName + "\". parameters. Opening port and retrying again after 1 s...");
                    CloseHandle(commHandle);
                    commHandle = INVALID_HANDLE_VALUE;
                    sleep(1);
                }
            }
        } while ((commHandle == INVALID_HANDLE_VALUE) && !terminateRequest);

        if (!terminateRequest)
        {
            setDefaultCommTimeouts();
            flush();
            sendMutex.lock();
            sendQueue.clear();
            sendMutex.unlock();
            emit infoMessage("Entering main loop.");
        }

        while (!terminateRequest)
        {
            char readBuffer[256];
            unsigned long BytesRead;
            do
            {
                // Read bytes from com port using only total timeout (see setDefaultCommTimeouts-function)
                // since some serial adapters can work strangely with "higher level" timeouts.

                unsigned long bytesToRead = sizeof(readBuffer);

                if (bytesToRead > maxReadDataSize - static_cast<unsigned int>(receiveBuffer.length()))
                {
                    bytesToRead = maxReadDataSize - static_cast<unsigned int>(receiveBuffer.length());
                }

                ReadFile(commHandle, readBuffer, bytesToRead, &BytesRead, nullptr);
                if (BytesRead != 0)
                {
                    // Bytes received inside the timeout
                    // -> Add them to the buffer
                    receiveBuffer.append(readBuffer, static_cast<int>(BytesRead));

                    if (static_cast<unsigned int>(receiveBuffer.length()) >= maxReadDataSize)
                    {
                        emit dataReceived(receiveBuffer);
                        receiveBuffer.clear();
                    }
                }
                else if (receiveBuffer.length() != 0)
                {
                    // No bytes received inside the timeout
                    // -> Emit any bytes already received
                    emit dataReceived(receiveBuffer);
                    receiveBuffer.clear();
                }

                sendMutex.lock();

                // Send any pending data from queue
                while(!sendQueue.isEmpty())
                {
                    QByteArray sendArray = sendQueue.dequeue();
                    sendMutex.unlock();

                    DWORD numofBytesWritten;

                    WriteFile(commHandle, sendArray.data(), DWORD(sendArray.length()), &numofBytesWritten, nullptr);

                    sendMutex.lock();
                }
                sendMutex.unlock();

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
                        flush();
                    }
                }
            } while (BytesRead && !terminateRequest);

            if (!terminateRequest)
            {
                emit serialTimeout();
            }
        }
    }

    CloseHandle(commHandle);
    commHandle = INVALID_HANDLE_VALUE;

    emit infoMessage("Thread terminated.");
}

void SerialThread::flush(void)
{
    COMMTIMEOUTS originalTimeouts;
    COMMTIMEOUTS to;

    GetCommTimeouts(commHandle, &originalTimeouts);

    to.ReadIntervalTimeout = MAXDWORD;
    to.ReadTotalTimeoutMultiplier = 0;
    to.ReadTotalTimeoutConstant = 0;
    SetCommTimeouts(commHandle, &to);

    // Read everything away from buffer

    unsigned long BytesRead;

    char TempBuffer[256];

    do
    {
        ReadFile(commHandle, &TempBuffer, sizeof(TempBuffer), &BytesRead, nullptr);
    } while (BytesRead);

    receiveBuffer.clear();

    SetCommTimeouts(commHandle, &originalTimeouts);
}


void SerialThread::addToSendQueue(const QByteArray& dataToSend)
{
    if (!terminateRequest)
    {
        QMutexLocker locker(&sendMutex);
        sendQueue.enqueue(dataToSend);
    }
}

void SerialThread::setDefaultCommTimeouts(void)
{
    COMMTIMEOUTS CommTimeouts;

    /* From SetCommTimeouts documentation:
    * If an application sets ReadIntervalTimeout and ReadTotalTimeoutMultiplier to MAXDWORD
    * and sets ReadTotalTimeoutConstant to a value greater than zero and less than MAXDWORD,
    * one of the following occurs when the ReadFile function is called:

    * If there are any bytes in the input buffer, ReadFile returns immediately with the bytes in the buffer.
    * If there are no bytes in the input buffer, ReadFile waits until a byte arrives and then returns immediately.
    * If no bytes arrive within the time specified by ReadTotalTimeoutConstant, ReadFile times out.
    */
    CommTimeouts.ReadIntervalTimeout = MAXDWORD;
    CommTimeouts.ReadTotalTimeoutMultiplier = MAXDWORD;
    CommTimeouts.ReadTotalTimeoutConstant = charTimeout;
    CommTimeouts.WriteTotalTimeoutMultiplier = 0;
    CommTimeouts.WriteTotalTimeoutConstant = 0;

    if (!SetCommTimeouts(commHandle, &CommTimeouts))
    {
        emit warningMessage("Can't set com port\"" + comPortFileName + "\". timeouts.");
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

