/*
    serialthread.h (part of GNSS-Stylus)
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
 * @file serialthread.h
 * @brief Declarations for a class handling serial communication.
 * This is currently Windows-only, but could easily be ported to other oparating systems.
 */

#ifndef SERIALLISTENERTHREAD_H
#define SERIALLISTENERTHREAD_H

#include <QObject>
#include <QThread>
#include <QMutex>
#include <QQueue>
#include <windows.h>

#include "gnssmessage.h"

/**
 * @brief Class that handles all serial communication with u-blox-devices
 */
class SerialThread : public QThread
{
    Q_OBJECT

private:
    volatile bool terminateRequest; //!< Thread requested to terminate
    volatile bool suspended;        //!< Thread suspended (=paused/in sleep)

    QMutex sendMutex;               //!< Mutex for handling sending of the data through serial port
    QQueue<QByteArray> sendQueue;   //!< Queue for data to be sent through serial port

    QByteArray receiveBuffer;       //!< Buffer for received data

    void flush(void);               //!< Flushes receive buffer

    void setDefaultCommTimeouts(void);  //!< Sets default comm timeouts

    HANDLE commHandle;                  //!< Handle for com port

    QString comPortFileName;            //!< File name for com port
    unsigned int charTimeout;           //!< Timeout (ms) when reading after which any received bytes are emitted. Timeout-signal is emitted afterwards even if no bytes were received.
    unsigned int maxReadDataSize;       //!< Maximum bytes allowed to be transferred with one dataReceived-signal

public:
    /**
     * @brief Constructor
     * @param comPortFileName File name of com port to open
     * @param charTimeout Maximum time to allow between two received bytes whe maxReadDataSize bytes were not received yet. Signal is emitted when exceeded.
     * @param maxReadDataSize Maximum number of bytes to read on one go. Signal is emitted after this number of bytes were received.
     */
    SerialThread(const QString& comPortFileName,
                 const unsigned int charTimeout = 20,
                 const unsigned int maxReadDataSize = 256);
    ~SerialThread() override;
    void run() override;            //!< Thread code
    void requestTerminate(void);    //!< Requests thread to terminate
    void suspend(void);             //!< Requests thread to suspend. Suspend may not be immediate
    void resume(void);              //!< Requests thread to resume (from suspend). Resuming may not be immediate.
    void addToSendQueue(const QByteArray& dataToSend);  //!< Adds dataToSend to thread's send queue. Data will be sent later.

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message
    void dataReceived(const QByteArray&);   //!< Signal that is emitted when data is received. Amount of bytes is limited either by maxReadDataSize or time elapses between two received bytes.
    void serialTimeout(void);               //!< Signal that is emitted when charTimeout have been elapsed after last received byte or no bytes received in charTimeout.
};

#endif // SERIALLISTENERTHREAD_H
