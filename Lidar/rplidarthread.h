/*
    rplidarthread.h (part of GNSS-Stylus)
    Copyright (C) 2020-2021 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef RPLIDARTHREAD_H
#define RPLIDARTHREAD_H

#include <QObject>
#include <QThread>

#include "rplidar_sdk/include/rplidar.h"

class RPLidarThread : public QThread
{
    Q_OBJECT

public:
    RPLidarThread();

    /**
     * @brief Constructor
     * @param serialPortFileName File name of com port to open
     * @param serialPortBPS Speed (Bits Per Second) of the serial port
     */
    RPLidarThread(const QString& serialPortFileName, const unsigned int serialPortBPS, const unsigned short motorPWM = 660);
    ~RPLidarThread() override;

    void run() override;            //!< Thread code
    void requestTerminate(void);    //!< Requests thread to terminate
    void suspend(void);             //!< Requests thread to suspend. Suspend may not be immediate
    void resume(void);              //!< Requests thread to resume (from suspend). Resuming may not be immediate.

    class DistanceItem
    {
    public:
        float angle;                //!< Radians (0...2 * pi)
        float distance;             //!< m
        float quality;              //!< Value returned from RPLidar ("Related the reflected laser pulse strength"), 0...255 -> 0...1
    };

private:
    QString serialPortFileName;
    unsigned int serialPortBPS;
    unsigned short motorPWM = 0;

    bool terminateRequest = false;
    bool suspended = false;

    QString getRPLidarResultString(const u_result result);
    bool suspendIfNeeded(void);

    rplidar_response_measurement_node_hq_t measBuffer[20000];

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

    void distanceRoundReceived(const QVector<RPLidarThread::DistanceItem>& data, qint64 startTime, qint64 endTime);

// QT's signals and slots need to be defined exactly the same way, therefore SerialThread::DataReceivedEmitReason
//    void dataReceived(const QByteArray&, qint64 startTime, qint64 endTime, const SerialThread::DataReceivedEmitReason&);   //!< Signal that is emitted when data is received. Amount of bytes is limited either by maxReadDataSize or time elapses between two received bytes. Times are read by QElapsedTimer::msecsSinceReference()
//    void serialTimeout(void);               //!< Signal that is emitted when charTimeout have been elapsed after last received byte or no bytes received in charTimeout.

};

Q_DECLARE_METATYPE(QVector<RPLidarThread::DistanceItem>);

#endif // RPLIDARTHREAD_H
