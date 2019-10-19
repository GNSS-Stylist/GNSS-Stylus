/*
    laserrangefinder20hzv2serialthread.h (part of GNSS-Stylus)
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
 * @file laserrangefinder20hzv2serialthread.h
 * @brief Declarations for a class handling serial communication of a cheap "V2" 20Hz laser rangefinder module.
 */

#ifndef LASERRANGEFINDER20HZV2SERIALTHREAD_H
#define LASERRANGEFINDER20HZV2SERIALTHREAD_H

#include <QObject>
#include <QThread>
//#include <QMutex>
//#include <QQueue>
#include <QSerialPort>

/**
 * @brief Class that handles all serial communication with "V2" 20Hz laser rangefinder module
 */
class LaserRangeFinder20HzV2SerialThread : public QThread
{
    Q_OBJECT

public:

    typedef enum
    {
        RESOLUTION_1mm,
        RESOLUTION_01mm
    } MeasurementResolution;

private:
    volatile bool terminateRequest; //!< Thread requested to terminate
    volatile bool suspended;        //!< Thread suspended (=paused/in sleep)

    QByteArray receiveBuffer;       //!< Buffer for received data

    void flushReceiveBuffer(QSerialPort* serialPort);//!< Flushes receive buffer

    QString serialPortFileName;         //!< File name for serial port
    double distanceOffset;

    MeasurementResolution resolution;

    bool checkChecksum(const QByteArray& array);

public:
    /**
     * @brief Constructor
     * @param serialPortFileName File name of com port to open
     * @param distanceOffset Offset to add to distance returned from the device
     */
    LaserRangeFinder20HzV2SerialThread(const QString& serialPortFileName, const double distanceOffset, const MeasurementResolution resolution);
    ~LaserRangeFinder20HzV2SerialThread() override;

    void run() override;            //!< Thread code
    void requestTerminate(void);    //!< Requests thread to terminate
    void suspend(void);             //!< Requests thread to suspend. Suspend may not be immediate
    void resume(void);              //!< Requests thread to resume (from suspend). Resuming may not be immediate.

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

    void distanceReceived(const double& distance, qint64 startTime, qint64 endTime);   //!< Signal that is emitted when distance is received. Times are read by QElapsedTimer::msecsSinceReference()
    void errorReceived(const QString& errorString, qint64 startTime, qint64 endTime);   //!< Signal that is emitted when error is received. Times are read by QElapsedTimer::msecsSinceReference()
    void unidentifiedDataReceived(const QByteArray& data, qint64 startTime, qint64 endTime);
};

#endif // LASERRANGEFINDER20HZV2SERIALTHREAD_H
