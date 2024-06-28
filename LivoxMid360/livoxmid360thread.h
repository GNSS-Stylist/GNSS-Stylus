/*
    livoxmid360thread.h (part of GNSS-Stylus)
    Copyright (C) 2024 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#ifndef LIVOXMID360THREAD_H
#define LIVOXMID360THREAD_H

#include <QObject>
#include <QThread>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QElapsedTimer>

#include "PostProcessing/postprocessingform.h"
#include "ubloxdatastreamprocessor.h"
#include "FastCRC/FastCRC.h"
#include "livoxmid360controlcommand.h"
#include <QMutex>

class LivoxMid360Thread : public QThread
{
    Q_OBJECT

public:
    LivoxMid360Thread(const quint32 hostIPAddress, UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA, const bool replayMode, QVector<quint32> allowedDeviceIPs);
    ~LivoxMid360Thread() override;

    friend class LivoxMid360DatagramHandler;

    void run() override;            //!< Thread code
    void requestTerminate(void);    //!< Requests thread to terminate
    void suspend(void);             //!< Requests thread to suspend (ignoring all incoming packets). Suspend may not be immediate
    void resume(void);              //!< Requests thread to resume (from suspend). Resuming may not be immediate.

    class DeviceCounters
    {
    public:
        quint64 totalDatagrams;
        quint64 totalDatagramBytes;

        quint64 pushLidarInformationDatagrams;
        quint64 pushLidarInformationDatagramBytes;

        quint64 pointCloudDataDatagrams;
        quint64 pointCloudDataDatagramBytes;
        quint64 pointCloudPoints;

        quint64 imuDataDatagrams;
        quint64 imuDataDatagramBytes;

        void init(void);

        DeviceCounters operator - (DeviceCounters const& subtractor);
        DeviceCounters operator + (DeviceCounters const& additor);
    };

    QMap<quint32, DeviceCounters> getDeviceCounters(void);
    void clearDeviceCounters(void);

    void connectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor
    void disconnectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor
    void connectPostProcessingSlots(PostProcessingForm* postProcessingForm);
    void disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm);

    // Currently expecting the mid-360 to be configured correctly already and just listening for the data
    // Therefore only 3 listening ports are used for now (for "push command" (=lidar info), points and IMU data).
    const quint16 controlCommandPort_Host = 56101;
    const quint16 controlCommandPort_Lidar = 56100;
    //const quint16 pushCommandPort_Lidar = 56200;
    const quint16 pushCommandPort_Host = 56201;
    //const quint16 pointCloudDataPort_Lidar = 56300;
    const quint16 pointCloudDataPort_Host = 56301;
    //const quint16 imuDataPort_Lidar = 56400;
    const quint16 imuDataPort_Host = 56401;

private:
//    class DatagramHandler* datagramHandler = nullptr;

    typedef enum
    {
        DT_DISCARDED = 0,
        DT_ENABLED,
    } DetectionType;

    QVector<quint32> allowedDeviceIPs;
    QMap<quint32, DetectionType> detectedDeviceIPs;

    quint32 hostIPAddress;

    bool terminateRequest = false;
    bool suspended = false;
    bool replayMode = false;

    bool suspendIfNeeded(void);

    QUdpSocket* udpSocket_ControlCommand = nullptr;
    QUdpSocket* udpSocket_PushCommand = nullptr;
    QUdpSocket* udpSocket_PointCloudData = nullptr;
    QUdpSocket* udpSocket_IMUData = nullptr;

    quint32 setGPSTimestampSeqNum = 0;
    UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA = nullptr;

    QMutex deviceCountersMutex;
    QMap<quint32, DeviceCounters> deviceCounters;

    void handleDatagram_ControlCommand(const QNetworkDatagram& datagram, qint64 uptime);
    void handleDatagram_PushCommand(const QNetworkDatagram& datagram, qint64 uptime);
    void handleDatagram_PointCloudData(const QNetworkDatagram& datagram, qint64 uptime);
    void handleDatagram_IMUData(const QNetworkDatagram& datagram, qint64 uptime);

    bool checkDeviceValidity(const quint32 senderAddress);

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

    void rawDatagramReceived(const QNetworkDatagram&, qint64);
    void pushLidarInformationReceived(quint32 ipAddress, const LivoxMid360::PushLidarInformation&, qint64);

private slots:

    void on_rawReplayDatagramReceived(const QNetworkDatagram& datagram, qint64 timeStamp);

//    void newDeviceDetected(quint32 ipAddress, QByteArray, quint8 devType, qint64);

//    void newPointsInfo(QByteArray deviceSerialNumber, quint32 pointCount, CoordType coordType);
//    void newIMUData(QByteArray deviceSerialNumber, float gyro_x, float gyro_y, float gyro_z, float acc_x, float acc_y, float acc_z);
/*
public slots:
    void readPendingDatagrams_PushCommand();
    void readPendingDatagrams_PointCloudData();
    void readPendingDatagrams_IMUData();
*/
};

// This should be in the cpp-file, but connect doesn't seem to work then
// ("undefined reference to `vtable for LivoxMid360DatagramHandler'")
class LivoxMid360DatagramHandler : public QObject
{
    Q_OBJECT
public:
    LivoxMid360DatagramHandler(LivoxMid360Thread* ownerThread) { this->ownerThread = ownerThread; };
    void run();
private:
    LivoxMid360Thread* ownerThread;
    QElapsedTimer datagramTimer;
    int lastSentTOWSeconds = -1;   //!< last "full" second value sent to the lidar for sync. Negative values: not sent.

    FastCRC16 fastCRC16;
    FastCRC32 fastCRC32;

public slots:
    void readPendingDatagrams_ControlCommand();
    void readPendingDatagrams_PushCommand();
    void readPendingDatagrams_PointCloudData();
    void readPendingDatagrams_IMUData();
    void ubxMessageReceived_RoverA(const UBXMessage& ubxMessage);
};



#endif // LIVOXMID360THREAD_H
