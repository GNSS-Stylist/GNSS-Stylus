#ifndef LIVOXMID360THREAD_H
#define LIVOXMID360THREAD_H

#include <QObject>
#include <QThread>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QUdpSocket>
#include <QElapsedTimer>

#include "ubloxdatastreamprocessor.h"
#include "FastCRC/FastCRC.h"

class LivoxMid360Thread : public QThread
{
    Q_OBJECT

public:
    LivoxMid360Thread(const quint32 hostIPAddress, UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA);
    ~LivoxMid360Thread() override;

    friend class LivoxMid360DatagramHandler;

    void run() override;            //!< Thread code
    void requestTerminate(void);    //!< Requests thread to terminate
    void suspend(void);             //!< Requests thread to suspend (ignoring all incoming packets). Suspend may not be immediate
    void resume(void);              //!< Requests thread to resume (from suspend). Resuming may not be immediate.

    /*
    class Statistics
    {
    public:
        class DatagramCounts
        {
        public:
            quint64 total = 0;
            quint64 pushCommand = 0;
            quint64 pointCloudData = 0;
            quint64 imuData = 0;
        };

        DatagramCounts totalCounters;
        QMap<QHostAddress, DatagramCounts> DeviceCounters;
    };

    Statistics getStatistics();
    */

    void connectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Connects signals from UBloxDataStreamProcessor
    void disconnectUBloxDataStreamProcessorSlots_Rover(UBloxDataStreamProcessor* ubloxDataStreamProcessor); //!< Disconnects signals from UBloxDataStreamProcessor

/*    enum CoordType
    {
        CT_CARTESIAN_32 = 0,
        CT_CARTESIAN_16,
        CT_SPHERICAL
    };
    Q_ENUM(CoordType)
*/
    // Currently expecting the mid-360 to be configured correctly already and just listening for the data
    // Therefore only 3 listening ports are used for now (for "push command" (=lidar info), points and IMU data).
    //const quint16 controlCommandPort_Host = 56101;
    //const quint16 controlCommandPort_Lidar = 56100;
    //const quint16 pushCommandPort_Lidar = 56200;
    const quint16 pushCommandPort_Host = 56201;
    //const quint16 pointCloudDataPort_Lidar = 56300;
    const quint16 pointCloudDataPort_Host = 56301;
    //const quint16 imuDataPort_Lidar = 56400;
    const quint16 imuDataPort_Host = 56401;

private:
//    class DatagramHandler* datagramHandler = nullptr;

    quint32 hostIPAddress;

    bool terminateRequest = false;
    bool suspended = false;

    bool suspendIfNeeded(void);

    QUdpSocket* udpSocket_PushCommand = nullptr;
    QUdpSocket* udpSocket_PointCloudData = nullptr;
    QUdpSocket* udpSocket_IMUData = nullptr;
    UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA = nullptr;

signals:
    void infoMessage(const QString&);       //!< Signal for info-message (not warning or error)
    void warningMessage(const QString&);    //!< Signal for warning message (less severe than error)
    void errorMessage(const QString&);      //!< Signal for error message

    void rawDatagramReceived(const QNetworkDatagram&, qint64);
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
    void readPendingDatagrams_PushCommand();
    void readPendingDatagrams_PointCloudData();
    void readPendingDatagrams_IMUData();
    void ubxMessageReceived_RoverA(const UBXMessage& ubxMessage);
};



#endif // LIVOXMID360THREAD_H
