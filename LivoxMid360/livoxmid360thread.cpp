/*
    livoxmid360thread.cpp (part of GNSS-Stylus)
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

#include "livoxmid360thread.h"
#include "livoxmid360pointcloudandimudata.h"

LivoxMid360Thread::LivoxMid360Thread(const quint32 hostIPAddress, UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA, const bool replayMode, QVector<quint32> allowedDeviceIPs)
{
    this->hostIPAddress = hostIPAddress;
    this->ubloxDataStreamProcessor_RoverA = ubloxDataStreamProcessor_RoverA;
    this->replayMode = replayMode;
    this->allowedDeviceIPs = allowedDeviceIPs;
    suspended = false;
}

LivoxMid360Thread::~LivoxMid360Thread()
{
    terminateRequest = true;
    this->wait(5000);
}

void LivoxMid360Thread::suspend(void)
{
    suspended = true;
}

void LivoxMid360Thread::resume(void)
{
    suspended = false;
}

void LivoxMid360Thread::requestTerminate(void)
{
    exit();
    terminateRequest = true;
}

/*
LivoxMid360Thread::Statistics LivoxMid360Thread::getStatistics()
{
    Statistics stats;



    return stats;
}
*/

void LivoxMid360Thread::run()
{
    if (replayMode)
    {
        // This is very ugly way to handle replay data. I didn't figure out quickly how to handle two different "data sources" nicely,
        // without copy-pasting code and/or implementing two separate "data paths" for replay and UDP-data.
        // So now just running this "dummy" thread when replaying. Could also create the thread without running, but easier this way...
        // TODO: Make this nicer...
        emit infoMessage("Thread started in replay mode.");

        while (!terminateRequest)
        {
            msleep(100);
        }

        emit infoMessage("Thread terminated.");
        return;
    }

    LivoxMid360DatagramHandler* datagramHandler = new LivoxMid360DatagramHandler(this);
//    datagramHandler->run();

    udpSocket_ControlCommand = new QUdpSocket;
    udpSocket_PushCommand = new QUdpSocket;
    udpSocket_PointCloudData = new QUdpSocket;
    udpSocket_IMUData = new QUdpSocket;

    QHostAddress address = QHostAddress(hostIPAddress);

    while (!terminateRequest)
    {
        emit infoMessage("Binding UDP socket to address " + address.toString() + ", control command port " + QString::number(controlCommandPort_Host) + "...");
        if (udpSocket_ControlCommand->bind(address, controlCommandPort_Host))
        {
            emit infoMessage("Binding successful.");
            break;
        }
        emit warningMessage("Binding failed. Sleeping for 1 s and trying again...");
        sleep(1);
        suspendIfNeeded();
    }

    while (!terminateRequest)
    {
        emit infoMessage("Binding UDP socket to address " + address.toString() + ", push command port " + QString::number(pushCommandPort_Host) + "...");
        if (udpSocket_PushCommand->bind(address, pushCommandPort_Host))
        {
            emit infoMessage("Binding successful.");
            break;
        }
        emit warningMessage("Binding failed. Sleeping for 1 s and trying again...");
        sleep(1);
        suspendIfNeeded();
    }

    while (!terminateRequest)
    {
        emit infoMessage("Binding UDP socket to address " + address.toString() + ", point cloud data port " + QString::number(pointCloudDataPort_Host) + "...");
        if (udpSocket_PointCloudData->bind(address, pointCloudDataPort_Host))
        {
            emit infoMessage("Binding successful.");
            break;
        }
        emit warningMessage("Binding failed. Sleeping for 1 s and trying again...");
        sleep(1);
        suspendIfNeeded();
    }

    while (!terminateRequest)
    {
        emit infoMessage("Binding UDP socket to address " + address.toString() + ", IMU data port " + QString::number(imuDataPort_Host) + "...");
        if (udpSocket_IMUData->bind(address, imuDataPort_Host))
        {
            emit infoMessage("Binding successful.");
            break;
        }
        emit warningMessage("Binding failed. Sleeping for 1 s and trying again...");
        sleep(1);
        suspendIfNeeded();
    }

    connect(udpSocket_ControlCommand, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_ControlCommand);

    connect(udpSocket_PushCommand, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PushCommand);    

    connect(udpSocket_PointCloudData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PointCloudData);

    connect(udpSocket_IMUData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_IMUData);

    connect(ubloxDataStreamProcessor_RoverA, &UBloxDataStreamProcessor::ubxMessageReceived,
            datagramHandler, &LivoxMid360DatagramHandler::ubxMessageReceived_RoverA);

//    connect(udpSocket_PushCommand, &QUdpSocket::readyRead,
//            datagramHandler, &DatagramHandler::readPendingDatagrams_PushCommand);

//    connect(udpSocket_PushCommand, &QUdpSocket::readyRead,
//            thread(), [=](){ datagramHandler->readPendingDatagrams_PushCommand(); });

//    connect(udpSocket_PushCommand, SIGNAL(&QUdpSocket::readyRead()),
//            datagramHandler, SLOT(&DatagramHandler::readPendingDatagrams_PushCommand()));

//    DatagramHandler* datagramHandler = new DatagramHandler(this);
/*
    qDebug() << "Test";

    connect(udpSocket_PushCommand, &QUdpSocket::readyRead,
            thread(), [=](){ emit infoMessage("Lambda saatana"); });

    connect(udpSocket_PushCommand, &QUdpSocket::readyRead,
            [=](){ qDebug() << "Lambda saatana"; });

    connect(udpSocket_PointCloudData, &QUdpSocket::readyRead,
            datagramHandler, &DatagramHandler::readPendingDatagrams_PointCloudData);

    connect(udpSocket_IMUData, &QUdpSocket::readyRead,
            datagramHandler, &DatagramHandler::readPendingDatagrams_IMUData);
*/
//    emit infoMessage("Main thread ptr: " + QString::number(qlonglong(currentThread())));


/*    while (!terminateRequest)
    {
//        datagramHandler->readPendingDatagrams_PushCommand();
        msleep(100);
    }
*/

    if (!terminateRequest)
    {
        exec();
    }

    disconnect(udpSocket_ControlCommand, &QUdpSocket::readyRead,
               datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_ControlCommand);

    disconnect(udpSocket_PushCommand, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PushCommand);

    disconnect(udpSocket_PointCloudData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PointCloudData);

    disconnect(udpSocket_IMUData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_IMUData);

    disconnect(ubloxDataStreamProcessor_RoverA, &UBloxDataStreamProcessor::ubxMessageReceived,
            datagramHandler, &LivoxMid360DatagramHandler::ubxMessageReceived_RoverA);

    udpSocket_ControlCommand->close();
    udpSocket_PushCommand->close();
    udpSocket_PointCloudData->close();
    udpSocket_IMUData->close();

    delete udpSocket_ControlCommand;
    delete udpSocket_PushCommand;
    delete udpSocket_PointCloudData;
    delete udpSocket_IMUData;
    delete datagramHandler;

    udpSocket_ControlCommand = nullptr;
    udpSocket_PushCommand = nullptr;
    udpSocket_PointCloudData = nullptr;
    udpSocket_IMUData = nullptr;
    datagramHandler = nullptr;

    emit infoMessage("Thread terminated.");
}



bool LivoxMid360Thread::suspendIfNeeded(void)
{
    bool wasSuspended = false;

    if (suspended && !terminateRequest)
    {
        emit infoMessage("Suspending...");
        while (suspended && !terminateRequest)
        {
            wasSuspended = true;
            msleep(100);
        }
        if (!terminateRequest)
        {
            emit infoMessage("Resuming...");
        }
    }

    return wasSuspended;
}

void LivoxMid360Thread::DeviceCounters::init(void)
{
    totalDatagrams = 0;
    totalDatagramBytes = 0;

    pushLidarInformationDatagrams = 0;
    pushLidarInformationDatagramBytes = 0;

    pointCloudDataDatagrams = 0;
    pointCloudDataDatagramBytes = 0;
    pointCloudPoints = 0;

    imuDataDatagrams = 0;
    imuDataDatagramBytes = 0;
}

LivoxMid360Thread::DeviceCounters LivoxMid360Thread::DeviceCounters::operator - (LivoxMid360Thread::DeviceCounters const& subtractor)
{
    LivoxMid360Thread::DeviceCounters ret;

    ret.totalDatagrams = totalDatagrams - subtractor.totalDatagrams;
    ret.totalDatagramBytes = totalDatagramBytes - subtractor.totalDatagramBytes;

    ret.pushLidarInformationDatagrams = pushLidarInformationDatagrams - subtractor.pushLidarInformationDatagrams;
    ret.pushLidarInformationDatagramBytes = pushLidarInformationDatagramBytes - subtractor.pushLidarInformationDatagramBytes;

    ret.pointCloudDataDatagrams = pointCloudDataDatagrams - subtractor.pointCloudDataDatagrams;
    ret.pointCloudDataDatagramBytes = pointCloudDataDatagramBytes - subtractor.pointCloudDataDatagramBytes;
    ret.pointCloudPoints = pointCloudPoints - subtractor.pointCloudPoints;

    ret.imuDataDatagrams = imuDataDatagrams - subtractor.imuDataDatagrams;
    ret.imuDataDatagramBytes = imuDataDatagramBytes - subtractor.imuDataDatagramBytes;

    return ret;
}

LivoxMid360Thread::DeviceCounters LivoxMid360Thread::DeviceCounters::operator + (LivoxMid360Thread::DeviceCounters const& additor)
{
    LivoxMid360Thread::DeviceCounters ret;

    ret.totalDatagrams = totalDatagrams + additor.totalDatagrams;
    ret.totalDatagramBytes = totalDatagramBytes + additor.totalDatagramBytes;

    ret.pushLidarInformationDatagrams = pushLidarInformationDatagrams + additor.pushLidarInformationDatagrams;
    ret.pushLidarInformationDatagramBytes = pushLidarInformationDatagramBytes + additor.pushLidarInformationDatagramBytes;

    ret.pointCloudDataDatagrams = pointCloudDataDatagrams + additor.pointCloudDataDatagrams;
    ret.pointCloudDataDatagramBytes = pointCloudDataDatagramBytes + additor.pointCloudDataDatagramBytes;
    ret.pointCloudPoints = pointCloudPoints + additor.pointCloudPoints;

    ret.imuDataDatagrams = imuDataDatagrams + additor.imuDataDatagrams;
    ret.imuDataDatagramBytes = imuDataDatagramBytes + additor.imuDataDatagramBytes;

    return ret;
}


QMap<quint32, LivoxMid360Thread::DeviceCounters> LivoxMid360Thread::getDeviceCounters(void)
{
    QMap<quint32, DeviceCounters> retval;

    deviceCountersMutex.lock();
    retval = deviceCounters;
    deviceCountersMutex.unlock();

    return retval;
}

void LivoxMid360Thread::clearDeviceCounters(void)
{
    deviceCountersMutex.lock();
    deviceCounters.clear();
    deviceCountersMutex.unlock();
}

// Thread-safe datagram handler method that can be used from the UDP-thread and replay
void LivoxMid360Thread::handleDatagram_ControlCommand(const QNetworkDatagram& datagram, qint64 uptime)
{
    bool isIPV4Address;
    quint32 senderAddress = datagram.senderAddress().toIPv4Address(&isIPV4Address);
    if ((!isIPV4Address) || suspended)
    {
        // Just discard datagram if sender address is not IPV4 (should not happen, though)
        // (also when thread is suspended)
        return;
    }

    // Check that datagram is coming from a device that is in the list of allowed devices
    if (!checkDeviceValidity(senderAddress))
    {
        // Device not found in the allowed list. Just discard data.
        return;
    }

    // Check that the datagram can be interpreted as a valid "control command" before emitting it.
    LivoxMid360::ControlCommand controlCommand(datagram);

    if (controlCommand.status != LivoxMid360::ControlCommand::STATUS_VALID)
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("Control command datagram invalid. Status: " + QString::number(controlCommand.status) + ". Ignoring the datagram.");
        return;
    }

//    if (controlCommand.cmd_id == LivoxMid360::ControlCommand::SET_GPS_TIMESTAMP)
//    {
        // TODO: Maybe add check that set GPS timestamp is acknowledged by all devices(?)
//    }

    emit rawDatagramReceived(datagram, uptime);
}


// Thread-safe datagram handler method that can be used from the UDP-thread and replay
void LivoxMid360Thread::handleDatagram_PushCommand(const QNetworkDatagram& datagram, qint64 uptime)
{
    bool isIPV4Address;
    quint32 senderAddress = datagram.senderAddress().toIPv4Address(&isIPV4Address);
    if ((!isIPV4Address) || suspended)
    {
        // Just discard datagram if sender address is not IPV4 (should not happen, though)
        // (also when thread is suspended)
        return;
    }

    // Check that datagram is coming from a device that is in the list of allowed devices
    if (!checkDeviceValidity(senderAddress))
    {
        // Device not found in the allowed list. Just discard data.
        return;
    }

    if (detectedDeviceIPs.value(senderAddress) != DT_ENABLED)
    {
        // Device already detected and not found in the allowed list. Just discard data.
        return;
    }

    // Check that the datagram can be interpreted as a valid "control command" before emitting it.
    LivoxMid360::ControlCommand controlCommand(datagram);

    if (controlCommand.status != LivoxMid360::ControlCommand::STATUS_VALID)
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("Control command datagram invalid. Status: " + QString::number(controlCommand.status) + ". Ignoring the datagram.");
        return;
    }

    if (controlCommand.cmd_id == LivoxMid360::ControlCommand::PUSH_LIDAR_INFORMATION)
    {
        LivoxMid360::PushLidarInformation pushInfo(datagram);

        if (pushInfo.status != LivoxMid360::ControlCommand::STATUS_VALID)
        {
            // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
            emit warningMessage("Push lidar information datagram invalid (can't cast to push info). Status: " + QString::number(pushInfo.status) + ". Ignoring the datagram.");
            return;
        }

        deviceCountersMutex.lock();

        LivoxMid360Thread::DeviceCounters counters;

        if (deviceCounters.contains(senderAddress))
        {
            counters = deviceCounters.value(senderAddress);
        }
        else
        {
            counters.init();
        }

        counters.totalDatagrams++;
        counters.totalDatagramBytes += datagram.data().size();
        counters.pushLidarInformationDatagrams++;
        counters.pushLidarInformationDatagramBytes += datagram.data().size();

        deviceCounters[senderAddress] = counters;

        deviceCountersMutex.unlock();

        emit pushLidarInformationReceived(datagram.senderAddress().toIPv4Address(), pushInfo, uptime);
    }

    emit rawDatagramReceived(datagram, uptime);
}

// Thread-safe datagram handler method that can be used from the UDP-thread and replay
void LivoxMid360Thread::handleDatagram_PointCloudData(const QNetworkDatagram& datagram, qint64 uptime)
{
    bool isIPV4Address;
    quint32 senderAddress = datagram.senderAddress().toIPv4Address(&isIPV4Address);
    if ((!isIPV4Address) || suspended)
    {
        // Just discard the datagram if sender address is not IPV4 (should not happen, though)
        // (also when thread is suspended)
        return;
    }

    // Check that datagram is coming from a device that is in the list of allowed devices
    if (!checkDeviceValidity(senderAddress))
    {
        // Device not found in the allowed list. Just discard data.
        return;
    }

    // Check that the datagram can be interpreted as a valid "control command" before emitting it.
    LivoxMid360::PointCloudAndIMUDataHeader header(datagram);

    if (header.status != LivoxMid360::PointCloudAndIMUDataHeader::STATUS_VALID)
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("Point cloud datagram invalid. Status: " + QString::number(header.status) + ". Ignoring the datagram.");
        return;
    }

    if ((header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT) &&
        (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
        (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL))
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("Datagram not being point cloud data received into point cloud port. Ignoring the datagram.");
        return;
    }

    deviceCountersMutex.lock();

    LivoxMid360Thread::DeviceCounters counters;

    if (deviceCounters.contains(senderAddress))
    {
        counters = deviceCounters.value(senderAddress);
    }
    else
    {
        counters.init();
    }

    counters.totalDatagrams++;
    counters.totalDatagramBytes += datagram.data().size();
    counters.pointCloudDataDatagrams++;
    counters.pointCloudDataDatagramBytes += datagram.data().size();
    counters.pointCloudPoints += header.dot_num;

    deviceCounters[senderAddress] = counters;

    deviceCountersMutex.unlock();

    emit rawDatagramReceived(datagram, uptime);
}

// Thread-safe datagram handler method that can be used from the UDP-thread and replay
void LivoxMid360Thread::handleDatagram_IMUData(const QNetworkDatagram& datagram, qint64 uptime)
{
    bool isIPV4Address;
    quint32 senderAddress = datagram.senderAddress().toIPv4Address(&isIPV4Address);
    if ((!isIPV4Address) || suspended)
    {
        // Just discard datagram if sender address is not IPV4 (should not happen, though)
        // (also when thread is suspended)
        return;
    }

    // Check that datagram is coming from a device that is in the list of allowed devices
    if (!checkDeviceValidity(senderAddress))
    {
        // Device not found in the allowed list. Just discard data.
        return;
    }

    // Check that the datagram can be interpreted as a valid "control command" before emitting it.
    LivoxMid360::PointCloudAndIMUDataHeader header(datagram);

    if (header.status != LivoxMid360::PointCloudAndIMUDataHeader::STATUS_VALID)
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("IMU datagram invalid. Status: " + QString::number(header.status) + ". Ignoring the datagram.");
        return;
    }

    if (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_IMU)
    {
        // too lazy to make statuses readable. Refer to the header file (MessageDataStatus) if parsing fails.
        emit warningMessage("Datagram not being IMU data received into IMU port. Ignoring the datagram.");
        return;
    }


    deviceCountersMutex.lock();

    LivoxMid360Thread::DeviceCounters counters;

    if (deviceCounters.contains(senderAddress))
    {
        counters = deviceCounters.value(senderAddress);
    }
    else
    {
        counters.init();
    }

    counters.totalDatagrams++;
    counters.totalDatagramBytes += datagram.data().size();
    counters.imuDataDatagrams++;
    counters.imuDataDatagramBytes += datagram.data().size();

    deviceCounters[senderAddress] = counters;

    deviceCountersMutex.unlock();

    emit rawDatagramReceived(datagram, uptime);
}

void LivoxMid360Thread::on_rawReplayDatagramReceived(const QNetworkDatagram& datagram, qint64 timeStamp)
{
    quint16 destinationPort = datagram.destinationPort();

    if (destinationPort == controlCommandPort_Host)
    {
        handleDatagram_ControlCommand(datagram, timeStamp);
    }
    else if (destinationPort == pushCommandPort_Host)
    {
        handleDatagram_PushCommand(datagram, timeStamp);
    }
    else if (destinationPort == pointCloudDataPort_Host)
    {
        handleDatagram_PointCloudData(datagram, timeStamp);
    }
    else if (destinationPort == imuDataPort_Host)
    {
        handleDatagram_IMUData(datagram, timeStamp);
    }
    else
    {
        emit errorMessage("replay data handler called with invalid datagram. Destination port " +
                          QString::number(destinationPort) +
                          " not supported port (Push command, point cloud or IMU data). Datagram not handled.");
    }
}

void LivoxMid360Thread::connectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    connect(postProcessingForm, &PostProcessingForm::replayData_LivoxMid360,
                this, &LivoxMid360Thread::on_rawReplayDatagramReceived);
}

void LivoxMid360Thread::disconnectPostProcessingSlots(PostProcessingForm* postProcessingForm)
{
    disconnect(postProcessingForm, &PostProcessingForm::replayData_LivoxMid360,
            this, &LivoxMid360Thread::on_rawReplayDatagramReceived);
}

bool LivoxMid360Thread::checkDeviceValidity(const quint32 senderAddress)
{
    // Check that datagram is coming from a device that is in the list of allowed devices
    if (!detectedDeviceIPs.contains(senderAddress))
    {
        // This is the first time this device sends the push command

        if (allowedDeviceIPs.contains(senderAddress))
        {
            emit infoMessage("New device detected, IP: " + QHostAddress(senderAddress).toString() + ". Device listed as allowed, starting receiving data.");
            detectedDeviceIPs.insert(senderAddress, DT_ENABLED);
        }
        else
        {
            emit infoMessage("New device detected, IP: " + QHostAddress(senderAddress).toString() + ". Device not listed as allowed, discarding data (every device reported only once during the thread lifetime).");
            detectedDeviceIPs.insert(senderAddress, DT_DISCARDED);
        }
    }

    return (detectedDeviceIPs.value(senderAddress) == DT_ENABLED);
}


void LivoxMid360DatagramHandler::readPendingDatagrams_ControlCommand()
{
    //    emit ownerThread->infoMessage("Control command datagram handler ThreadPtr: " + QString::number(qlonglong(ownerThread->currentThread())));
    //    emit ownerThread->infoMessage("Control command datagram handler called");
    while (ownerThread->udpSocket_ControlCommand->hasPendingDatagrams()) {
        datagramTimer.start();
        quint64 uptime = datagramTimer.msecsSinceReference();
        QNetworkDatagram datagram = ownerThread->udpSocket_ControlCommand->receiveDatagram();
        ownerThread->handleDatagram_ControlCommand(datagram, uptime);
    }
}

void LivoxMid360DatagramHandler::readPendingDatagrams_PushCommand()
{
//    emit ownerThread->infoMessage("Push command datagram handler ThreadPtr: " + QString::number(qlonglong(ownerThread->currentThread())));
//    emit ownerThread->infoMessage("Push command datagram handler called");
    while (ownerThread->udpSocket_PushCommand->hasPendingDatagrams()) {
        datagramTimer.start();
        quint64 uptime = datagramTimer.msecsSinceReference();
        QNetworkDatagram datagram = ownerThread->udpSocket_PushCommand->receiveDatagram();
        ownerThread->handleDatagram_PushCommand(datagram, uptime);
    }
}

void LivoxMid360DatagramHandler::readPendingDatagrams_PointCloudData()
{
    //    emit ownerThread->infoMessage("Point cloud datagram handler called");
    while (ownerThread->udpSocket_PointCloudData->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ownerThread->udpSocket_PointCloudData->receiveDatagram();
        quint64 uptime = datagramTimer.msecsSinceReference();
        datagramTimer.start();
        ownerThread->handleDatagram_PointCloudData(datagram, uptime);
    }
}

void LivoxMid360DatagramHandler::readPendingDatagrams_IMUData()
{
    //    emit ownerThread->infoMessage("IMU datagram handler called");
    while (ownerThread->udpSocket_IMUData->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ownerThread->udpSocket_IMUData->receiveDatagram();
        datagramTimer.start();
        quint64 uptime = datagramTimer.msecsSinceReference();
        ownerThread->handleDatagram_IMUData(datagram, uptime);
    }
}

void LivoxMid360DatagramHandler::ubxMessageReceived_RoverA(const UBXMessage& ubxMessage)
{
    UBXMessage_RELPOSNED relposned(ubxMessage);

    if (relposned.messageDataStatus == UBXMessage::STATUS_VALID)
    {
        // "Casting" generic UBX-message to RELPOSNED was successful
        // We are only interested in ITOW-time here.

        int currTOWSecs = relposned.iTOW / 1000;

        if (currTOWSecs != lastSentTOWSeconds)
        {
            emit ownerThread->infoMessage("New rover ITOW: " + QString::number(currTOWSecs));
            lastSentTOWSeconds = currTOWSecs;

            quint64 nanoTOW = quint64(currTOWSecs) * 1000000000L;

//            nanoTOW |= quint64(1) << 62;

            QByteArray array = LivoxMid360::SetGPSTimestamp::createSetCommand(nanoTOW, ownerThread->setGPSTimestampSeqNum++);

            LivoxMid360::ControlCommand cc(QNetworkDatagram(array,QHostAddress()));

            for (auto i = ownerThread->detectedDeviceIPs.cbegin(); i != ownerThread->detectedDeviceIPs.cend(); i++)
            {
                if (i.value() == LivoxMid360Thread::DT_ENABLED)
                {
                    ownerThread->udpSocket_ControlCommand->writeDatagram(array, QHostAddress(i.key()), ownerThread->controlCommandPort_Lidar);
//                    ownerThread->udpSocket_ControlCommand->writeDatagram(array, QHostAddress(0xFFFFFFFF), ownerThread->controlCommandPort_Lidar);
                }
            }
        }
    }
}
