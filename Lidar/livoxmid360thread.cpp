#include "livoxmid360thread.h"

LivoxMid360Thread::LivoxMid360Thread(const quint32 hostIPAddress, UBloxDataStreamProcessor* ubloxDataStreamProcessor_RoverA)
{
    this->hostIPAddress = hostIPAddress;
    this->ubloxDataStreamProcessor_RoverA = ubloxDataStreamProcessor_RoverA;
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
    LivoxMid360DatagramHandler* datagramHandler = new LivoxMid360DatagramHandler(this);
//    datagramHandler->run();

    udpSocket_PushCommand = new QUdpSocket;
    udpSocket_PointCloudData = new QUdpSocket;
    udpSocket_IMUData = new QUdpSocket;

    QHostAddress address = QHostAddress(hostIPAddress);

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
    }

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
    emit infoMessage("Main thread ptr: " + QString::number(qlonglong(currentThread())));


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

    disconnect(udpSocket_PushCommand, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PushCommand);

    disconnect(udpSocket_PointCloudData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_PointCloudData);

    disconnect(udpSocket_IMUData, &QUdpSocket::readyRead,
            datagramHandler, &LivoxMid360DatagramHandler::readPendingDatagrams_IMUData);

    disconnect(ubloxDataStreamProcessor_RoverA, &UBloxDataStreamProcessor::ubxMessageReceived,
            datagramHandler, &LivoxMid360DatagramHandler::ubxMessageReceived_RoverA);

    udpSocket_PushCommand->close();
    udpSocket_PointCloudData->close();
    udpSocket_IMUData->close();

    delete udpSocket_PushCommand;
    delete udpSocket_PointCloudData;
    delete udpSocket_IMUData;
    delete datagramHandler;

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

void LivoxMid360DatagramHandler::readPendingDatagrams_PushCommand()
{
    emit ownerThread->infoMessage("Push command datagram handler ThreadPtr: " + QString::number(qlonglong(ownerThread->currentThread())));
    emit ownerThread->infoMessage("Push command datagram handler called");
    while (ownerThread->udpSocket_PushCommand->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ownerThread->udpSocket_PushCommand->receiveDatagram();

        if (datagram.data().size() < 24)
        {
            emit ownerThread->warningMessage("Push command datagram too short. Got " + QString::number(datagram.data().size()) + " bytes, minimum is 24 bytes. Ignoring the datagram.");
            continue;
        }

        const unsigned char *datagramData = (const unsigned char *)(datagram.data().constData());
        int datagramSize = datagram.data().size();

        quint16 headerCRC16_Calc = fastCRC16.ccitt(datagramData, 18);
        quint16 headerCRC16_Datagram = datagramData[18] | (datagramData[19] << 8);

        if (headerCRC16_Calc != headerCRC16_Datagram)
        {
            emit ownerThread->warningMessage("Push command datagram header CRC16 doesn't match. Ignoring the datagram.");
            continue;
        }

        quint16 datagramLength_Datagram = datagramData[2] | (datagramData[3] << 8);

        if (datagramSize != datagramLength_Datagram)
        {
            emit ownerThread->warningMessage("Push command datagram size doesn't match length-field of the header. Length in header: " + QString::number(datagramLength_Datagram) + ", datagram length: " + QString::number(datagram.data().size()) + ". Ignoring the datagram.");
            continue;
        }

        quint32 dataCRC32_Calc = fastCRC32.crc32(&datagramData[24], datagramLength_Datagram - 24);
        quint32 dataCRC32_Datagram = datagramData[20] | (datagramData[21] << 8) | (datagramData[22] << 16) | (datagramData[23] << 24);

        if (dataCRC32_Calc != dataCRC32_Datagram)
        {
            emit ownerThread->warningMessage("Push command datagram data CRC32 doesn't match. Ignoring the datagram.");
            continue;
        }

        datagramTimer.start();
        emit ownerThread->rawDatagramReceived(datagram, datagramTimer.msecsSinceReference());
        emit ownerThread->infoMessage("Push command datagram received.");

        QString dataString = "Data: ";

        for (int i = 0; i < datagram.data().size(); i++)
        {
            if (i != 0)
            {
                dataString += ", ";
            }
            dataString += QString::number(uint((unsigned char)(datagram.data().at(i))), 16);
        }

        emit ownerThread->infoMessage(dataString);

        // processTheDatagram(datagram);
    }
}

void LivoxMid360DatagramHandler::readPendingDatagrams_PointCloudData()
{
    //    emit ownerThread->infoMessage("Point cloud datagram handler called");
    while (ownerThread->udpSocket_PointCloudData->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ownerThread->udpSocket_PointCloudData->receiveDatagram();
        datagramTimer.start();
        emit ownerThread->rawDatagramReceived(datagram, datagramTimer.msecsSinceReference());
        //        emit ownerThread->infoMessage("Point cloud datagram received");
        // processTheDatagram(datagram);
    }
}

void LivoxMid360DatagramHandler::readPendingDatagrams_IMUData()
{
    //    emit ownerThread->infoMessage("IMU datagram handler called");
    while (ownerThread->udpSocket_IMUData->hasPendingDatagrams()) {
        QNetworkDatagram datagram = ownerThread->udpSocket_IMUData->receiveDatagram();
        datagramTimer.start();
        emit ownerThread->rawDatagramReceived(datagram, datagramTimer.msecsSinceReference());
        //        emit ownerThread->infoMessage("IMU datagram received");
        // processTheDatagram(datagram);
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
        }
    }
}
