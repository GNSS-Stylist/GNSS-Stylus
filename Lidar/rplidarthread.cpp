/*
    rplidarthread.cpp (part of GNSS-Stylus)
    Copyright (C) 2020 Pasi Nuutinmaki (gnssstylist<at>sci<dot>fi)

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

#include "rplidarthread.h"
#include <math.h>

RPLidarThread::RPLidarThread(const QString& serialPortFileName, const unsigned int serialPortBPS, const unsigned short motorPWM)
{
    this->serialPortFileName = serialPortFileName;
    this->serialPortBPS = serialPortBPS;
    this->motorPWM = motorPWM;

    terminateRequest = false;
    suspended = false;

    qRegisterMetaType<QVector<RPLidarThread::DistanceItem>>();
}

RPLidarThread::~RPLidarThread()
{
    terminateRequest = true;
    this->wait(5000);
}

void RPLidarThread::run()
{
    using namespace rp::standalone::rplidar;

    RPlidarDriver* lidarDriver = RPlidarDriver::CreateDriver();

    if (!lidarDriver)
    {
        emit errorMessage("RPlidarDriver::CreateDriver() failed. Can't do anything.");
        emit infoMessage("Thread terminated.");
        return;
    }

    u_result rpResult;

    while (!terminateRequest)
    {
        if (!terminateRequest)
        {
            do
            {
                suspendIfNeeded();

                emit infoMessage("Connecting RPLidar, port " + serialPortFileName + ", speed: " + QString::number(serialPortBPS) + "...");

                rpResult = lidarDriver->connect(serialPortFileName.toLocal8Bit(), serialPortBPS);

                if (IS_FAIL(rpResult))
                {
                    emit errorMessage("Connection failed, error: " + getRPLidarResultString(rpResult));
                    emit infoMessage("Retrying after 1 s...");

                    sleep(1);
                }
            } while (IS_FAIL(rpResult) && !terminateRequest);
        }

        if (!terminateRequest)
        {
            rplidar_response_device_health_t health;
            rpResult = lidarDriver->getHealth(health);
            if (IS_FAIL(rpResult))
            {
                emit errorMessage("getHealth failed, error: " + getRPLidarResultString(rpResult));
            }
            else
            {
                emit infoMessage("getHealth() status: " + QString::number(health.status) + ", errorCode: " + QString::number(health.error_code));
            }
        }

        if (!terminateRequest)
        {
            rplidar_response_device_info_t info;
            rpResult = lidarDriver->getDeviceInfo(info);

            if (IS_FAIL(rpResult))
            {
                emit errorMessage("getDeviceInfo failed, error: " + getRPLidarResultString(rpResult));
            }
            else
            {
                QString serNum;

                for (unsigned int i = 0; i < 16; i++)
                {
                    serNum += QString::number(info.serialnum[i], 16).toUpper();
                }

                emit infoMessage("getDeviceInfo() model: " + QString::number(info.model) +
                                 ", firmware_version: " + QString::number(info.firmware_version) +
                                 ", hardware_version: " + QString::number(info.hardware_version) +
                                 ", serialnum: " + serNum);
            }
        }

        if (!terminateRequest)
        {
            do
            {
                suspendIfNeeded();

                emit infoMessage("Starting motor...");

                rpResult = lidarDriver->startMotor();

                if (IS_FAIL(rpResult))
                {
                    emit errorMessage("Starting motor failed, error: " + getRPLidarResultString(rpResult));
                    emit infoMessage("Retrying after 1 s...");

                    sleep(1);
                }
            } while (IS_FAIL(rpResult) && !terminateRequest);
        }

        if (!terminateRequest)
        {
            emit infoMessage("Checking motor control support...");
            bool motorCtrlSupport;
            rpResult = lidarDriver->checkMotorCtrlSupport(motorCtrlSupport, 1000);

            if (IS_FAIL(rpResult))
            {
                emit warningMessage("Motor control support function call failed, not setting motor PWM.");
            }
            else if (!motorCtrlSupport)
            {
                emit warningMessage("Motor control not supported, not setting motor PWM.");
            }
            else if (!terminateRequest)
            {
                do
                {
                    suspendIfNeeded();

                    emit infoMessage("Setting motor PWM...");

                    rpResult = lidarDriver->setMotorPWM(motorPWM);

                    if (IS_FAIL(rpResult))
                    {
                        emit errorMessage("Setting motor PWM failed, error: " + getRPLidarResultString(rpResult));
                        emit infoMessage("Retrying after 1 s...");

                        sleep(1);
                    }
                } while (IS_FAIL(rpResult) && !terminateRequest);
            }
        }

        if (!terminateRequest)
        {
            do
            {
                suspendIfNeeded();

                emit infoMessage("Starting scan...");

                rpResult = lidarDriver->startScan(false, true);

                if (IS_FAIL(rpResult))
                {
                    emit errorMessage("Starting scan failed, error: " + getRPLidarResultString(rpResult));
                    emit infoMessage("Retrying after 1 s...");

                    sleep(1);
                }
            } while (IS_FAIL(rpResult) && !terminateRequest);
        }


//        while (!terminateRequest && IS_FAIL(lidarDriver->connect(serialPortFileName.toLocal8Bit(), serialPortBPS));

//        lidarDriver->startScan(true, true);
//        lidarDriver->startMotor();

//        if (lidarDriver->checkMotorCtrlSupport())
//        lidarDriver->setMotorPWM(motorPWM);

        emit infoMessage("Reading data...");
//        msleep(5000);

//        rplidar_response_measurement_node_hq_t meas;

        QElapsedTimer timer;

        timer.start();
        qint64 prevUptime = timer.msecsSinceReference();

        while (!terminateRequest)
        {
            size_t count = sizeof(measBuffer) / sizeof(measBuffer[0]);
            if (suspendIfNeeded())
            {
                // Flush buffer before continuing
                lidarDriver->grabScanDataHq(measBuffer, count);
            }

            lidarDriver->grabScanDataHq(measBuffer, count);
            timer.start();
            qint64 newUptime = timer.msecsSinceReference();

            QVector<DistanceItem> distanceData;

            for (unsigned int i = 0; i < count; i++)
            {
                DistanceItem newItem;
                newItem.angle = 2 * M_PI * measBuffer[i].angle_z_q14 / 65536.;
                newItem.distance = measBuffer[i].dist_mm_q2 / 4000.;
                newItem.quality = measBuffer[i].quality / 255.;
                distanceData.push_back(newItem);
            }

            emit distanceRoundReceived(distanceData, prevUptime, newUptime);
            prevUptime = newUptime;
        }

        emit infoMessage("Stopping motor...");
        lidarDriver->stopMotor();

        emit infoMessage("Disconnecting...");
        lidarDriver->disconnect();
    }

    emit infoMessage("Thread terminated.");

    RPlidarDriver::DisposeDriver(lidarDriver);
}

void RPLidarThread::suspend(void)
{
    suspended = true;
}

void RPLidarThread::resume(void)
{
    suspended = false;
}

void RPLidarThread::requestTerminate(void)
{
    terminateRequest = true;
}

struct
{
    u_result res;
    QString text;
} static const resultStrings[] =
{
    { RESULT_OK, "OK" },
    { RESULT_FAIL_BIT, "Only fail bit set" },
    { RESULT_ALREADY_DONE, "Already done" },
    { RESULT_INVALID_DATA, "Invalid data" },
    { RESULT_OPERATION_FAIL, "Operation fail" },
    { RESULT_OPERATION_TIMEOUT, "Operation timeout" },
    { RESULT_OPERATION_STOP, "Operation stop" },
    { RESULT_OPERATION_NOT_SUPPORT, "Operation not support(ed?)" },
    { RESULT_FORMAT_NOT_SUPPORT, "Format not support(ed?)" },
    { RESULT_INSUFFICIENT_MEMORY, "Insufficient memory" }
};


QString RPLidarThread::getRPLidarResultString(const u_result result)
{
    for (unsigned int i = 0; i < sizeof(resultStrings) / sizeof(resultStrings[0]); i++)
    {
        if (resultStrings[i].res == result)
        {
            return resultStrings[i].text + " (0x" + QString::number(result, 16).toUpper() + ")";
        }
    }

    return "Unknown result (0x" + QString::number(result, 16).toUpper() + ")";
}

bool RPLidarThread::suspendIfNeeded(void)
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


