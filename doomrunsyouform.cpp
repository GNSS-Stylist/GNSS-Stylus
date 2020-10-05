/*
    doomrunsyouform.cpp (part of GNSS-Stylus)
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

#include <stdio.h>
#include <conio.h>
#include <tchar.h>
#include <algorithm>
#include <QTime>
#include <QElapsedTimer>


#include "doomrunsyouform.h"
#include "ui_doomrunsyouform.h"

static const wchar_t* pipeName = L"\\\\.\\pipe\\DoomRunsYou";

DoomRunsYouForm::DoomRunsYouForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DoomRunsYouForm)
{
    ui->setupUi(this);

    QSettings settings;

    ui->doubleSpinBox_CoordMultiplier->setValue(settings.value("DoomRunsYou_CoordMultiplier", 32768).toDouble());
    ui->doubleSpinBox_LocationUpdateThreshold->setValue(settings.value("DoomRunsYou_LocationUpdateThreshold", 0.020).toDouble());

    ui->doubleSpinBox_BFGG_Camera_N->setValue(settings.value("DoomRunsYou_BFGG_Camera_N", -0.5).toDouble());
    ui->doubleSpinBox_BFGG_Camera_E->setValue(settings.value("DoomRunsYou_BFGG_Camera_E", 0).toDouble());
    ui->doubleSpinBox_BFGG_Camera_D->setValue(settings.value("DoomRunsYou_BFGG_Camera_D", -0.1).toDouble());

    ui->doubleSpinBox_BFGG_LookAt_N->setValue(settings.value("DoomRunsYou_BFGG_LookAt_N", 1).toDouble());
    ui->doubleSpinBox_BFGG_LookAt_E->setValue(settings.value("DoomRunsYou_BFGG_LookAt_E", 0).toDouble());
    ui->doubleSpinBox_BFGG_LookAt_D->setValue(settings.value("DoomRunsYou_BFGG_LookAt_D", -0.1).toDouble());

    ui->comboBox_MotionPredictorType->setCurrentIndex(settings.value("DoomRunsYou_MotionPredictorType", 0).toInt());

    ui->spinBox_MotionPredictTime->setValue(settings.value("DoomRunsYou_MotionPredictTime", 0).toInt());
    ui->doubleSpinBox_PostLPFilteringCoefficient->setValue(settings.value("DoomRunsYou_PostLPFilteringCoefficient", 1).toDouble());

    ui->spinBox_MaxLogLines->setValue(settings.value("DoomRunsYou_MaxLogLines", 1000).toInt());

    QElapsedTimer uptimeBaselineTimer;
    uptimeBaselineTimer.start();
    uptimeBaseline = uptimeBaselineTimer.msecsSinceReference();

    connect(&fastTickTimer, SIGNAL(timeout()), this, SLOT(fastTickTimerTimeout()));

    lineSeries_Yaw_Unfiltered = new QLineSeries();
    lineSeries_Yaw_Filtered = new QLineSeries();

    lineSeries_Pitch_Unfiltered = new QLineSeries();
    lineSeries_Pitch_Filtered = new QLineSeries();

    chart_Angles = new QChart();
    chart_Angles->legend()->hide();
    chart_Angles->setTitle("Yaw and pitch angles");

    chart_Angles->addSeries(lineSeries_Yaw_Unfiltered);
    chart_Angles->addSeries(lineSeries_Yaw_Filtered);

    chart_Angles->addSeries(lineSeries_Pitch_Unfiltered);
    chart_Angles->addSeries(lineSeries_Pitch_Filtered);

    xAxis_Angles = new QValueAxis;
    xAxis_Angles->setTitleText("uptime");
    yAxis_Angles_Yaw = new QValueAxis;
    yAxis_Angles_Yaw->setTitleText("Yaw (cont)");
    yAxis_Angles_Pitch = new QValueAxis;
    yAxis_Angles_Pitch->setTitleText("Pitch");

    chart_Angles->addAxis(xAxis_Angles, Qt::AlignBottom);
    chart_Angles->addAxis(yAxis_Angles_Yaw, Qt::AlignLeft);
    chart_Angles->addAxis(yAxis_Angles_Pitch, Qt::AlignRight);

    lineSeries_Yaw_Unfiltered->attachAxis(xAxis_Angles);
    lineSeries_Yaw_Unfiltered->attachAxis(yAxis_Angles_Yaw);

    lineSeries_Yaw_Filtered->attachAxis(xAxis_Angles);
    lineSeries_Yaw_Filtered->attachAxis(yAxis_Angles_Yaw);

    lineSeries_Pitch_Unfiltered->attachAxis(xAxis_Angles);
    lineSeries_Pitch_Unfiltered->attachAxis(yAxis_Angles_Pitch);

    lineSeries_Pitch_Filtered->attachAxis(xAxis_Angles);
    lineSeries_Pitch_Filtered->attachAxis(yAxis_Angles_Pitch);

    ui->chartView_MotionEstimator->setChart(chart_Angles);

    fastTickTimer.setTimerType(Qt::PreciseTimer);
    fastTickTimer.start(1);
}

DoomRunsYouForm::~DoomRunsYouForm()
{
    fastTickTimer.stop();

    QSettings settings;

    settings.setValue("DoomRunsYou_CoordMultiplier", ui->doubleSpinBox_CoordMultiplier->value());
    settings.setValue("DoomRunsYou_LocationUpdateThreshold", ui->doubleSpinBox_LocationUpdateThreshold->value());

    settings.setValue("DoomRunsYou_BFGG_Camera_N", ui->doubleSpinBox_BFGG_Camera_N->value());
    settings.setValue("DoomRunsYou_BFGG_Camera_E", ui->doubleSpinBox_BFGG_Camera_E->value());
    settings.setValue("DoomRunsYou_BFGG_Camera_D", ui->doubleSpinBox_BFGG_Camera_D->value());

    settings.setValue("DoomRunsYou_BFGG_LookAt_N", ui->doubleSpinBox_BFGG_LookAt_N->value());
    settings.setValue("DoomRunsYou_BFGG_LookAt_E", ui->doubleSpinBox_BFGG_LookAt_E->value());
    settings.setValue("DoomRunsYou_BFGG_LookAt_D", ui->doubleSpinBox_BFGG_LookAt_D->value());

    settings.setValue("DoomRunsYou_MotionPredictorType", ui->comboBox_MotionPredictorType->currentIndex());

    settings.setValue("DoomRunsYou_MotionPredictTime", ui->spinBox_MotionPredictTime->value());
    settings.setValue("DoomRunsYou_PostLPFilteringCoefficient", ui->doubleSpinBox_PostLPFilteringCoefficient->value());

    settings.setValue("DoomRunsYou_MaxLogLines", ui->spinBox_MaxLogLines->value());

    delete ui;
}

int DoomRunsYouForm::getRelativeUptime_ms(void)
{
    QElapsedTimer uptimeTimer;
    uptimeTimer.start();
    return uptimeTimer.msecsSinceReference() - uptimeBaseline;
}


void DoomRunsYouForm::addLogLine(const QString& line, bool important)
{
    QElapsedTimer uptimeTimer;
    uptimeTimer.start();
    qint64 absUptime = uptimeTimer.msecsSinceReference();
    int relUptime = uptimeTimer.msecsSinceReference() - uptimeBaseline;

    QTime currentTime = QTime::currentTime();

    QString timeString = currentTime.toString("hh:mm:ss:zzz") + ", uptime abs: " + QString::number(absUptime) + ", rel: " + QString::number(relUptime);

    ui->plainTextEdit_Log->setMaximumBlockCount(ui->spinBox_MaxLogLines->value());
    ui->plainTextEdit_Log_Severe->setMaximumBlockCount(ui->spinBox_MaxLogLines->value());
    ui->plainTextEdit_Log->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Log_Severe->setWordWrapMode(QTextOption::NoWrap);
    ui->plainTextEdit_Log->appendPlainText(timeString + ": " + line);

    if (important)
    {
        ui->plainTextEdit_Log_Severe->appendPlainText(timeString + ": " + line);
    }

    QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
}

void DoomRunsYouForm::on_pushButton_SendManualCommand_clicked()
{
    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        int sendData[6];

        sendData[0] = CT_LOCATION_ORIENTATION_COMMAND;
        sendData[1] = 0;    // Counter always 0 in manual commands
        sendData[2] = ui->spinBox_Forward->value();
        sendData[3] = ui->spinBox_Side->value();
        sendData[4] = ui->spinBox_Pitch->value();
        sendData[5] = ui->spinBox_Yaw->value();

        DWORD bytesWritten;

        BOOL success = WriteFile(
                    pipeHandle,            // pipe handle
                    &sendData,             // message
                    sizeof(sendData),      // message length
                    &bytesWritten,         // bytes written
                    NULL);                 // not overlapped

        if (!success)
        {
            addLogLine("Error: Sending manual command failed (WriteFile-function ot called successfully).", true);
        }
    }
    else
    {
        addLogLine("Error: Pipe not open, not sending manual command.", true);
    }
}

void DoomRunsYouForm::newPositionData(const UBXMessage_RELPOSNED& relposned_RoverA, const UBXMessage_RELPOSNED& relposned_RoverB)
{
    int dataReceivedUptime = getRelativeUptime_ms();

    Eigen::Vector3d roverAPosNED(
            relposned_RoverA.relPosN,
            relposned_RoverA.relPosE,
            relposned_RoverA.relPosD);

    Eigen::Vector3d roverBPosNED(
            relposned_RoverB.relPosN,
            relposned_RoverB.relPosE,
            relposned_RoverB.relPosD);

    Eigen::Vector3d roverBToAVecNormalizedNED = (roverAPosNED - roverBPosNED).normalized();

    Eigen::Vector3d downVecNED(0,0,1);
    Eigen::Vector3d bfggForwardAxis = roverBToAVecNormalizedNED;
    Eigen::Vector3d bfggRightAxis = -(roverBToAVecNormalizedNED.cross(downVecNED).normalized());
    Eigen::Vector3d bfggDownAxis = roverBToAVecNormalizedNED.cross(bfggRightAxis).normalized();

    // Some variables for camera:
    double cameraNShift = ui->doubleSpinBox_BFGG_Camera_N->value();
    double cameraEShift = ui->doubleSpinBox_BFGG_Camera_E->value();
    double cameraDShift = ui->doubleSpinBox_BFGG_Camera_D->value();

    double lookAtNShift = ui->doubleSpinBox_BFGG_LookAt_N->value();
    double lookAtEShift = ui->doubleSpinBox_BFGG_LookAt_E->value();
    double lookAtDShift = ui->doubleSpinBox_BFGG_LookAt_D->value();

    Eigen::Vector3d cameraPosNED = roverAPosNED +
            bfggForwardAxis * cameraNShift +
            bfggRightAxis * cameraEShift +
            bfggDownAxis * cameraDShift;

    Eigen::Vector3d lookAtPosNED = roverAPosNED +
            bfggForwardAxis * lookAtNShift +
            bfggRightAxis * lookAtEShift +
            bfggDownAxis * lookAtDShift;

    Eigen::Vector3d cameraToLookAtVecNED = lookAtPosNED - cameraPosNED;

    double yaw = atan2(cameraToLookAtVecNED(1), cameraToLookAtVecNED(0));
    double pitch = atan2(-cameraToLookAtVecNED(2), sqrt(cameraToLookAtVecNED(0) * cameraToLookAtVecNED(0) + cameraToLookAtVecNED(1) * cameraToLookAtVecNED(1)));

    trimChart();

    LocationOrientation newLO;

    if ((yaw - lastYawCalculatedFromReceivedData) > M_PI)
    {
        contYawRounds--;
    }

    if ((yaw - lastYawCalculatedFromReceivedData) < -M_PI)
    {
        contYawRounds++;
    }

    lastYawCalculatedFromReceivedData = yaw;

    newLO.uptime = dataReceivedUptime;
    newLO.x = cameraPosNED(1);
    newLO.y = cameraPosNED(0);
    newLO.contYaw = yaw + contYawRounds * 2 * M_PI;
    newLO.pitch = pitch;

    lineSeries_Yaw_Unfiltered->append(dataReceivedUptime, newLO.contYaw * 360 / (2 * M_PI));
    lineSeries_Pitch_Unfiltered->append(dataReceivedUptime, newLO.pitch * 360 / (2 * M_PI));

    locationOrientationHistory.push_back(newLO);

    while (locationOrientationHistory.size() > 100)
    {
        locationOrientationHistory.remove(0);
    }
}

void DoomRunsYouForm::on_pushButton_ReOpenPipe_clicked()
{
    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(pipeHandle);
        addLogLine("Existing pipe handle closed.", true);
    }

    addLogLine("Trying to open new pipe...", true);

    pipeHandle = CreateFile(
                pipeName,       // pipe name
                GENERIC_READ |  // read and write access
                GENERIC_WRITE,
                0,              // no sharing
                NULL,           // default security attributes
                OPEN_EXISTING,  // opens existing pipe
                0,              // default attributes
                NULL);          // no template file

    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        // Using nonblocking mode here, because it seems to work
        // (and I'm too lazy to implement this "right" for this).

        DWORD mode = PIPE_READMODE_MESSAGE | PIPE_NOWAIT; // "Note that nonblocking mode is supported for compatibility with Microsoft LAN Manager version 2.0 and should not be used to achieve asynchronous input and output (I/O) with named pipes."
        BOOL success = SetNamedPipeHandleState(
           pipeHandle,    // pipe handle
           &mode,  // new pipe mode
           NULL,     // don't set maximum bytes
           NULL);    // don't set maximum time

        if (!success)
        {
            addLogLine("Error: SetNamedPipeHandleState failed. Pipe not opened.", true);
            CloseHandle(pipeHandle);
            pipeHandle = INVALID_HANDLE_VALUE;
        }
        else
        {
            addLogLine("Pipe opened.", true);
        }
    }
    else
    {
        addLogLine("Error: Can not open pipe (CreateFile failed).", true);
    }
}

void DoomRunsYouForm::on_pushButton_ClosePipe_clicked()
{
    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        CloseHandle(pipeHandle);
        pipeHandle = INVALID_HANDLE_VALUE;
        addLogLine("Pipe closed.", true);
    }
    else
    {
        addLogLine("Pipe not open, nothing to do.", true);
    }
}

void DoomRunsYouForm::fastTickTimerTimeout()
{
    // This (polling at 1 ms interval) is a horrible way to handle this.
    // Should use threads / overlapped IO / signals / slots instead.
    // I'm, however too lazy to implement "the correct" handling here.
    // This works good enough for this joke(ish) thingy...

    //    addLogLine("Timer!!!");

    int currentUptime = getRelativeUptime_ms();

    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        int recData[6];
        DWORD bytesRead;
        if (ReadFile(pipeHandle, &recData, sizeof(recData), &bytesRead, NULL))
        {
            if (bytesRead == sizeof(recData))
            {
                CommandType commandType = CommandType(recData[0]);
                int readBackCounter = recData[1];
                DWORD bytesWritten;
                BOOL success;

                switch (commandType)
                {
                case CT_LOCATION_ORIENTATION_COMMAND:

                    if (readBackCounter == 0)
                    {
                        addLogLine("Response to manual command received.", true);
                    }
                    else if (readBackCounter == doomRunsYouCommandCounter)
                    {
                        addLogLine("Response to command received.");

                        if (locationOrientationHistory.size() >= 3)
                        {
                            // Require at least 3 measurements. Most of the "modes" don't actually need that many,
                            // but the "constant acceleration"- mode does and additional startup delay is negligible anyway.

                            LocationOrientation currentLO = locationOrientationHistory[locationOrientationHistory.size() - 1];

                            if (currentUptime - locationOrientationHistory[locationOrientationHistory.size() - 3].uptime < 1000)
                            {
                                double posX = lastPosX;
                                double posY = lastPosY;
                                double contYaw = 0;
                                double pitch = 0;
                                double predictUptime = (currentUptime + ui->spinBox_MotionPredictTime->value());
                                int i;

                                switch(ui->comboBox_MotionPredictorType->currentIndex())
                                {
                                case 0: // None - use most recent value
                                    posX = currentLO.x;
                                    posY = currentLO.y;
                                    contYaw = currentLO.contYaw;
                                    pitch = currentLO.pitch;
                                    break;

                                case 1: // None - use most recent or linearly interpolated value (when using negative time values)
                                    if (locationOrientationHistory[0].uptime >= predictUptime)
                                    {
                                        posX = locationOrientationHistory[0].x;
                                        posY = locationOrientationHistory[0].y;
                                        contYaw = locationOrientationHistory[0].contYaw;
                                        pitch = locationOrientationHistory[0].pitch;
                                    }
                                    else
                                    {
                                        for (i = locationOrientationHistory.size() - 1; i >= 0; i--)
                                        {
                                            if (locationOrientationHistory[i].uptime <= predictUptime)
                                            {
                                                if (i == locationOrientationHistory.size() - 1)
                                                {
                                                    // Time is more recent than the last value -> use non-interpolated most recent value
                                                    posX = currentLO.x;
                                                    posY = currentLO.y;
                                                    contYaw = currentLO.contYaw;
                                                    pitch = currentLO.pitch;
                                                }
                                                else
                                                {
                                                    // Interpolate
                                                    double fraction = (predictUptime - locationOrientationHistory[i].uptime) /
                                                            (locationOrientationHistory[i + 1].uptime - locationOrientationHistory[i].uptime);

                                                    posX = locationOrientationHistory[i].x + fraction * (locationOrientationHistory[i + 1].x - locationOrientationHistory[i].x);
                                                    posY = locationOrientationHistory[i].y + fraction * (locationOrientationHistory[i + 1].y - locationOrientationHistory[i].y);
                                                    contYaw = locationOrientationHistory[i].contYaw + fraction * (locationOrientationHistory[i + 1].contYaw - locationOrientationHistory[i].contYaw);
                                                    pitch = locationOrientationHistory[i].pitch + fraction * (locationOrientationHistory[i + 1].pitch - locationOrientationHistory[i].pitch);
                                                }
                                                break;
                                            }
                                        }
                                    }
                                    break;
                                case 2: // Linear extrapolation (constant changing speed estimated using the last two measurements)
                                    {
                                    LocationOrientation prevLO = locationOrientationHistory[locationOrientationHistory.size() - 2];

                                    double timeDiff = currentLO.uptime - prevLO.uptime;
                                    double predictTimeDiff = (currentUptime + ui->spinBox_MotionPredictTime->value()) - currentLO.uptime;

                                    double xDiff = currentLO.x - prevLO.x;
                                    double yDiff = currentLO.y - prevLO.y;

                                    double yawDiff = currentLO.contYaw - prevLO.contYaw;
                                    double pitchDiff = currentLO.pitch - prevLO.pitch;

                                    posX = currentLO.x + xDiff * (predictTimeDiff / timeDiff);
                                    posY = currentLO.y + yDiff * (predictTimeDiff / timeDiff);

                                    contYaw = currentLO.contYaw + yawDiff * (predictTimeDiff / timeDiff);
                                    pitch = currentLO.pitch + pitchDiff * (predictTimeDiff / timeDiff);
                                    }
                                    break;

                                case 3: // Constant acceleration estimated using the last three measurements
                                    // This is somewhat simplified so that the acceleration is not handled as
                                    // continuous but instead the changing speed of the signals is calculated
                                    // using the last three measurements and this "slope" is then used in extrapolation.
                                    // This is only correct for the next "extrapolation piece"
                                    // (basically valid only when motion predict time is 0).

                                    {
                                    LocationOrientation prevLO = locationOrientationHistory[locationOrientationHistory.size() - 2];
                                    LocationOrientation prevPrevLO = locationOrientationHistory[locationOrientationHistory.size() - 3];

                                    double predictTimeDiff = (currentUptime + ui->spinBox_MotionPredictTime->value()) - currentLO.uptime;

                                    posX = currentLO.x +
                                        (2 * ((currentLO.x - prevLO.x) / (currentLO.uptime - prevLO.uptime)) -
                                            ((prevLO.x - prevPrevLO.x) / (prevLO.uptime - prevPrevLO.uptime))) *
                                            predictTimeDiff;

                                    posY = currentLO.y +
                                        (2 * ((currentLO.y - prevLO.y) / (currentLO.uptime - prevLO.uptime)) -
                                            ((prevLO.y - prevPrevLO.y) / (prevLO.uptime - prevPrevLO.uptime))) *
                                            predictTimeDiff;

                                    contYaw = currentLO.contYaw +
                                        (2 * ((currentLO.contYaw - prevLO.contYaw) / (currentLO.uptime - prevLO.uptime)) -
                                            ((prevLO.contYaw - prevPrevLO.contYaw) / (prevLO.uptime - prevPrevLO.uptime))) *
                                            predictTimeDiff;

                                    pitch = currentLO.pitch +
                                        (2 * ((currentLO.pitch - prevLO.pitch) / (currentLO.uptime - prevLO.uptime)) -
                                            ((prevLO.pitch - prevPrevLO.pitch) / (prevLO.uptime - prevPrevLO.uptime))) *
                                            predictTimeDiff;

                                }
                                    break;
                                }

                                lastPosX = posX;
                                lastPosY = posY;

                                // "Post"-filtering here is just a simple 1st-order low-pass filter.

                                double LPFilteringCoeff = ui->doubleSpinBox_PostLPFilteringCoefficient->value();

                                posX = posX * LPFilteringCoeff + posXFilteringStorage * (1 - LPFilteringCoeff);
                                posXFilteringStorage = posX;

                                posY = posY * LPFilteringCoeff + posYFilteringStorage * (1 - LPFilteringCoeff);
                                posYFilteringStorage = posY;

                                contYaw = contYaw * LPFilteringCoeff + yawFilteringStorage * (1 - LPFilteringCoeff);
                                yawFilteringStorage = contYaw;

                                pitch = pitch * LPFilteringCoeff + pitchFilteringStorage * (1 - LPFilteringCoeff);
                                pitchFilteringStorage = pitch;

                                double movement = sqrt(pow(posX - lastSentPosX, 2) + pow(posY - lastSentPosY, 2));

                                int intYaw = -contYaw * 65536 / (2 * M_PI);
                                int intPitch = pitch * 65536 / (2 * M_PI);

                                intYaw &= 0xFFFF;
                                intPitch &= 0xFFFF;

                                int intMovementX = 0;
                                int intMovementY = 0;

                                if (movement >= 1)
                                {
                                    addLogLine("Instantaneous movement is insanely big, skipping.");
                                    lastSentPosX = posX;
                                    lastSentPosY = posY;
                                }
                                else if (movement >= ui->doubleSpinBox_LocationUpdateThreshold->value())
                                {
                                    // Threshold is here to prevent error accumulating into location
                                    // while standing still.

                                    addLogLine("Movement exceeding threshold, sending new location.");

                                    double movementX = posX - lastSentPosX - movementRoundingErrorX;
                                    double movementY = posY - lastSentPosY - movementRoundingErrorY;

                                    Eigen::Vector2d movementXY(movementX, movementY);
                                    Eigen::Vector2d firstPersonBasisForward(sin(contYaw), cos(contYaw));
                                    Eigen::Vector2d firstPersonBasisRight(sin(contYaw + M_PI/2), cos(contYaw + M_PI/2));

                                    Eigen::Vector2d firstPersonMovement(firstPersonBasisForward.dot(movementXY), firstPersonBasisRight.dot(movementXY));

                                    lastSentPosX = posX;
                                    lastSentPosY = posY;

                                    intMovementX = (firstPersonMovement(0) * ui->doubleSpinBox_CoordMultiplier->value()) + 0.5;
                                    intMovementY = (firstPersonMovement(1) * ui->doubleSpinBox_CoordMultiplier->value()) + 0.5;
#if 0
                                    // Try to minimize effect of the rounding error accumulating into the location
                                    // There's some brainfart and movement goes all over the place.
                                    // (this was the case even before changing the yaw value of 0 to point to north)
                                    // Works good enough without, so not using more time for this, at least for now
                                    double realMovementX = sin(intYaw * 2 * M_PI / 65536) * intMovementX / ui->doubleSpinBox_CoordMultiplier->value() +
                                            sin(((intYaw + 65536 / 4) & 0xFFFF) * 2 * M_PI / 65536) * intMovementY / ui->doubleSpinBox_CoordMultiplier->value();

                                    double realMovementY = cos(intYaw * 2 * M_PI / 65536) * intMovementX / ui->doubleSpinBox_CoordMultiplier->value() +
                                            cos(((intYaw + 65536 / 4) & 0xFFFF) * 2 * M_PI / 65536) * intMovementY / ui->doubleSpinBox_CoordMultiplier->value();

                                    movementRoundingErrorX = realMovementX - movementX;
                                    movementRoundingErrorY = realMovementY - movementY;
#endif
                                }

                                double clampedYaw = std::max(contYaw - M_PI * 2 * floor(contYaw / (2 * M_PI)), double(0));   // 0 to 2 * M_PI, max to prevent negative values due to rounding errors

                                addLogLine("Dbg:\tYaw, cont:" + QString::number(contYaw, 'f', 1) +
                                           "\t(" + QString::number(clampedYaw * 360 / (2*M_PI), 'f', 2) + " deg)" +
                                           "\t(as int: " + QString::number(intYaw) +
                                           "),\tpitch: " + QString::number(pitch, 'f', 2) + "\t(as int " + QString::number(intPitch) +
                                           "),\tcontYawRounds:" + QString::number(contYawRounds)
                                           );

                                int sendData[6];

                                if (ui->checkBox_Active->checkState() == Qt::Checked)
                                {
                                    sendData[0] = CT_LOCATION_ORIENTATION_COMMAND;
                                    sendData[1] = ++doomRunsYouCommandCounter;
                                    sendData[2] = intMovementX;
                                    sendData[3] = intMovementY;
                                    sendData[4] = intPitch - lastIntPitch;
                                    sendData[5] = intYaw - lastIntYaw;
                                }
                                else
                                {
                                    // When not active, just send zero data.
                                    // This can be used for example when starting the game to sync location/yaw/pitch-values
                                    // You can start the game when inactive here and switch to active -> Yaw will match in the
                                    // game and here and location will not be changed (much). This way you can place some items into the
                                    // real world, whose locations will (approximately) match the locations in the game.

                                    addLogLine("Not active, sending dummy command.");

                                    sendData[0] = CT_LOCATION_ORIENTATION_COMMAND;
                                    sendData[1] = ++doomRunsYouCommandCounter;
                                    sendData[2] = 0;
                                    sendData[3] = 0;
                                    sendData[4] = 0;
                                    sendData[5] = 0;

                                }

                                DWORD bytesWritten;

                                BOOL success = WriteFile(
                                            pipeHandle,            // pipe handle
                                            &sendData,             // message
                                            sizeof(sendData),      // message length
                                            &bytesWritten,         // bytes written
                                            NULL);                 // not overlapped

                                if (!success)
                                {
                                    addLogLine("Error: Sending command failed (WriteFile-function not called successfully).");
                                }
                                else
                                {
                                    addLogLine("New command sent successfully.");
                                    lastSentCommandUptime = currentUptime;
                                }

                                lastIntPitch = intPitch;
                                lastIntYaw = intYaw;

                                lineSeries_Yaw_Filtered->append(predictUptime, contYaw * 360 / (2*M_PI));
                                lineSeries_Pitch_Filtered->append(predictUptime, pitch * 360 / (2*M_PI));

                                trimChart();
                            }
                            else
                            {
                                addLogLine("GNSS data too old, sending dummy command.");

                                int sendData[6];

                                sendData[0] = CT_LOCATION_ORIENTATION_COMMAND;
                                sendData[1] = ++doomRunsYouCommandCounter;
                                sendData[2] = 0;
                                sendData[3] = 0;
                                sendData[4] = 0;
                                sendData[5] = 0;

                                DWORD bytesWritten;

                                BOOL success = WriteFile(
                                            pipeHandle,            // pipe handle
                                            &sendData,             // message
                                            sizeof(sendData),      // message length
                                            &bytesWritten,         // bytes written
                                            NULL);                 // not overlapped

                                if (!success)
                                {
                                    addLogLine("Error: Sending command failed (WriteFile-function not called successfully).");
                                }

                                lastSentCommandUptime = currentUptime;
                            }
                        }
                        else if (locationOrientationHistory.size() == 1)
                        {
                            // Initialize filters to prevent jumps at the start

                            posXFilteringStorage = locationOrientationHistory[0].x;
                            posYFilteringStorage = locationOrientationHistory[0].y;
                            yawFilteringStorage = locationOrientationHistory[0].contYaw;
                            pitchFilteringStorage = locationOrientationHistory[0].pitch;
                        }
                    }
                    else
                    {
                        addLogLine("Counter mismatch. Waiting for correct counter value.");
                    }
                    break;
                case CT_PING_FROM_SERVER:

                    addLogLine("Ping request from server received, responding...", true);

                    success = WriteFile(
                                pipeHandle,            // pipe handle
                                &recData,             // message
                                sizeof(recData),      // message length
                                &bytesWritten,         // bytes written
                                NULL);                 // not overlapped

                    if (!success)
                    {
                        addLogLine("Error: Sending response to server's ping failed (WriteFile-function ot called successfully).", true);
                    }
                    break;

                case CT_PING_FROM_CLIENT:
                    addLogLine("Response to ping received.", true);
                    break;

                default:
                    addLogLine("Undefined command id (" + QString::number(recData[0]) + ") received.", true);
                    break;

                }
            }
        }

        if (currentUptime - lastSentCommandUptime  > 500)
        {
            addLogLine("No commands in a while, sending dummy command to wake up sending loop...");

            int sendData[6];

            sendData[0] = CT_LOCATION_ORIENTATION_COMMAND;
            sendData[1] = ++doomRunsYouCommandCounter;
            sendData[2] = 0;
            sendData[3] = 0;
            sendData[4] = 0;
            sendData[5] = 0;

            DWORD bytesWritten;

            BOOL success = WriteFile(
                        pipeHandle,            // pipe handle
                        &sendData,             // message
                        sizeof(sendData),      // message length
                        &bytesWritten,         // bytes written
                        NULL);                 // not overlapped

            if (!success)
            {
                addLogLine("Error: Sending command failed (WriteFile-function not called successfully).");
            }

            lastSentCommandUptime = currentUptime;
        }
    }
}


void DoomRunsYouForm::on_pushButton_Ping_clicked()
{
    if (pipeHandle != INVALID_HANDLE_VALUE)
    {
        int sendData[6];

        sendData[0] = CT_PING_FROM_CLIENT;
        for (unsigned int i = 1; i < sizeof(sendData) / sizeof(sendData[0]); i++)
        {
            sendData[i] = 0;
        }

        DWORD bytesWritten;

        BOOL success = WriteFile(
                    pipeHandle,            // pipe handle
                    &sendData,             // message
                    sizeof(sendData),      // message length
                    &bytesWritten,         // bytes written
                    NULL);                 // not overlapped

        if (!success)
        {
            addLogLine("Error: Sending ping failed (WriteFile-function ot called successfully).", true);
        }
    }
    else
    {
        addLogLine("Error: Pipe not open, not sending ping.", true);
    }
}

void DoomRunsYouForm::trimChart(void)
{
    int uptime = getRelativeUptime_ms();
    int trimUptime = uptime - 5000;

    removeOldSeriesData(lineSeries_Yaw_Filtered, trimUptime);
    removeOldSeriesData(lineSeries_Yaw_Unfiltered, trimUptime);

    removeOldSeriesData(lineSeries_Pitch_Filtered, trimUptime);
    removeOldSeriesData(lineSeries_Pitch_Unfiltered, trimUptime);

    double minY = 1e12;
    double maxY = -1e12;

    // TODO: Remove these repeating code blocks.
    // There might also be nicer way to do this automatic scaling.

    for (int i = 0; i < lineSeries_Yaw_Unfiltered->count(); i++)
    {
        if (lineSeries_Yaw_Unfiltered->at(i).y() < minY)
        {
            minY = lineSeries_Yaw_Unfiltered->at(i).y();
        }
        if (lineSeries_Yaw_Unfiltered->at(i).y() > maxY)
        {
            maxY = lineSeries_Yaw_Unfiltered->at(i).y();
        }
    }

    for (int i = 0; i < lineSeries_Yaw_Filtered->count(); i++)
    {
        if (lineSeries_Yaw_Filtered->at(i).y() < minY)
        {
            minY = lineSeries_Yaw_Filtered->at(i).y();
        }
        if (lineSeries_Yaw_Filtered->at(i).y() > maxY)
        {
            maxY = lineSeries_Yaw_Filtered->at(i).y();
        }
    }

    yAxis_Angles_Yaw->setRange(minY - ((maxY-minY) * 0.1), maxY + ((maxY-minY) * 0.1));

    minY = 1e12;
    maxY = -1e12;

    for (int i = 0; i < lineSeries_Pitch_Unfiltered->count(); i++)
    {
        if (lineSeries_Pitch_Unfiltered->at(i).y() < minY)
        {
            minY = lineSeries_Pitch_Unfiltered->at(i).y();
        }
        if (lineSeries_Pitch_Unfiltered->at(i).y() > maxY)
        {
            maxY = lineSeries_Pitch_Unfiltered->at(i).y();
        }
    }

    for (int i = 0; i < lineSeries_Pitch_Filtered->count(); i++)
    {
        if (lineSeries_Pitch_Filtered->at(i).y() < minY)
        {
            minY = lineSeries_Pitch_Filtered->at(i).y();
        }
        if (lineSeries_Pitch_Filtered->at(i).y() > maxY)
        {
            maxY = lineSeries_Pitch_Filtered->at(i).y();
        }
    }

    yAxis_Angles_Pitch->setRange(minY - ((maxY-minY) * 0.1), maxY + ((maxY-minY) * 0.1));

    xAxis_Angles->setRange(trimUptime, std::max(uptime, uptime+ui->spinBox_MotionPredictTime->value()));
}

void DoomRunsYouForm::removeOldSeriesData(QtCharts::QLineSeries* series, int xAxisThreshold)
{
    while ((series->count() > 0) &&
           (series->at(0).x() < xAxisThreshold))
    {
        series->remove(0);
    }
}

void DoomRunsYouForm::on_checkBox_Active_stateChanged(int arg1)
{
    Qt::CheckState state = Qt::CheckState(arg1);

    if (state == Qt::Checked)
    {
        addLogLine("Activity state changed: Restarting everything.", true);

        contYawRounds = 0;
        lastYawCalculatedFromReceivedData = 0;
        lastIntYaw = 0;
        lastIntPitch = 0;
        lastPosX = 0;
        lastPosY = 0;

        locationOrientationHistory.clear();

        lastSentCommandUptime = 0;

        posXFilteringStorage = 0;
        posYFilteringStorage = 0;
        yawFilteringStorage = 0;
        pitchFilteringStorage = 0;
        lastSentPosX = 0;
        lastSentPosY = 0;

        movementRoundingErrorX = 0;
        movementRoundingErrorY = 0;
    }
}

void DoomRunsYouForm::on_pushButton_ClearAll_clicked()
{
    ui->plainTextEdit_Log->clear();
    ui->plainTextEdit_Log_Severe->clear();
}
