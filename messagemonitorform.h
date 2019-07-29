/*
    messagemonitorform.h (part of GNSS-Stylus)
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
 * @file messagemonitorform.h
 * @brief Declaration for a form that shows some data about messages sent by u-blox-devices.
 */

#ifndef MESSAGEMONITORFORM_H
#define MESSAGEMONITORFORM_H

#include <QWidget>
#include "serialthread.h"
#include "ntripthread.h"
#include "ubloxdatastreamprocessor.h"

namespace Ui {
class MessageMonitorForm;
}

/**
 * @brief Form used to show some data about messages sent by u-blox-devices.
 */
class MessageMonitorForm : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent widget
     * @param title Form title
     */
    explicit MessageMonitorForm(QWidget *parent = nullptr, const QString& title = "Message monitor");
    ~MessageMonitorForm();

    /**
     * @brief Connects slots from SerialThread
     * @param serThread SerialThread to connect signals from
     */
    void connectSerialThreadSlots(SerialThread* serThread);

    /**
     * @brief Disconnects slots from SerialThread
     * @param serThread SerialThread to disconnect signals from
     */
    void disconnectSerialThreadSlots(SerialThread* serThread);

    /**
     * @brief Connects slots from UBloxDataStreamProcessor
     * @param ubloxDataStreamProcessor UBloxDataStreamProcessor to connect signals from
     */
    void connectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor);

    /**
     * @brief Disconnects slots from UBloxDataStreamProcessor
     * @param ubloxDataStreamProcessor UBloxDataStreamProcessor to disconnect signals from
     */
    void disconnectUBloxDataStreamProcessorSlots(UBloxDataStreamProcessor* ubloxDataStreamProcessor);

    /**
     * @brief Connects slots from NTRIPThread
     * @param ntripThread NTRIPThread to connect signals from
     */
    void connectNTRIPThreadSlots(NTRIPThread* ntripThread);

    /**
     * @brief Disconnects slots from NTRIPThread
     * @param serThread NTRIPThread to disconnect signals from
     */
    void disconnectNTRIPThreadSlots(NTRIPThread* ntripThread);


private:
    Ui::MessageMonitorForm *ui;

    void addLogLine(const QString& line);

private slots:
    void ubloxProcessor_nmeaSentenceReceived(const QByteArray&);
    void ubloxProcessor_ubxMessageReceived(const UBXMessage&);
    void ubloxProcessor_rtcmMessageReceived(const RTCMMessage&);

    void ubloxProcessor_ubxParseError(const QString&);
    void ubloxProcessor_nmeaParseError(const QString&);
    void ubloxProcessor_unidentifiedDataReceived(const QByteArray& data);

    void ErrorMessage(const QString& errorMessage);
    void WarningMessage(const QString& warningMessage);
    void InfoMessage(const QString& infoMessage);
    void DataReceived(const QByteArray& bytes);
    void serialTimeout(void);

    void on_pushButton_ClearAll_clicked();
};

#endif // MESSAGEMONITORFORM_H
