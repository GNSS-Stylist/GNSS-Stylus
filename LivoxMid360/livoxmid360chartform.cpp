#include "livoxmid360chartform.h"
#include "ui_livoxmid360chartform.h"
#include "Util/textblockparser.h"

LivoxMid360ChartForm::LivoxMid360ChartForm(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::LivoxMid360ChartForm)
{
    ui->setupUi(this);

    chart = new QtCharts::QChart();
    ui->chartView->setChart(chart);


/*    lineSeries_Angle = new QtCharts::QLineSeries();
    chart->addSeries(lineSeries_Angle);
    lineSeries_Angle->setName("First");
    lineSeries_Angle->setUseOpenGL(true);

    scatterSeries_Angle = new QtCharts::QScatterSeries();
    chart->addSeries(scatterSeries_Angle);
    scatterSeries_Angle->setMarkerSize(5);
    scatterSeries_Angle->setPen(QPen(Qt::PenStyle::NoPen));
    scatterSeries_Angle->setName("First");
    scatterSeries_Angle->setUseOpenGL(true);
*/
    timeAxis = new QtCharts::QValueAxis();
//    timeAxis->setTitleText("Time");
    timeAxis->setTickCount(10);
    timeAxis->setLabelFormat("%.2d");
    timeAxis->setRange(0, 0.1);
    chart->addAxis(timeAxis, Qt::AlignBottom);

    angleAxis = new QtCharts::QValueAxis();
//    angleAxis->setTitleText("Angle");
//    angleAxis->setTickCount(10);
    angleAxis->setLabelFormat("%d");
    angleAxis->setRange(-7, 52);
    chart->addAxis(angleAxis, Qt::AlignLeft);
/*
    lineSeries_Angle->attachAxis(timeAxis);
    lineSeries_Angle->attachAxis(angleAxis);

    scatterSeries_Angle->attachAxis(timeAxis);
    scatterSeries_Angle->attachAxis(angleAxis);
*/
/*
    lineSeries_Angle->append(-0.1, 52);
    lineSeries_Angle->append(-0.09, 45);
    lineSeries_Angle->append(-0.08, 20);
    lineSeries_Angle->append(-0.04, 0);
    lineSeries_Angle->append(0, -7);

    scatterSeries_Angle->append(-0.1, 52);
*/
/*    scatterSeries_Angle->append(-0.09, 45);
    scatterSeries_Angle->append(-0.08, 20);
    scatterSeries_Angle->append(-0.04, 0);
    scatterSeries_Angle->append(0, -7);
*/

/*    QVector<QPointF> newVals;

    newVals.push_back(QPointF(-0.1, -7));
    newVals.push_back(QPointF(-0.075, 0));
    newVals.push_back(QPointF(-0.05, 45));
    newVals.push_back(QPointF(0, 24));
*/
//    lineSeries_Angle->replace(newVals);
//    scatterSeries_Angle->replace(newVals);

    // test data


}

LivoxMid360ChartForm::~LivoxMid360ChartForm()
{
    delete ui;
}

void LivoxMid360ChartForm::connectLivoxMid360Thread(LivoxMid360Thread* livoxMid360Thread)
{
    this->mid360Thread = livoxMid360Thread;

    if (active)
    {
        connectLivoxMid360ThreadSlots();
    }
}

void LivoxMid360ChartForm::disconnectLivoxMid360Thread(LivoxMid360Thread* livoxMid360Thread)
{
    disconnectLivoxMid360ThreadSlots();

    this->mid360Thread = nullptr;
}

bool LivoxMid360ChartForm::connectLivoxMid360ThreadSlots(void)
{
    if (mid360Thread)
    {
        QObject::connect(mid360Thread, &LivoxMid360Thread::pushLidarInformationReceived,
                         this, &LivoxMid360ChartForm::pushLidarInformationReceived);

        QObject::connect(mid360Thread, &LivoxMid360Thread::rawDatagramReceived,
                this, &LivoxMid360ChartForm::on_LivoxMid360RawDatagramReceived);

        return true;
    }

    return false;
}

void LivoxMid360ChartForm::disconnectLivoxMid360ThreadSlots(void)
{
    if (mid360Thread)
    {
        QObject::disconnect(mid360Thread, &LivoxMid360Thread::pushLidarInformationReceived,
                            this, &LivoxMid360ChartForm::pushLidarInformationReceived);

        QObject::disconnect(mid360Thread, &LivoxMid360Thread::rawDatagramReceived,
                   this, &LivoxMid360ChartForm::on_LivoxMid360RawDatagramReceived);
    }
}

volatile int dbgTrap = 0;

void LivoxMid360ChartForm::pushLidarInformationReceived(quint32 ipAddress, const LivoxMid360::PushLidarInformation& info, qint64 upTime)
{
    dbgTrap = 100;
}

void LivoxMid360ChartForm::on_LivoxMid360RawDatagramReceived(const QNetworkDatagram& datagram, qint64 timeStamp)
{
    bool isIPV4Address;
    quint32 senderAddress = datagram.senderAddress().toIPv4Address(&isIPV4Address);
    if (!isIPV4Address)
    {
        // Just discard the datagram if sender address is not IPV4 (should not happen, though)
        return;
    }

    if ((samplingState == STATE_WAITING_FOR_TRIGGER) && (senderAddress != settings.synchronizationSource))
    {
        // Only interested on the triggering device in this state so save some time by exiting early (here)
        return;
    }

    // Check that datagram is coming from a device that is in the list of allowed devices (or sync device)
    if (!deviceDataMap.contains(senderAddress) && (senderAddress != settings.synchronizationSource))
    {
        // Device not found in the allowed list. Just discard data.
        return;
    }

    // Check that the datagram can be interpreted as a valid "control command" before handling it.
    LivoxMid360::PointCloudAndIMUDataHeader header(datagram);

    if (header.status != LivoxMid360::PointCloudAndIMUDataHeader::STATUS_VALID)
    {
        return;
    }

    if ((header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_32BIT) &&
        (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_CARTESIAN_16BIT) &&
        (header.data_type != LivoxMid360::PointCloudAndIMUDataHeader::DATA_TYPE_POINTS_SPHERICAL))
    {
        return;
    }

    LivoxMid360::PointCloudData pcData(header, datagram);

    if ((pcData.status != LivoxMid360::PointCloudAndIMUDataHeader::MessageDataStatus::STATUS_VALID) ||
        (pcData.time_type != LivoxMid360::PointCloudAndIMUDataHeader::TimeSyncType::TIME_SYNC_GPS))
    {
        return;
    }

    quint64 pointStartTime_ns = pcData.timestamp;

    if ((samplingState == STATE_COLLECTING_SAMPLES) && (pointStartTime_ns < trigTime_ns))
    {
        if (senderAddress == settings.synchronizationSource)
        {
            // This is for replay purposes. When replay loops or is restarted earlier
            // than it was stopped, it's better to just start waiting for trigger again
            samplingState = STATE_WAITING_FOR_TRIGGER;
            return;
        }

        // Sometimes packets arrive at a "wrong order" so discard datagrams having too early timestamp.
        // (not really, because units operate on their own timebases, also UDP does it's things).
        // Without this, non-triggering devices have a horizontal line on the chart now and then.
        return;
    }

    double distSquaredLimit = settings.minimumDistance * settings.minimumDistance;

    double angleSum = 0;
    int anglePoints = 0;

    for (auto& point:pcData.points)
    {
        double distSquared = point.x * point.x + point.y * point.y + point.z * point.z;

        if (distSquared >= distSquaredLimit)
        {
            angleSum += asin(point.z / sqrt(distSquared));
            anglePoints++;
        }
    }

    if (anglePoints == 0)
    {
        // No valid points available in this datagram -> skip
        return;
    }

    double avgAngle = qRadiansToDegrees(angleSum / anglePoints);

    if (samplingState == STATE_WAITING_FOR_TRIGGER)
    {
        bool trig = false;
        if (settings.triggerInverted)
        {
            if ((avgAngle <= settings.triggerLevel) && (prevTriggerDeviceAngle > settings.triggerLevel))
            {
                trig = true;
            }
        }
        else
        {
            if ((avgAngle >= settings.triggerLevel) && (prevTriggerDeviceAngle < settings.triggerLevel))
            {
                trig = true;
            }
        }

        if (trig)
        {
            trigTime_ns = pointStartTime_ns;    // Start time should be accurate enough for this use
            samplingState = STATE_COLLECTING_SAMPLES;
        }
    }

    if (senderAddress == settings.synchronizationSource)
    {
        prevTriggerDeviceAngle = avgAngle;
    }

    if ((samplingState == STATE_COLLECTING_SAMPLES) && (deviceDataMap.contains(senderAddress)))
    {
        sampleData[senderAddress][pointStartTime_ns] = avgAngle;
    }

    if ((samplingState == STATE_COLLECTING_SAMPLES) && (senderAddress == settings.synchronizationSource) && ((pointStartTime_ns - trigTime_ns) > settings.chartWidth * 1e9))
    {
        for (auto deviceData = deviceDataMap.constBegin(); deviceData != deviceDataMap.constEnd(); deviceData++)
        {
            quint32 ipAddress = deviceData.key();
            const DeviceData& data = deviceData.value();

            if (sampleData.contains(ipAddress))
            {
                QVector<QPointF> points;

                for (auto angleSample = sampleData[ipAddress].constBegin(); angleSample != sampleData[ipAddress].constEnd(); angleSample++)
                {
                    points.push_back(QPointF((angleSample.key() - trigTime_ns) * 1e-9, angleSample.value()));
                }

                data.lineSeries->replace(points);
                data.scatterSeries->replace(points);
            }
            else
            {
                data.lineSeries->clear();
                data.scatterSeries->clear();
            }
        }

        sampleData.clear();
        samplingState = STATE_WAITING_FOR_TRIGGER;
    }


    dbgTrap = 123;
}





void LivoxMid360ChartForm::on_pushButton_Activate_clicked()
{
    if (mid360Thread)
    {
        if (!connectLivoxMid360ThreadSlots())
        {
            // Should not happen
            return;
        }
    }

    QHostAddress syncSourceIPAddressNotValidated;

    if (!syncSourceIPAddressNotValidated.setAddress(ui->lineEdit_SyncSourceIpAddress->text()))
    {
        QMessageBox msgBox;
        msgBox.setText("Can't convert sync source to IP-address.");
        msgBox.exec();
        ui->lineEdit_SyncSourceIpAddress->setFocus();
        ui->lineEdit_SyncSourceIpAddress->selectAll();
        return;
    }

    bool convOk = false;
    settings.synchronizationSource = syncSourceIPAddressNotValidated.toIPv4Address(&convOk);

    if (!convOk)
    {
        QMessageBox msgBox;
        msgBox.setText("Can't convert sync source to IPV4-address.");
        msgBox.exec();
        ui->lineEdit_SyncSourceIpAddress->setFocus();
        ui->lineEdit_SyncSourceIpAddress->selectAll();
        return;
    }

    settings.triggerLevel = ui->doubleSpinBox_TriggerLevel->value();
    settings.triggerInverted = ui->checkBox_InvertedTrigger->isChecked();
    settings.chartWidth = ui->doubleSpinBox_ChartWidth->value();
    settings.minimumDistance = ui->doubleSpinBox_MinimumDistance->value();
    settings.angleAxisMin = ui->doubleSpinBox_AngleAxisMin->value();
    settings.angleAxisMax = ui->doubleSpinBox_AngleAxisMax->value();

    if (settings.angleAxisMax <= settings.angleAxisMin)
    {
        QMessageBox msgBox;
        msgBox.setText("angle axis max must be bigger than min.");
        msgBox.exec();
        ui->doubleSpinBox_AngleAxisMax->setFocus();
        ui->doubleSpinBox_AngleAxisMax->selectAll();
        return;
    }

    try
    {
        ui->chartView->chart()->removeAllSeries();  // This also deletes all series objects!!!
        deviceDataMap.clear();  // Note: No need for destructors, see above ^

        constructDeviceSettingsMap();

        for (auto deviceSettingsItem = settings.deviceSettingsMap.constBegin(); deviceSettingsItem != settings.deviceSettingsMap.constEnd(); deviceSettingsItem++)
        {
            DeviceData newDeviceData;

            newDeviceData.lineSeries = new QtCharts::QLineSeries();
            chart->addSeries(newDeviceData.lineSeries);
            newDeviceData.lineSeries->setUseOpenGL(true);
            newDeviceData.lineSeries->attachAxis(timeAxis);
            newDeviceData.lineSeries->attachAxis(angleAxis);
            newDeviceData.lineSeries->setName(deviceSettingsItem.value().name);
            newDeviceData.lineSeries->setColor(QColor(deviceSettingsItem.value().lineColor));


            newDeviceData.lineSeries->append(-0.1, -5);
            newDeviceData.lineSeries->append(-0.09, 22);
            newDeviceData.lineSeries->append(-0.08, 40);
            newDeviceData.lineSeries->append(-0.04, 52);
            newDeviceData.lineSeries->append(0, -7);


            newDeviceData.scatterSeries = new QtCharts::QScatterSeries();
            chart->addSeries(newDeviceData.scatterSeries);
            newDeviceData.scatterSeries->setUseOpenGL(true);
            newDeviceData.scatterSeries->attachAxis(timeAxis);
            newDeviceData.scatterSeries->attachAxis(angleAxis);
            newDeviceData.scatterSeries->setName(deviceSettingsItem.value().name);
            newDeviceData.scatterSeries->setColor(QColor(deviceSettingsItem.value().dotColor));
            newDeviceData.scatterSeries->setMarkerSize(5);


            newDeviceData.scatterSeries->append(-0.1, 0);
            newDeviceData.scatterSeries->append(-0.09, 10);
            newDeviceData.scatterSeries->append(-0.08, 20);
            newDeviceData.scatterSeries->append(-0.04, 40);
            newDeviceData.scatterSeries->append(0, 57);

            deviceDataMap[deviceSettingsItem.key()] = newDeviceData;
        }

    }
    catch (Issue& issue)
    {
        QMessageBox msgBox;
        msgBox.setText(issue.text);
        msgBox.exec();

        QTextCursor cursor = ui->plainTextEdit->textCursor();
        cursor.setPosition(issue.beginChar);
        if (issue.endChar != -1)
        {
            cursor.setPosition(issue.endChar, QTextCursor::KeepAnchor);
        }
        else
        {
            cursor.setPosition(issue.beginChar + 1, QTextCursor::KeepAnchor);
        }
        ui->plainTextEdit->setTextCursor(cursor);
        ui->plainTextEdit->setFocus();

        return;
    }

    active = true;

    ui->pushButton_Activate->setEnabled(false);
    ui->pushButton_Deactivate->setEnabled(true);
}


void LivoxMid360ChartForm::on_pushButton_Deactivate_clicked()
{
    if (mid360Thread)
    {
        disconnectLivoxMid360ThreadSlots();
    }

    ui->pushButton_Activate->setEnabled(true);
    ui->pushButton_Deactivate->setEnabled(false);

    active = false;
}

void LivoxMid360ChartForm::constructDeviceSettingsMap(void)
{
    settings.deviceSettingsMap.clear();

    QString plainText = ui->plainTextEdit->toPlainText();
    int plainTextLength = plainText.length();
    int charIndex = 0;

    while (charIndex < plainTextLength)
    {
        TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

        if (charIndex >= plainTextLength)
        {
            break;
        }

        int deviceIPAddressStartIndex = charIndex;
        QString deviceIPAddressString = TextBlockParser::getSubString(plainText, charIndex, " \t\n{");

        QHostAddress deviceIPAddressNotValidated;

        if (!deviceIPAddressNotValidated.setAddress(deviceIPAddressString))
        {
            Issue error;
            error.beginChar = deviceIPAddressStartIndex;
            error.endChar = charIndex;
            error.text = "Device IP address \"" + deviceIPAddressString + "\" not valid.";
            throw error;
        }

        bool convOk = false;
        quint32 deviceIPV4Address = deviceIPAddressNotValidated.toIPv4Address(&convOk);

        if (!convOk)
        {
            Issue error;
            error.beginChar = deviceIPAddressStartIndex;
            error.endChar = charIndex;
            error.text = "Device IP address \"" + deviceIPAddressString + "\" not valid IPV4-address.";
            throw error;
        }


        int deviceNameEndIndex = charIndex;

        auto device = deviceDataMap.constBegin();

        while (device != deviceDataMap.constEnd())
        {
            if (device.key() == deviceIPV4Address)
            {
                Issue error;
                error.beginChar = deviceIPAddressStartIndex;
                error.endChar = charIndex;
                error.text = "Duplicate device: \"" + deviceIPAddressString + "\".";
                throw error;
            }

            device++;
        }

        TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

        if (charIndex >= plainText.length())
        {
            Issue error;
            error.beginChar = deviceIPAddressStartIndex;
            error.endChar = deviceNameEndIndex;
            error.text = "All definitions missing for device \"" + deviceIPAddressString + "\".";
            throw error;
        }

        if (plainText.at(charIndex) == '{')
        {
            Issue error;
            error.beginChar = deviceIPAddressStartIndex;
            error.endChar = deviceNameEndIndex;
            error.text = "Name missing for device \"" + deviceIPAddressString + "\".";;
            throw error;
        }

        QString deviceNameString = TextBlockParser::getSubString(plainText, charIndex, " \t\n{");

        TextBlockParser::skipWhitespacesAndComments(plainText, charIndex);

        if (charIndex >= plainText.length())
        {
            Issue error;
            error.beginChar = deviceIPAddressStartIndex;
            error.endChar = deviceNameEndIndex;
            error.text = "Definitions missing for device \"" + deviceIPAddressString + "\".";
            throw error;
        }

        if (plainText.at(charIndex) != '{')
        {
            Issue error;
            error.beginChar = charIndex;
            error.endChar = charIndex + 1;
            error.text = "Only comments and whitespaces allowed between device ip and opening curly brace for parameter definition block.";
            throw error;
        }

        int settingsBlockStartIndex = charIndex;
        QString settingsBlockContents = TextBlockParser::getTextBlockAsString(plainText, charIndex, true);
//        int settingsBlockEndIndex = charIndex;

        Settings::DeviceSettings deviceSettings;

        deviceSettings.name = deviceNameString;

        getDeviceSettingsFromBlock(settingsBlockContents, settingsBlockStartIndex + 1, deviceSettings);
        settings.deviceSettingsMap.insert(deviceIPV4Address, deviceSettings);
    }
}


void LivoxMid360ChartForm::getDeviceSettingsFromBlock(const QString &blockString, const int blockStartCharIndex, Settings::DeviceSettings &settings)
{
    double vals[4];
    int charIndex = 0;
    int blockStringLength = blockString.length();

    try
    {
        for (int i = 0; i < 4; i++)
        {
            TextBlockParser::skipWhitespacesAndComments(blockString, charIndex);

            if (charIndex >= blockStringLength)
            {
                Issue error;
                error.beginChar = blockStartCharIndex;
                error.endChar = blockStringLength + blockStartCharIndex;
                error.text = "At least some settings for the device are missing.";
                throw error;
            }

            if (blockString.at(charIndex) != '{')
            {
                Issue error;
                error.beginChar = charIndex + blockStartCharIndex;
                error.endChar = charIndex + blockStartCharIndex+ 1;
                error.text = "Only comments and whitespaces allowed in device settings block outside the value fields.";
                throw error;
            }

            int settingsFieldBlockStartIndex = charIndex + blockStartCharIndex;
            QString settingsFieldBlockContents = TextBlockParser::getTextBlockAsString(blockString, charIndex);

            vals[i] = evaluateBlockContents(settingsFieldBlockContents, settingsFieldBlockStartIndex + 1); // + 1 for '{'
        }

        TextBlockParser::skipWhitespacesAndComments(blockString, charIndex);

        if ((charIndex < blockStringLength) && (blockString.at(charIndex) == '{'))
        {
            Issue error;
            error.beginChar = charIndex + blockStartCharIndex;
            error.endChar = charIndex + blockStartCharIndex + 1;
            error.text = "Opening curly brace after settings fields (only 4 fields allowed).";
            throw error;
        }

        if (charIndex < blockStringLength)
        {
            Issue error;
            error.beginChar = charIndex + blockStartCharIndex;
            error.endChar = charIndex + blockStartCharIndex + 1;
            error.text = "Only comments and whitespaces allowed in settings block after the settings fields.";
            throw error;
        }

        settings.multiplier = vals[0];
        settings.offset = vals[1];
        settings.lineColor = vals[2];
        settings.dotColor = vals[3];
    }
    catch (TextBlockParser::Issue& parseIssue)
    {
        Issue error;
        error.beginChar = parseIssue.beginChar + blockStartCharIndex;
        error.endChar = parseIssue.endChar + blockStartCharIndex;
        error.text = parseIssue.text;
        throw error;
    }

}

double LivoxMid360ChartForm::evaluateBlockContents(const QString& blockString, const int blockStartCharIndex)
{
    QByteArray expression_8bit;
    TextBlockParser::CommentState cState;

    for (int i = 0; i < blockString.length(); i++)
    {
        bool inComment = TextBlockParser::isInComment(blockString, i, cState);

        char character = blockString.at(i).toLatin1();

        if (character == 0)
        {
            if (inComment)
            {
                character = '?';
            }
            else
            {
                Issue error;
                error.text = "Only Latin 1 (ISO/IEC 8859-1 / \"8-bit ASCII\") characters allowed in non-comment sections of an expression.";
                error.beginChar = i + blockStartCharIndex;
                error.endChar = i + blockStartCharIndex + 1;
                throw error;
            }
        }

        expression_8bit += character;
    }

    char* prevLocale = std::setlocale(LC_NUMERIC, "C");

    te_parser parser;
    double evalResult = parser.evaluate(expression_8bit.constData());

    if (prevLocale)
    {
        setlocale(LC_NUMERIC, prevLocale);
    }

    if (std::isnan(evalResult))
    {
        Issue error;

        QString qstrErrorMessage = QString::fromStdString(parser.get_last_error_message());

        if (qstrErrorMessage.isEmpty())
        {
            error.text = "Error evaluating expression. TinyExpr error: (empty).";
        }
        else
        {
            error.text = "Error evaluating expression. TinyExpr error: " + qstrErrorMessage;
        }

        int errorPos = parser.get_last_error_position();

        if (errorPos == te_parser::npos)
        {
            errorPos = 0;
        }

        error.beginChar = errorPos + blockStartCharIndex;
        error.endChar = error.beginChar;
        throw error;
    }

    if (!std::isfinite(evalResult))
    {
        // Not absolutely sure if TinyExpr++ ever returns infinite
        // (could not find a way by quickly checking).
        // Doesn't cost much to check, so why not...
        Issue error;
        error.text = "Expression returns an infinite value (plus or minus).";
        error.beginChar = blockStartCharIndex;
        error.endChar = blockString.length() + blockStartCharIndex;
        throw error;
    }

    return evalResult;
}







