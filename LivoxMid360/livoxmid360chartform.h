#ifndef LIVOXMID36_CHARTFORM_H
#define LIVOXMID36_CHARTFORM_H

#include <QWidget>
#include <QChart>
#include <QValueAxis>
#include <QLineSeries>
#include <QScatterSeries>

#include "livoxmid360thread.h"

namespace Ui {
class LivoxMid360ChartForm;
}

class LivoxMid360ChartForm : public QWidget
{
    Q_OBJECT

public:
    explicit LivoxMid360ChartForm(QWidget *parent = nullptr);
    ~LivoxMid360ChartForm();

    /**
     * @brief "Connects" LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to connect
     */
    void connectLivoxMid360Thread(LivoxMid360Thread* mid360Thread);

    /**
     * @brief "Disconnects" LivoxMid360Thread
     * @param mid360Thread LivoxMid360Thread to disconnect
     */
    void disconnectLivoxMid360Thread(LivoxMid360Thread* mid360Thread);

private slots:
    void on_pushButton_Activate_clicked();

    void on_pushButton_Deactivate_clicked();

private:
    Ui::LivoxMid360ChartForm *ui;


    class Settings
    {
    public:

        class DeviceSettings
        {
        public:
            QString name;
            double multiplier = 1;
            double offset = 0;
            int lineColor = 0x000000;
            int dotColor = 0x000000;
        };

        quint32 synchronizationSource = 0;
        double triggerLevel = 22.5;
        bool triggerInverted = false;
        double chartWidth = 0.1;
        double minimumDistance = 1;
        double angleAxisMin = -7;
        double angleAxisMax = 52;

        QMap<quint32, DeviceSettings> deviceSettingsMap;
    };

    Settings settings;

    bool active =false;
    typedef enum
    {
        STATE_WAITING_FOR_TRIGGER = 0,
        STATE_COLLECTING_SAMPLES,
    } SamplingState;

    SamplingState samplingState = STATE_WAITING_FOR_TRIGGER;
    double prevTriggerDeviceAngle = 0;
    quint64 trigTime_ns = 0;

    LivoxMid360Thread* mid360Thread = nullptr;
    void pushLidarInformationReceived(quint32 ipAddress, const LivoxMid360::PushLidarInformation&, qint64);
    void on_LivoxMid360RawDatagramReceived(const QNetworkDatagram& datagram, qint64 timeStamp);

    bool connectLivoxMid360ThreadSlots(void);
    void disconnectLivoxMid360ThreadSlots(void);

    QtCharts::QChart* chart = nullptr;
    QtCharts::QValueAxis* timeAxis = nullptr;
    QtCharts::QValueAxis* angleAxis = nullptr;

//    QtCharts::QLineSeries* lineSeries_Angle = nullptr;
//    QtCharts::QScatterSeries* scatterSeries_Angle = nullptr;

    class DeviceData
    {
    public:
        // Note: QChart "takes ownership" of series added to it and therefore it also handles their deletion
        // (tried to handle these with std::shared_ptr just causing freeing of already freed object...).
        QtCharts::QLineSeries* lineSeries = nullptr;
        QtCharts::QScatterSeries* scatterSeries = nullptr;
    };

    void getDeviceSettingsFromBlock(const QString &blockString, const int blockStartCharIndex, Settings::DeviceSettings &settings);
    double evaluateBlockContents(const QString& blockString, const int blockStartCharIndex);

    class Issue
    {
    public:
        int beginChar = -1;
        int endChar = -1;
        QString text;
    };

    QMap<quint32, DeviceData> deviceDataMap;
    void constructDeviceSettingsMap(void);

    QMap<quint32, QMap<quint64, double>> sampleData;    // first key = IP, second = time from device (ns), value = avg angle from datagram

};

#endif // LIVOXMID36_CHARTFORM_H
