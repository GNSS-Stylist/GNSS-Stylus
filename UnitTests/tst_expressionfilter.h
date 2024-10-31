#ifndef TST_EXPRESSIONFILTER_H
#define TST_EXPRESSIONFILTER_H

#include <QtTest>
#include <QCoreApplication>
#include <QRandomGenerator>
#include "Eigen/Geometry"
#include "../PostProcessing/Lidar/PointFilter/expressionfilter.h"

class TestExpressionFilter : public QObject
{
    Q_OBJECT

public:
    TestExpressionFilter();
    ~TestExpressionFilter();

private:
    static const unsigned int defaultTestRounds = 10000;
    static const unsigned int filterBufferLength = PointFilter::ExpressionFilter::bufferLength;

    QRandomGenerator randomGenerator;
    Eigen::Vector3d getRandomVec(double lowLimit = -10, double highLimit = 10);
    Eigen::Transform<double, 3, Eigen::Affine> getRandomTransform(double translateLowLimit = -10.0, double translateHighLimit = 10.0);
    bool compareVectors(const Eigen::Vector3d vec1, const Eigen::Vector3d& vec2);

    PointFilter::ExpressionFilter::OutItem getRandomOutItem(void);
    LivoxMid360::PointCloudData::Point getRandomLidarSourcePoint(const quint8 propertyMask = 0x3f);

private slots:
    void initTestCase();
    void cleanupTestCase();
    void expressionValidity_ValidExpressions();
    void expressionValidity_InvalidExpressions();
    void noData();
    void defaultExpressions();
    void pureFunctions();
    void lidarCoords();
    void lidarCoords_Indexed();
    void lidarDistance();
    void lidarDistance_Indexed();

    void rigCoords_DefaultTransform();
    void rigCoords_Indexed_DefaultTransform();
    void nedCoords_DefaultTransform();
    void nedCoords_Indexed_DefaultTransform();
    void rigAndNEDCoords_RandomTransforms();
};

#endif // TST_EXPRESSIONFILTER_H
