#include <QtTest>
#include <QCoreApplication>

// add necessary includes here

#include <QRandomGenerator>

#include "../Lidar/rplidarplausibilityfilter.h"

class LidarFiltering : public QObject
{
    Q_OBJECT

public:
    LidarFiltering();
    ~LidarFiltering();

private:
    QRandomGenerator randomGenerator;

private slots:
    void initTestCase();
    void cleanupTestCase();
    void test_Quality_Pre();
    void test_SlopeFiltering();
};

LidarFiltering::LidarFiltering()
{

}

LidarFiltering::~LidarFiltering()
{

}

void LidarFiltering::initTestCase()
{
    randomGenerator.seed(1);
}

void LidarFiltering::cleanupTestCase()
{

}

void LidarFiltering::test_Quality_Pre()
{
    QVector<RPLidarThread::DistanceItem> itemsToFilter;
    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;

    RPLidarPlausibilityFilter filter;

    RPLidarPlausibilityFilter::Settings settings;
    settings.qualityLimit_PreFiltering = 0.5;
    filter.setSettings(settings);

    for (double angle = 0; angle < M_PI * 2; angle += (2 * M_PI) / 1000)
    {
        RPLidarThread::DistanceItem newItem;
        newItem.angle = angle;
        newItem.distance = 1 + randomGenerator.bounded(10.);
        newItem.quality = randomGenerator.bounded(1);

        itemsToFilter.push_back(newItem);
    }

    filter.filter(itemsToFilter, filteredItems);

    QCOMPARE(itemsToFilter.count(), filteredItems.count());

    for (int i = 0; i < itemsToFilter.count(); i++)
    {
        const RPLidarThread::DistanceItem& sourceItem = itemsToFilter[i];
        const RPLidarPlausibilityFilter::FilteredItem& destItem = filteredItems[i];

        QCOMPARE(sourceItem.distance, destItem.item.distance);
        QCOMPARE(sourceItem.angle, destItem.item.angle);
        QCOMPARE(sourceItem.quality, destItem.item.quality);

        if (sourceItem.quality < settings.qualityLimit_PreFiltering)
        {
            QCOMPARE(destItem.type, RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_QUALITY_PRE);
        }
        else
        {
            QCOMPARE(destItem.type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
        }
    }
}

void LidarFiltering::test_SlopeFiltering()
{
    QVector<RPLidarThread::DistanceItem> itemsToFilter;
    QVector<RPLidarPlausibilityFilter::FilteredItem> filteredItems;

    RPLidarPlausibilityFilter filter;

    RPLidarPlausibilityFilter::Settings settings;

    settings.relativeSlopeLimit = 0.1;

    filter.setSettings(settings);

    RPLidarThread::DistanceItem newItem;
    newItem.quality = 1;

    newItem.angle = 1;
    newItem.distance = 1;

    // Steady
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        itemsToFilter.push_back(newItem);
    }

    // Slow rising change (slope under limit)
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        newItem.distance *= 1 + randomGenerator.bounded(0.00099999999999);
        itemsToFilter.push_back(newItem);
    }

    // Rising change (slope over limit)
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        newItem.distance *= 1.001 + randomGenerator.bounded(0.0001);
        itemsToFilter.push_back(newItem);
    }

    // Steady
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        itemsToFilter.push_back(newItem);
    }

    // Slow downwards change (slope under limit)
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        newItem.distance /= 1 + randomGenerator.bounded(0.00099999999999);
        itemsToFilter.push_back(newItem);
    }

    // Rising angle and lowering distance (lowering slope over limit)
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        newItem.distance /= 1.01;
        itemsToFilter.push_back(newItem);
    }

    // Steady
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 0.01;
        itemsToFilter.push_back(newItem);
    }

    // Large zigzag
    for (int i = 0; i < 10; i++)
    {
        newItem.angle += 1;
        newItem.distance += 5;
        itemsToFilter.push_back(newItem);

        newItem.angle -= 1;
        newItem.distance += 5;
        itemsToFilter.push_back(newItem);
    }

    filter.filter(itemsToFilter, filteredItems);
    QCOMPARE(itemsToFilter.count(), filteredItems.count());

    int itemIndex = 0;

    // Steady
    for (int i = 0; i < 10; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Slow rising change (slope under limit)
    for (int i = 0; i < 10; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Rising change (slope over limit)
    for (int i = 0; i < 9; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_SLOPE);
    }

    // Last rising sample should not trigger filtering as "both" sides should change to the same direction
    QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);

    // Steady
    for (int i = 0; i < 10; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Slow downwards change (slope under limit)
    for (int i = 0; i < 10; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Rising angle and lowering distance (lowering slope over limit)
    for (int i = 0; i < 9; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_REJECTED_SLOPE);
    }

    // Last lowering sample should not trigger filtering as "both" sides should change to the same direction
    QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);

    // Steady
    for (int i = 0; i < 10; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Large zigzag
    for (int i = 0; i < 20; i++)
    {
        QCOMPARE(filteredItems[itemIndex++].type, RPLidarPlausibilityFilter::FilteredItem::FIT_PASSED);
    }

    // Just testing the unit test if all items are checked
    QCOMPARE(itemIndex, filteredItems.count());
}


QTEST_MAIN(LidarFiltering)

#include "tst_lidarfiltering.moc"
