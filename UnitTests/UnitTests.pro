QT += testlib
QT += gui
QT += network

CONFIG += qt warn_on depend_includepath testcase
CONFIG += c++17

TEMPLATE = app

# Enable these two for coverage:
QMAKE_CXXFLAGS += --coverage
QMAKE_LFLAGS += --coverage

SOURCES +=  tst_lidarfiltering.cpp \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.cpp \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhullgenerator.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_base.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_mid360.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_rplidar.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfiltergenerator.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.cpp \
    ../RPLidar/rplidarplausibilityfilter.cpp \
    ../Util/textblockparser.cpp \
    tst_convexhull.cpp \
    tst_convexhullgenerator.cpp \
    tst_expressionfilter.cpp \
    tst_expressionfiltergenerator.cpp \
    tst_lazyevaluator.cpp \
    tst_main.cpp \
    tst_postfilter.cpp \
    tst_textblockparser.cpp

HEADERS += \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhullgenerator.h \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convhull_3d/convhull_3d.h \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_base.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_mid360.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_rplidar.h \
    ../PostProcessing/Lidar/PointFilter/expressionfiltergenerator.h \
    ../PostProcessing/Lidar/PointFilter/lazyevaluator.h \
    ../PostProcessing/Lidar/PointFilter/postfilter.h \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h \
    ../PostProcessing/Lidar/lidardevice.h \
    ../Util/textblockparser.h \
    tst_convexhull.h \
    tst_convexhullgenerator.h \
    tst_expressionfilter.h \
    tst_expressionfiltergenerator.h \
    tst_lazyevaluator.h \
    tst_lidarfiltering.h \
    tst_postfilter.h \
    tst_textblockparser.h

INCLUDEPATH += ../ ../LivoxMid360
