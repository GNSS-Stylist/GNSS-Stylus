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
    ../PostProcessing/Lidar/PointFilter/expressionfilter_base.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_mid360.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_rplidar.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.cpp \
    ../RPLidar/rplidarplausibilityfilter.cpp \
    tst_convexhull.cpp \
    tst_expressionfilter.cpp \
    tst_lazyevaluator.cpp \
    tst_main.cpp

HEADERS += \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convhull_3d/convhull_3d.h \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_base.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_mid360.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter_rplidar.h \
    ../PostProcessing/Lidar/PointFilter/lazyevaluator.h \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h \
    tst_convexhull.h \
    tst_expressionfilter.h \
    tst_lazyevaluator.h \
    tst_lidarfiltering.h

INCLUDEPATH += ../ ../LivoxMid360
