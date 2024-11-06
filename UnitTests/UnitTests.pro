QT += testlib
QT += gui
QT += network

CONFIG += qt warn_on depend_includepath testcase

TEMPLATE = app

# Enable these two for coverage:
QMAKE_CXXFLAGS += --coverage
QMAKE_LFLAGS += --coverage

SOURCES +=  tst_lidarfiltering.cpp \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.cpp \
    ../PostProcessing/Lidar/PointFilter/expressionfilter.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.cpp \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.cpp \
    ../RPLidar/rplidarplausibilityfilter.cpp \
    tst_convexhull.cpp \
    tst_expressionfilter.cpp \
    tst_lazyevaluator.cpp \
    tst_main.cpp

HEADERS += \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/3d-quickhull/quickhull.h \
    ../PostProcessing/Lidar/PointFilter/ConvexHull/convexhull.h \
    ../PostProcessing/Lidar/PointFilter/expressionfilter.h \
    ../PostProcessing/Lidar/PointFilter/lazyevaluator.h \
    ../PostProcessing/Lidar/PointFilter/tinyexpr-plusplus/tinyexpr.h \
    ../PostProcessing/Lidar/PointFilter/tinyexprcustomfunctions.h \
    tst_convexhull.h \
    tst_expressionfilter.h \
    tst_lazyevaluator.h \
    tst_lidarfiltering.h

INCLUDEPATH += ../ ../LivoxMid360
