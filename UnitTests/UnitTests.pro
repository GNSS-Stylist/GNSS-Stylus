QT += testlib
QT += gui
CONFIG += qt warn_on depend_includepath testcase

TEMPLATE = app

# Enable these two for coverage:
QMAKE_CXXFLAGS += --coverage
QMAKE_LFLAGS += --coverage

SOURCES +=  tst_lidarfiltering.cpp \
    ../RPLidar/rplidarplausibilityfilter.cpp \
    tst_lazyevaluator.cpp \
    tst_main.cpp

HEADERS += \
    ../PostProcessing/Lidar/PointFilter/lazyevaluator.h \
    tst_lazyevaluator.h \
    tst_lidarfiltering.h

INCLUDEPATH += ../
