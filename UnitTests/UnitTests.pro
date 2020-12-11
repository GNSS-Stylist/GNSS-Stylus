QT += testlib
QT += gui
CONFIG += qt warn_on depend_includepath testcase

TEMPLATE = app

SOURCES +=  tst_lidarfiltering.cpp \
    ../Lidar/rplidarplausibilityfilter.cpp
