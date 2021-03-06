#-------------------------------------------------
#
# Project created by QtCreator 2019-03-24T16:37:43
#
#-------------------------------------------------

QT       += core gui serialport charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets multimedia

TARGET = GNSS-Stylus
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# Following two defines remove Eigen's vectorization that seems to cause
# runtime assertion failure (not always, but sometimes, how?!?)
# By default it tries to use fast (SIMD) instructions for calculation, see:
# http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
# Here lack of vectorization may not be a big issue as speed is probably more
# limited by IO than math.
DEFINES += EIGEN_DONT_VECTORIZE
DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

INCLUDEPATH += $$PWD/Eigen $$PWD/Lidar

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17

win32:LIBS += -l"ws2_32"

# for gcc 9.x (not recognized by 7.x): QMAKE_CXXFLAGS += -Wno-deprecated-copy

SOURCES += \
    laserrangefinder20hzv2messagemonitorform.cpp \
    laserrangefinder20hzv2serialthread.cpp \
    Lidar/lidarchartform.cpp \
    Lidar/lidarchartview.cpp \
    licensesform.cpp \
    losolver.cpp \
        main.cpp \
        mainwindow.cpp \
    gnssmessage.cpp \
    ntripthread.cpp \
    Lidar/rplidar_sdk/src/arch/rplidarplatforms.cpp \
    Lidar/rplidar_sdk/src/hal/thread.cpp \
    Lidar/rplidar_sdk/src/rplidar_driver.cpp \
    Lidar/rplidarmessagemonitorform.cpp \
    Lidar/rplidarplausibilityfilter.cpp \
    Lidar/rplidarthread.cpp \
    transformmatrixgenerator.cpp \
    ubloxdatastreamprocessor.cpp \
    serialthread.cpp \
    messagemonitorform.cpp \
    relposnedform.cpp \
    essentialsform.cpp \
    rightclickpushbutton.cpp \
    postprocessform.cpp

HEADERS += \
    laserrangefinder20hzv2messagemonitorform.h \
    laserrangefinder20hzv2serialthread.h \
    Lidar/lidarchartform.h \
    Lidar/lidarchartview.h \
    licensesform.h \
    losolver.h \
        mainwindow.h \
    gnssmessage.h \
    ntripthread.h \
    Lidar/rplidarmessagemonitorform.h \
    Lidar/rplidarplausibilityfilter.h \
    Lidar/rplidarthread.h \
    transformmatrixgenerator.h \
    ubloxdatastreamprocessor.h \
    serialthread.h \
    messagemonitorform.h \
    relposnedform.h \
    essentialsform.h \
    rightclickpushbutton.h \
    postprocessform.h

FORMS += \
    laserrangefinder20hzv2messagemonitorform.ui \
    Lidar/lidarchartform.ui \
    licensesform.ui \
        mainwindow.ui \
    messagemonitorform.ui \
    relposnedform.ui \
    essentialsform.ui \
    postprocessform.ui \
    Lidar/rplidarmessagemonitorform.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
