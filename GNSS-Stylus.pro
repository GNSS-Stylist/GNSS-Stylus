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

# message("QMAKE_HOST.arch: $$QMAKE_HOST.arch")

# Following two defines remove Eigen's vectorization that seems to cause
# runtime assertion failure (not always, but sometimes, how?!?)
# By default it tries to use fast (SIMD) instructions for calculation, see:
# http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html
#
# Seems to be fixed in eigen V 3.4-rc1 and 3.4 at least when using mingw81-64bit on windows.
# V3.4.0 still fails (runtime assertion failure) in win32 so using non-vectorized version there.
# ("x86" in QMAKE_HOST.arch refers to 32-bit)
#
# Speed difference between default, non vectorized 32-bit build and vectorized,
# 64-bit, optimized with -O3 is only about 23 % (when creating lidarscript)
# so not worth investigating further.
win32-g++:contains(QMAKE_HOST.arch, x86):{
    DEFINES += EIGEN_DONT_VECTORIZE
    DEFINES += EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
}

INCLUDEPATH += $$PWD/Eigen $$PWD/Lidar

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++17

win32-g++:contains(QMAKE_HOST.arch, x86_64):{
    # -Optimization -O3 vs -O2 speeds generating of lidarscript about 13 %
    # (64-bit vectorized) so maybe it's worth using
    QMAKE_CXXFLAGS_RELEASE -= -O
    QMAKE_CXXFLAGS_RELEASE -= -O1
    QMAKE_CXXFLAGS_RELEASE -= -O2
    QMAKE_CXXFLAGS_RELEASE += -O3
}

win32:LIBS += -l"ws2_32"

# for gcc 9.x (not recognized by 7.x): QMAKE_CXXFLAGS += -Wno-deprecated-copy

SOURCES += \
    PostProcessing/EasyEXIF/exif.cpp \
    PostProcessing/Lidar/lidarscriptgenerator.cpp \
    PostProcessing/Lidar/pointcloudgeneratorlidar.cpp \
    PostProcessing/Stylus/moviescriptgenerator.cpp \
    PostProcessing/Stylus/pointcloudgeneratorstylus.cpp \
    PostProcessing/loscriptgenerator.cpp \
    PostProcessing/postprocessingform.cpp \
    PostProcessing/rastercameragenerator.cpp \
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
    rightclickpushbutton.cpp

HEADERS += \
    PostProcessing/EasyEXIF/exif.h \
    PostProcessing/Lidar/lidarscriptgenerator.h \
    PostProcessing/Lidar/pointcloudgeneratorlidar.h \
    PostProcessing/Stylus/moviescriptgenerator.h \
    PostProcessing/Stylus/pointcloudgeneratorstylus.h \
    PostProcessing/loscriptgenerator.h \
    PostProcessing/postprocessingform.h \
    PostProcessing/rastercameragenerator.h \
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
    rightclickpushbutton.h

FORMS += \
    PostProcessing/postprocessingform.ui \
    laserrangefinder20hzv2messagemonitorform.ui \
    Lidar/lidarchartform.ui \
    licensesform.ui \
        mainwindow.ui \
    messagemonitorform.ui \
    relposnedform.ui \
    essentialsform.ui \
    Lidar/rplidarmessagemonitorform.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
