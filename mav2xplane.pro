#-------------------------------------------------
#
# Project created by QtCreator 2016-07-12T12:43:06
#
#-------------------------------------------------

QT       += core gui network serialport

CONFIG += c++11

DEFINES += _USE_MATH_DEFINES

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = mav2xplane
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    QGC/XPlaneLink.cpp \
    QGC/UAS.cpp \
    connectionwidget/connectionwidget.cpp \
    indicator/indicator.cpp


HEADERS  += mainwindow.h \   
    QGC/XPlaneLink.h \
    QGC/UAS.h \
    connectionwidget/connectionwidget.h \
    indicator/indicator.h

    QGC/px4_custom_mode.h

INCLUDEPATH += c_library_v2/common \
               ../linkparamswgt \
               ../indicator
