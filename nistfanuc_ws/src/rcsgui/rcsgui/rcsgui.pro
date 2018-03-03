#-------------------------------------------------
#
# Project created by QtCreator 2017-04-22T13:20:44
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = rcsgui
TEMPLATE = app
DESTDIR=../../devel/rcsgui

INCLUDEPATH += /usr/include/eigen3

unix{
LIBS += -L/usr/lib  -lboost_thread -lboost_system
LIBS += -L/opt/ros/indigo/lib -lroscpp -lroslib -lrosconsole -lrostime -ltf -ltf2
LIBS += -lroscpp_serialization -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
}

QMAKE_CXXFLAGS += -Wno-unused-parameter
QMAKE_CXXFLAGS += -fpermissive
QMAKE_CXXFLAGS +=-Wno-deprecated

SOURCES += main.cpp\
        mainwindow.cpp \
    tablevector.cpp \
    rosthread.cpp

HEADERS  += mainwindow.h \
    tablevector.h \
    NIST/Globals.h \
    rosthread.h \
    NIST/Debug.h \
    NIST/Conversions.h \
    NIST/Boost.h

FORMS    += mainwindow.ui

OTHER_FILES += \
    Notes.txt
