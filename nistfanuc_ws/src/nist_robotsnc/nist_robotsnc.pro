QT += core
QT -= gui

TARGET = nist_robotsnc
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app

machine = woodsy



CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

#QMAKE_CXXFLAGS +=-DQt


contains(machine, woodsy){
message("Compiling for woodsy")
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nist_robotsnc/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nist_robotsnc/include/nist_robotsnc"
#INCLUDEPATH += "/usr/local/michalos/nistcrcl_ws/devel/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/devel/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/nistcrcl/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/gotraj/include"
INCLUDEPATH += "/usr/local/michalos/nistfanuc_ws/src/gokin/include"
INCLUDEPATH += "/usr/include/freetype2"

INCLUDEPATH += "/usr/include"
QMAKE_CXXFLAGS +=-DXSD_CXX11
QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-sign-compare

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"
INCLUDEPATH += "/usr/include"
INCLUDEPATH += "/usr/include/eigen3"

# Libxml2
LIBS += -L/usr/lib -lxml2
# xerces code synthesis dependency
LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"

# Boost - many could be replace by C11 std
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time
LIBS += -lboost_regex
LIBS += -lboost_log_setup
LIBS += -lboost_log
LIBS += -lboost_locale

# Local ROS libs
LIBS += -L/usr/local/michalos/nistfanuc_ws/devel/lib
LIBS += -lgokin -lgotraj

# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

QMAKE_CXXFLAGS +=-DROS
QMAKE_CXXFLAGS +=-DROS=1

}

SOURCES += \
    src/BLogging.cpp \
    src/Checkers.cpp \
    src/Communication.cpp \
    src/Controller.cpp \
    src/CrclApi.cpp \
    src/CsvLogging.cpp \
    src/fanuclrmate200idkinematics.cpp \
    src/Globals.cpp \
    src/Kinematics.cpp \
    src/MotionControl.cpp \
    src/MotionException.cpp \
    src/motomansia20dkinematics.cpp \
    src/nist_robotsnc.cpp \
    src/RCS.cpp \
    src/RCSInterpreter.cpp \
    src/RosSetup.cpp \
    src/RvizCheckers.cpp \
    src/RvizMarker.cpp \
    src/Scene.cpp \
    src/Shape.cpp

HEADERS += \
    include/nist_robotsnc/Bezier.h \
    include/nist_robotsnc/Checkers.h \
    include/nist_robotsnc/Communication.h \
    include/nist_robotsnc/confetti.h \
    include/nist_robotsnc/Controller.h \
    include/nist_robotsnc/Conversions.h \
    include/nist_robotsnc/CrclApi.h \
    include/nist_robotsnc/CsvLogging.h \
    include/nist_robotsnc/Debug.h \
    include/nist_robotsnc/Demo.h \
    include/nist_robotsnc/Globals.h \
    include/nist_robotsnc/Gripper.h \
    include/nist_robotsnc/Kinematics.h \
    include/nist_robotsnc/MotionControl.h \
    include/nist_robotsnc/MotionException.h \
    include/nist_robotsnc/nist_robotsnc.h \
    include/nist_robotsnc/RCS.h \
    include/nist_robotsnc/RCSInterpreter.h \
    include/nist_robotsnc/RosSetup.h \
    include/nist_robotsnc/RvizCheckers.h \
    include/nist_robotsnc/RvizMarker.h \
    include/nist_robotsnc/Scene.h \
    include/nist_robotsnc/Shape.h \
    include/nist_robotsnc/Test.h \
    include/nist_robotsnc/trajectoryMaker.h \
    include/nist_robotsnc/ttt.h \
    include/nist_robotsnc/NIST/Boost.h \
    include/nist_robotsnc/NIST/CMath.h \
    include/nist_robotsnc/NIST/Config.h \
    include/nist_robotsnc/NIST/RCSMsgQueue.h \
    include/nist_robotsnc/NIST/RCSMsgQueueThread.h \
    include/nist_robotsnc/NIST/RCSThreadTemplate.h \
    include/nist_robotsnc/NIST/RCSTimer.h

DISTFILES += \
    include/nist_robotsnc/NIST/RCSTimer.txt

