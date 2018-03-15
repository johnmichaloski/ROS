#-------------------------------------------------
#
# Project created by QtCreator 2017-12-30T15:23:57
#
#-------------------------------------------------

QT       += core gui widgets

TARGET = cpp_joint_state_publisher
TEMPLATE = app

CONFIG += c++11

# Folder management for builds
release: DESTDIR = ../build/$$TARGET/release
debug:   DESTDIR = ../build/$$TARGET/debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
UI_DIR = $$DESTDIR/.ui

INCLUDEPATH += $$UI_DIR
ROSVER=kinetic

# In case of multiple machine compilation
machine = woodsy


contains(machine, woodsy){
message("Compiling for woodsy")

QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-sign-compare
QMAKE_CXXFLAGS +=-Wno-unused-value
QMAKE_CXXFLAGS +=-DTIXML_USE_STL

INCLUDEPATH += "/home/isd/michalos/src/QT/ROS/$$TARGET"
INCLUDEPATH += "/home/isd/michalos/src/QT/ROS/$$TARGET/include"
INCLUDEPATH += "/usr/include"

#ROS
QMAKE_CXXFLAGS +=-DROS
QMAKE_CXXFLAGS +=-DROS=1
INCLUDEPATH += "/opt/ros/$$ROSVER/include"
INCLUDEPATH += "/usr/include"
INCLUDEPATH += "/usr/include/eigen3"

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

# tinyxml
LIBS += -ltinyxml

# Ros libs
LIBS += -L/opt/ros/$$ROSVER/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

}

SOURCES += main.cpp\
        mainwindow.cpp \
    Globals.cpp \
    urdfparser.cpp

HEADERS  += mainwindow.h \
    Config.h \
    Globals.h \
    JointStateUpdater.h \
    urdfparser.h

FORMS    += mainwindow.ui

DISTFILES += \
    Notes

OTHER_FILES += \
    Notes.txt

target.path = /usr/local/michalos/nistfanuc_ws/install/lib/cpp_joint_state_publisher
#target.extra = $${QMAKE_INSTALL_FILE}  $${TARGET} /usr/local/michalos/nistfanuc_ws/src/cpp_joint_state_publisher/scripts
INSTALLS += target

header_files.path=/usr/local/michalos/nistfanuc_ws/src/cpp_joint_state_publisher/scripts
header_files.files=$$TARGET
#message($$QMAKE_INSTALL_FILE)

#INSTALLS += target header_files


#header_files.files = urdfparser.h
#header_files.path = /usr/local/include/ros/$$ROSVER/$$TARGET
#INSTALLS += header_files
