cmake_minimum_required(VERSION 2.8.3)
project(nistcrcl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
moveit_core
roscpp
cmake_modules
std_msgs 
sensor_msgs
geometry_msgs
trajectory_msgs
message_generation
)

find_package(Boost REQUIRED system thread chrono regex)
find_package(Eigen REQUIRED)

## Declare ROS messages and services
add_message_files(  DIRECTORY msg FILES   CrclCommandMsg.msg CrclStatusMsg.msg CrclMaxProfileMsg.msg)

## Generate added messages and services
generate_messages(
DEPENDENCIES
std_msgs
sensor_msgs
geometry_msgs
trajectory_msgs

)

#The catkin_package function specifies which dependencies your package exports to its dependencies. In particular, #INCLUDE_DIRS specifies which directory in your package contains the header directory that should be on the include #path for any packages that compile against your package.

catkin_package(
INCLUDE_DIRS
include
include/${PROJECT_NAME}/CrclXsd
LIBRARIES

CATKIN_DEPENDS
roscpp
moveit_core
sensor_msgs
trajectory_msgs
geometry_msgs
message_runtime
DEPENDS
Boost
Eigen
)

###########
## Build ##
###########
# set(CMAKE_CXX_COMPILER "g++")
#set(CMAKE_CXX_FLAGS "-Wdeprecated-declarations ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")

message("DEBUG variable catkin_INCLUDEDIR: ${catkin_INCLUDEDIR}")
message("DEBUG variable Eigen_INCLUDE_DIRS:  ${Eigen_INCLUDE_DIRS}")

include_directories(include
${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${Eigen_INCLUDE_DIRS}
# include breakdown for this project
include/${PROJECT_NAME}
include/${PROJECT_NAME}/CrclXsd
include/${PROJECT_NAME}/NIST
/usr/include/eigen3
)
# Codesynthesis include
# FMP
include_directories(
/usr/include/xsd/cxx/xml
/usr/local/include/xsd/cxx/xml
${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# Apache xercesc include
include_directories(/usr/include/xercesc)


add_executable(nistcrcl

src/CRCL/CRCLCommandInstance.cxx
src/CRCL/CRCLCommands.cxx 
src/CRCL/CRCLProgramInstance.cxx
src/CRCL/CRCLStatus.cxx
src/CRCL/DataPrimitives.cxx
src/RCS.cpp
src/crcl.cpp
src/CrclInterface.cpp
src/AsioCrclServer.cpp
src/Globals.cpp
src/Setup.cpp
src/Crcl2Ros.cpp
src/nistcrcl.cpp
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*)
FOREACH(child ${children})
IF(IS_DIRECTORY ${child})
file(GLOB_RECURSE dir_files "${child}/*")
LIST(APPEND extra_files ${dir_files})
ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})



link_directories(/usr/lib/x86_64-linux-gnu/)

target_link_libraries(nistcrcl 
libtinyxml.a  
libxerces-c.a  
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)

add_dependencies(nistcrcl ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# http://wiki.ros.org/catkin/CMakeLists.txt
install(
DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

