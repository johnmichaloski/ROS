#http://docs.ros.org/kinetic/api/catkin/html/index.html
# https://cmake.org/Wiki/CMake_Useful_Variables

cmake_minimum_required(VERSION 2.8.3)
project(nist_robotsnc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  cmake_modules
  trajectory_msgs
  geometry_msgs
  sensor_msgs
  tf
tf_conversions
eigen_conversions
rviz_visual_tools
   moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_ros_planning
    moveit_ros_manipulation
    gotraj
   gokin
   trac_ik_lib
)

find_package(Boost REQUIRED system thread chrono regex log_setup log locale )
find_package(Eigen REQUIRED)

#The catkin_package function specifies which dependencies your package exports to its dependencies. In particular, #INCLUDE_DIRS specifies which directory in your package contains the header directory that should be on the include #path for any packages that compile against your package.

catkin_package(
  INCLUDE_DIRS
    include
#    include/${PROJECT_NAME}/CrclXsd
  LIBRARIES
    gotraj
    gokin
    trac_ik_lib
  CATKIN_DEPENDS
    roscpp
    moveit_core
    sensor_msgs
    trajectory_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_ros_planning
    moveit_ros_manipulation
    nistcrcl
    tf
tf_conversions
eigen_conversions
rviz_visual_tools
gotraj
gokin
trac_ik_lib
  DEPENDS
    Boost
    Eigen


#  CFG_EXTRAS
#    descartes_core-extras.cmake
)

###########
## Build ##
###########
# set(CMAKE_CXX_COMPILER "g++")
#set(CMAKE_CXX_FLAGS "-Wdeprecated-declarations ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DBOOST_LOG_DYN_LINK ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

#set(CMAKE_CXX_FLAGS "-DCHECKERS ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "-DBOLTDEMO ${CMAKE_CXX_FLAGS}")


set(CMAKE_EXE_LINKER_FLAGS "-rdynamic ${CMAKE_EXE_LINKER_FLAGS}")

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    # include breakdown for this project
    include/${PROJECT_NAME}
 #   include/${PROJECT_NAME}/CrclXsd
    include/${PROJECT_NAME}/NIST
   /usr/include/eigen3
)

# http://docs.ros.org/jade/api/catkin/html/user_guide/variables.html
message("DEBUG variable PROJECT_NAME        : ${PROJECT_NAME}")
message("DEBUG variable CMAKE_BINARY_DIR    : ${CMAKE_BINARY_DIR}")
#message("DEBUG variable catkin_LIBRARIES   : ${catkin_LIBRARIES}")  # This is long
#message("DEBUG variable Boost_LIBRARIES    : ${Boost_LIBRARIES}")  # This is long

message("DEBUG variable catkin_INCLUDEDIR   : ${catkin_INCLUDE_DIRS}")
message("DEBUG variable Eigen_INCLUDE_DIRS  :  ${Eigen_INCLUDE_DIRS}")
message("DEBUG variable Boost_INCLUDE_DIRS  :  ${Boost_INCLUDE_DIRS}")
message("DEBUG variable CATKIN_DEVEL_PREFIX :  ${CATKIN_DEVEL_PREFIX}")
message("DEBUG variable CMAKE_INSTALL_PREFIX:  ${CMAKE_INSTALL_PREFIX}")


# Codesynthesis include
# FMP
#include_directories(
#  /usr/include/xsd/cxx/xml
#  /usr/local/include/xsd/cxx/xml
#  )
# Apache xercesc include
#include_directories(/usr/include/xercesc)



add_executable(nist_robotsnc 
src/BLogging.cpp
src/Controller.cpp
src/CsvLogging.cpp
src/Globals.cpp
src/Kinematics.cpp
src/MotionControl.cpp
src/RCS.cpp
src/RCSInterpreter.cpp
src/RvizMarker.cpp
src/Scene.cpp
src/RosSetup.cpp
src/nist_robotsnc.cpp
src/trajectoryMaker.cpp
src/Demo.cpp
src/motomansia20dkinematics.cpp
src/fanuclrmate200idkinematics.cpp
src/MotionException.cpp
src/Shape.cpp
src/JointTrajectory.cpp
)

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB children ${CMAKE_SOURCE_DIR}/*.h)
FOREACH(child ${children})
  IF(IS_DIRECTORY ${child})
    file(GLOB_RECURSE dir_files "${child}/*")
    LIST(APPEND extra_files ${dir_files})
  ENDIF()
ENDFOREACH()
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})



link_directories(/usr/lib/x86_64-linux-gnu/)
link_directories(${CATKIN_DEVEL_PREFIX}/install)
link_directories(${CATKIN_DEVEL_PREFIX}/devel/lib)

target_link_libraries(nist_robotsnc 
libtinyxml.a  
libxerces-c.a  
# I had to add ${catkin_LIBRARIES} AND remove the reference to the shared library 
#libgotraj.so since gotraj is a Catkin library?
${catkin_LIBRARIES}
${Boost_LIBRARIES}
)


#catkin_add_gtest(crclmathunittest test/crclmathtest.cpp)

#catkin_add_gtest(conversionTest test/conversiontests.cpp)
#add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND} --verbose)
#target_link_libraries(conversionTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    
    
    
    
