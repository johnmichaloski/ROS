cmake_minimum_required(VERSION 2.8.3)
project(cartesian_trajectory_msg)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  std_msgs 
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
)


## Declare ROS messages and services
add_message_files(  DIRECTORY msg FILES   
CartesianTrajectoryGoal.msg 
CartesianTolerance.msg 
CartesianTrajectoryPoint.msg
CartesianTrajectoryResult.msg
CartesianTrajectoryError.msg
)

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
  LIBRARIES
    
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
   trajectory_msgs
    geometry_msgs
    message_runtime
  DEPENDS

)

###########
## Build ##
###########
# set(CMAKE_CXX_COMPILER "g++")
#set(CMAKE_CXX_FLAGS "-Wdeprecated-declarations ${CMAKE_CXX_FLAGS}")



#############
## Install ##
#############

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

