cmake_minimum_required(VERSION 2.8.3)
project(crcl_testing)


find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_msgs
  actionlib
  nistcrcl
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf
)


catkin_python_setup()

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS})

