cmake_minimum_required(VERSION 2.8.3)
project(robotduo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(FILES readme.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
