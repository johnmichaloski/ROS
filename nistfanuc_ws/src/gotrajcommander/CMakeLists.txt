cmake_minimum_required(VERSION 2.8.3)
project(gotrajcommander)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
  sensor_msgs
  geometry_msgs
  tf
  gotraj
)



## Both Boost.python and Python libs are required.
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  gotrajcommander
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    gotraj
  DEPENDS
    Boost
)

###########
## Build ##
###########
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DDEBUG ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-DBOOST_LOG_DYN_LINK ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "-DGO_REAL_DOUBLE ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "-rdynamic ${CMAKE_EXE_LINKER_FLAGS}")


SET(PY_PROJECT_NAME ${PROJECTNAME})
SET(PY_PACKAGE_DIR python/gotrajcommander)


set(GOTRAJCOMMANDER_LIB_SOURCES
    src/gotrajcommander.cpp
)    

add_library(gotrajcommander ${GOTRAJCOMMANDER_LIB_SOURCES})
link_directories(${CATKIN_DEVEL_PREFIX}/lib})

# catkin will know about gotraj and include as link
target_link_libraries(gotrajcommander  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(gotrajcommander PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )  

        


