cmake_minimum_required(VERSION 2.8.3)
project(ur10)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2_ros
  math_lib
  param_lib
  io_lib
  ur_kinematics
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
)

set(SRC_LIST
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

# *************************************************
# *************************************************

add_library(ur10
  src/Robot.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(ur10
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(ur10
  ${catkin_LIBRARIES}
  armadillo
  math-lib
  param-lib
  io-lib
  ur10_kin
)


# *************************************************
# *************************************************
