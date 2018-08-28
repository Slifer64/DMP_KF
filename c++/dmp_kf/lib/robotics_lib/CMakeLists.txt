cmake_minimum_required(VERSION 2.8.3)

project(robotics_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  roslib
  autharl_core
  lwr_robot
  io_lib
  time_lib
  math_lib
  # Yaml-cpp REQUIRED
  # OpenCV REQUIRED
  # PCL REQUIRED
  # Eigen REQUIRED
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
#  DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  # ${Eigen_INCLUDE_DIRS}
)

##############################
######     Robotics   ########
##############################

add_library(robotics-lib
  src/robotics.cpp
  src/jlav.cpp
  src/control/RobotController.cpp
)

add_dependencies(robotics-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robotics-lib
  ${catkin_LIBRARIES}
  armadillo
  autharl-robot
  autharl-viz
  lwr-robot
  pthread
  io-lib
  time-lib
  # yaml-cpp
  math-lib
  #orocos-kdl
)
