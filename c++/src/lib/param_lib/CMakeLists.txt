cmake_minimum_required(VERSION 2.8.3)

project(param_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  # OpenCV REQUIRED
  # PCL REQUIRED
  # Eigen REQUIRED
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
  #DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
  /usr/include/eigen3
)

###############################
######     Param lib   ########
###############################

add_library(param-lib
  src/ParamList.cpp
  src/parser.cpp
)

add_dependencies(param-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(param-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
  armadillo
)
