cmake_minimum_required(VERSION 2.8.3)

project(optimization_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  # OpenCV REQUIRED
  # PCL REQUIRED
  # Eigen REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp #std_msgs geometry_msgs
  #DEPENDS opencv pcl eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  # ${Eigen_INCLUDE_DIRS}
)

##########################
######     TIME   ########
##########################

add_library(optimization-lib
  src/LWL.cpp
  src/LeastSquares.cpp
)

add_dependencies(optimization-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(optimization-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
  armadillo
)
