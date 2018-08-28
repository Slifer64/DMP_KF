cmake_minimum_required(VERSION 2.8.3)

project(gmm_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl)
find_package(catkin REQUIRED
  orocos_kdl # use KDL functionalities
)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(GMM-lib
  src/GMM_GMR/GMM_GMR.cpp
)

add_dependencies(GMM-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(GMM-lib
  ${catkin_LIBRARIES}
  orocos-kdl
  armadillo
)
