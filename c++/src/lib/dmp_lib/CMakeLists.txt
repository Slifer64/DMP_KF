cmake_minimum_required(VERSION 2.8.3)

project(dmp_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
)

add_library(dmp-lib
  src/CanonicalClock/CanonicalClock.cpp

  src/GatingFunction/GatingFunction.cpp
  src/GatingFunction/LinGatingFunction.cpp
  src/GatingFunction/ExpGatingFunction.cpp
  src/GatingFunction/SigmoidGatingFunction.cpp

  src/trainMethods/LeastSquares.cpp
  src/trainMethods/LWR.cpp

  src/DMP/DMP.cpp
)

add_dependencies(dmp-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dmp-lib
  ${catkin_LIBRARIES}
  armadillo
)
