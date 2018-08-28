cmake_minimum_required(VERSION 2.8.3)

project(dmp_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")



find_package(catkin REQUIRED COMPONENTS
  optimization_lib
  param_lib
  sigproc_lib
  math_lib
  plot_lib
  io_lib
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
  src/CanonicalClock/LinCanonicalClock.cpp

  src/GatingFunction/GatingFunction.cpp
  src/GatingFunction/LinGatingFunction.cpp
  src/GatingFunction/ExpGatingFunction.cpp
  src/GatingFunction/ConstGatingFunction.cpp
  src/GatingFunction/SigmoidGatingFunction.cpp
  src/GatingFunction/SpringDamperGatingFunction.cpp

  src/DMP/DMP_.cpp
  src/DMP/DMP.cpp
  src/DMP/DMP_bio.cpp
  src/DMP/DMP_plus.cpp
  src/DMP/DMP_VT.cpp
  src/DMP/DMP_Shannon.cpp
  src/DMP/DMP_CartPos.cpp
  src/DMP/DMP_orient.cpp

  src/DMP_utils.cpp
)

add_dependencies(dmp-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(dmp-lib
  ${catkin_LIBRARIES}
  armadillo
  optimization-lib
  param-lib
  sigproc-lib
  math-lib
  plot-lib
  io-lib
)
