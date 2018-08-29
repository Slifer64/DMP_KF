cmake_minimum_required(VERSION 2.8.3)
project(lwr_robot)

# Use C++ for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

find_package(FastResearchInterfaceLibrary REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
  std_msgs
  autharl_core
  roslint
)

link_libraries(pthread)

catkin_package(INCLUDE_DIRS
  include
  ${FastResearchInterfaceLibrary_INCLUDE_DIR})

include_directories(include
  ${FastResearchInterfaceLibrary_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS})

add_library(lwr-robot
  src/lwr_robot/lwr_robot.cpp
  src/lwr_robot/lwr_model.cpp)

add_dependencies(lwr-robot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(lwr-robot
  ${catkin_LIBRARIES}
  autharl-robot
  armadillo
  ${FastResearchInterfaceLibrary_LIBRARIES})

## Run roslint with `catkin_make roslint`
set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# Roslint macro
roslint_cpp()
