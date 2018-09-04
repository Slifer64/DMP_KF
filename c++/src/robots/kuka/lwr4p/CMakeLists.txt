cmake_minimum_required(VERSION 2.8.3)
project(lwr4p)

# Use C++ for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the path containing additional modules
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

find_package(FastResearchInterfaceLibrary REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  kdl_parser
  std_msgs
)

link_libraries(pthread)

catkin_package(INCLUDE_DIRS
  include
  ${FastResearchInterfaceLibrary_INCLUDE_DIR})

include_directories(include
  ${FastResearchInterfaceLibrary_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS})

add_library(lwr4p
  src/lwr4p/Robot.cpp
  # src/lwr4p/Model.cpp
)

add_dependencies(lwr4p
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(lwr4p
  ${catkin_LIBRARIES}
  armadillo
  ${FastResearchInterfaceLibrary_LIBRARIES}
)
