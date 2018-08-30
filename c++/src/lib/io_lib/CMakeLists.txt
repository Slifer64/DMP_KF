cmake_minimum_required(VERSION 2.8.3)

project(io_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(io-lib
  src/io_utils.cpp
  src/parser.cpp
)

add_dependencies(io-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(io-lib
  ${catkin_LIBRARIES}
  armadillo
)
