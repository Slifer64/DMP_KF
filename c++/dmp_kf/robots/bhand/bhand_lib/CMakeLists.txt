cmake_minimum_required(VERSION 2.8.3)

project(bhand_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(-DLINUX)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ ${CMAKE_MODULE_PATH} )

find_package(orocos_kdl)
find_package(catkin REQUIRED
  orocos_kdl
  kdl_parser
  roscpp
  roslib
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS orocos_kdl
)

file(GLOB BUILD_TARGER_FILES "src/*.cpp" "src/source_list/*.cpp" "include/bhand_lib/API/src/*.cpp" "include/bhand_lib/API/src/linux/*.cpp")

include_directories(
  include
  include/bhand_lib/API
  include/bhand_lib/API/ctb-0.14
  include/bhand_lib/API/ctb-0.14/linux
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
)

add_library(${PROJECT_NAME} SHARED
  ${BUILD_TARGER_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  pcan
  PocoFoundation
  pthread
  orocos-kdl
  ${catkin_LIBRARIES}
)
