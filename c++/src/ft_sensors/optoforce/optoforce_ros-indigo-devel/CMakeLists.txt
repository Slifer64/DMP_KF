cmake_minimum_required(VERSION 2.8.3)
project(optoforce_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
  optoforce
  omd
)
include_directories(${catkin_INCLUDE_DIRS})
## System dependencies are found with CMake's conventions
find_package( Boost REQUIRED COMPONENTS system chrono)

## Check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

add_action_files(
  DIRECTORY action
  FILES OptoForce.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs # Or other packages containing msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(${optoforce_INCLUDE_DIRS})
include_directories(${omd_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(optoforce_node src/optoforce_topic.cpp src/optoforce_node.cpp)
add_executable(optoforce_action_node src/optoforce_action_server.cpp src/optoforce_node.cpp)
add_executable(optoforce_action_client_example src/optoforce_action_client.cpp)


target_link_libraries(optoforce_node ${optoforce_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(optoforce_action_node ${optoforce_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(optoforce_action_client_example ${catkin_LIBRARIES})

## Add cmake target dependencies of the library
add_dependencies(optoforce_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(optoforce_action_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(optoforce_action_client_example ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS optoforce_node optoforce_action_node optoforce_action_client_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
