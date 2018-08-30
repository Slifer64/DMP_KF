cmake_minimum_required(VERSION 3.2)
project(dmp_kf)

add_definitions(-DLINUX)

set(CMAKE_CXX_STANDARD 11) # Tell CMake to compile with C++11
set(CMAKE_AUTOMOC ON) # Tell CMake to run moc when needed.
set(CMAKE_AUTOUIC ON) # Tell CMake to run uic when needed.
set(CMAKE_AUTORCC ON) # Tell CMake to run rcc when needed

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  #OpenCV REQUIRED
  #PCL REQUIRED
  roscpp
  roslib
  message_generation
  std_msgs
  geometry_msgs
  dmp_lib
  io_lib
  ur10_robot
  autharl_core
  lwr_robot
  robot_sim
  math_lib
)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})
SET(Qt_LIBRARIES
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)

# BHand need to know that we are on Linux
add_definitions( -DLINUX )

#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
  DEPENDS opencv pcl eigen
)

set(SRC_LIST

  src/source_list/utils.cpp

	src/source_list/LogData.cpp

  src/source_list/GUI.cpp

  src/source_list/Robot/Robot.cpp
  src/source_list/Robot/UR10_Robot.cpp
  src/source_list/Robot/LWR4p_Robot.cpp
  src/source_list/Robot/Sim_Robot.cpp

  src/source_list/Controller/Controller.cpp
  src/source_list/Controller/DMP_EKF_Controller.cpp

  src/source_list/dmp_kf.cpp

  include/dmp_kf/GUI/gui/mainwindow.cpp
)

file(GLOB INCLUDE_LIST "")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/dmp_kf/GUI/gui
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  /usr/include/eigen3
)

# *************************************************
# *************************************************

add_executable(dmp_kf
  src/main.cpp ${INCLUDE_LIST} ${SRC_LIST}
)

add_dependencies(dmp_kf
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_link_libraries(dmp_kf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  dmp-lib
  io-lib
  ur10-robot
  autharl-robot
  autharl-utils
  lwr-robot
  ${Qt_LIBRARIES}
  robot_sim
  math-lib
)
