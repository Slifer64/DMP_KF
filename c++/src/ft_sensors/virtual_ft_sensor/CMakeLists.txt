cmake_minimum_required(VERSION 2.8.3)

project(virtual_ft_sensor)

# Tell CMake to compile with C++11
set(CMAKE_CXX_STANDARD 11)

# Tell CMake to run moc when needed.
set(CMAKE_AUTOMOC ON)

# Tell CMake to run uic when needed.
set(CMAKE_AUTOUIC ON)

# Tell CMake to run rcc when needed
set(CMAKE_AUTORCC ON)

# Moc generated files are located in the current dir so we need to tell CMake to look for them.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED
  roscpp
  roslib
  std_msgs
  geometry_msgs
)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
# Add Qt5 definitions and includes to build libraries.
# Widgets add Widgets Core and Gui
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDES})

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
  include
  include/virtual_ft_sensor/gui
  ${catkin_INCLUDE_DIRS}
)

add_executable(virtual_ft_sensor
  src/virtual_ft_sensor.cpp
  src/virtual_ft_sensor_node.cpp
  include/virtual_ft_sensor/gui/slider.cpp
  include/virtual_ft_sensor/gui/mainwindow.cpp
)

add_dependencies(virtual_ft_sensor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(virtual_ft_sensor
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
  Qt5::Gui
)
