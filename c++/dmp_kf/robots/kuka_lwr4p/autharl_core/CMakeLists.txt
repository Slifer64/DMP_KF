cmake_minimum_required(VERSION 2.8.3)

project(autharl_core)

# Use C++  for initialise an std::vector without a constructor
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(orocos_kdl)
find_package(catkin REQUIRED
  orocos_kdl # use KDL functionalities
  rviz_visual_tools
  roslint
)

catkin_package(
        INCLUDE_DIRS include
        DEPENDS orocos_kdl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(autharl-utils
  src/utils/string.cpp
  src/utils/kdl_ros.cpp
  src/utils/kdl_arma.cpp
  src/utils/arma_ros.cpp
  src/utils/kdl_eigen.cpp
)

add_dependencies(autharl-utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(autharl-utils
  ${catkin_LIBRARIES}
  orocos-kdl
)

add_library(autharl-math
  src/math/rotation.cpp
  src/math/plane.cpp
  src/math/skew_symmetric.cpp
  src/math/orientation.cpp
  src/math/filters.cpp
  src/math/screw.cpp
)

add_dependencies(autharl-math
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(autharl-math
  ${catkin_LIBRARIES}
  orocos-kdl
  armadillo
)

add_library(autharl-robot
  src/robot/model.cpp
  src/robot/state.cpp
  src/robot/robot.cpp
  src/robot/robot_sim.cpp
  src/robot/controller.cpp
  src/robot/arm_hand_controller.cpp
  src/robot/trajectories.cpp
)

add_dependencies(autharl-robot
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(autharl-robot
  ${catkin_LIBRARIES}
  autharl-utils
  armadillo
)

add_library(autharl-viz
  src/viz/rvisualizer.cpp
  src/viz/ros_state_publisher.cpp
)

add_dependencies(autharl-viz
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(autharl-viz
  ${catkin_LIBRARIES}
  autharl-robot
  autharl-math
)


# Compile tests
catkin_add_gtest( test_autharl_core
  test/src/test_autharl_core.cpp
  test/src/math/test_rotation.cpp
  test/src/math/test_plane.cpp
  test/src/math/test_skew_symmetric.cpp
  test/src/math/test_orientation.cpp
  test/src/utils/test_string.cpp
  test/src/utils/test_map_kdl_to_arma.cpp
  test/src/utils/test_map_arma_to_kdl.cpp
  test/src/robot/test_model.cpp
)

target_link_libraries( test_autharl_core
  ${catkin_LIBRARIES}
  pthread gtest
  autharl-math
  autharl-utils
  autharl-robot
)

## Run roslint with `catkin_make roslint`
# Filter out errors with respect C++11 (for ROS-indigo)
set(ROSLINT_CPP_OPTS "--filter=-build/c++11,-runtime/references,-build/include_what_you_use")
# Roslint macro
roslint_cpp()
