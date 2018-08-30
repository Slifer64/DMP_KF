cmake_minimum_required(VERSION 2.8.3)
project(kinect_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
