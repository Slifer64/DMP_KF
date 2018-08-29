cmake_minimum_required(VERSION 2.8.3)

project(math_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package(orocos_kdl)
find_package(catkin REQUIRED
  #orocos_kdl # use KDL functionalities
  roscpp
  geometry_msgs
  # OpenCV REQUIRED
  # PCL REQUIRED
  # Eigen REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs # std_msgs
  #DEPENDS opencv pcl eigen
)

SET(CODEGEN_INCLUDES
codegen/mat2quat/
)

file(GLOB CODEGEN_SOURCES
    "codegen/mat2quat/*.cpp"
)

include_directories(
include
${CODEGEN_INCLUDES}
/usr/include/eigen3
${catkin_INCLUDE_DIRS}
# ${Eigen_INCLUDE_DIRS}
)




##########################
######     MATH   ########
##########################

add_library(math-lib
  src/math.cpp
  src/quaternions.cpp
  src/convertions.cpp
  src/svf.cpp
  #src/dtw.cpp
  ${CODEGEN_SOURCES}
)

add_dependencies(math-lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(math-lib
  ${catkin_LIBRARIES}
  #orocos-kdl
  armadillo
)
