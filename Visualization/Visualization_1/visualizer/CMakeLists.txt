cmake_minimum_required(VERSION 3.0.2)
project(visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_filters
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES visualizer
 CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/rviz_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)