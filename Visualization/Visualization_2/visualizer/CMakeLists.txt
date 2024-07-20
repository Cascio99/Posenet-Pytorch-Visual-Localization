cmake_minimum_required(VERSION 3.0.2)
project(visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_filters
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES visualizer
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs visualization_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/rviz_node.py
  src/data_publish_node.py
  src/posenet_pytorch/test.py
  src/image_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)