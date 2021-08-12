cmake_minimum_required(VERSION 3.0.2)
project(scout_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scout_python
#  CATKIN_DEPENDS rospy scout_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
#  include
  ${catkin_INCLUDE_DIRS}
)
