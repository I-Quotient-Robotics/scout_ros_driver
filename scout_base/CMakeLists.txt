cmake_minimum_required(VERSION 3.0.2)
project(scout_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  message_generation
  roscpp
  rospy
  scout_msgs
  sensor_msgs
  serial
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES scout_base
  CATKIN_DEPENDS controller_manager hardware_interface roscpp rospy scout_msgs sensor_msgs serial
#  DEPENDS system_lib
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/scout_base.cpp
  src/scout_hardware.cpp
  src/ScoutDriver.cpp
  src/QThread.cpp
)
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)