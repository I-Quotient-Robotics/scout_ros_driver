cmake_minimum_required(VERSION 3.0.2)
project(scout_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iqr_scout_description
#  CATKIN_DEPENDS aubo_description scout_description urdf xacro
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)