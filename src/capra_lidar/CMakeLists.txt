cmake_minimum_required(VERSION 2.8.3)
project(capra_lidar)

find_package(catkin REQUIRED COMPONENTS
  sick_tim
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_thermal
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
