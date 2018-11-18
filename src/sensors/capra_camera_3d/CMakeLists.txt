cmake_minimum_required(VERSION 2.8.3)
project(capra_camera_3d)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rgbd_launch
  astra_camera
  astra_launch
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_thermal
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ../../include/
)