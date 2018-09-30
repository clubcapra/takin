cmake_minimum_required(VERSION 2.8.3)
project(capra_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  image_publisher
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_landolt_C
#  CATKIN_DEPENDS opencv3 roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)