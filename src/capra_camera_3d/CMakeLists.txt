cmake_minimum_required(VERSION 2.8.3)
project(capra_camera_3d)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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

add_executable(capra_camera_3d src/capra_camera_3d.cpp)
target_link_libraries(capra_camera_3d ${catkin_LIBRARIES})
