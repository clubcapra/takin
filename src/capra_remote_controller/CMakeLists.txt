cmake_minimum_required(VERSION 2.8.3)
project(capra_remote_controller)
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_motors
#  CATKIN_DEPENDS joy roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#set( src/controller_logitech.cpp )

add_executable(capra_remote_controller src/remote_controller.cpp )
target_link_libraries(capra_remote_controller ${catkin_LIBRARIES})
