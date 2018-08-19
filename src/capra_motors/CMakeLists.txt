cmake_minimum_required(VERSION 2.8.3)
project(capra_motors)

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


add_executable(capra_motors src/cmd_vel.cpp)
target_link_libraries(capra_motors ${catkin_LIBRARIES})

