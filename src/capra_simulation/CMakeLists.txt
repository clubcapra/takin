cmake_minimum_required(VERSION 2.8.3)
project(capra_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp    
    rospy
    tf
    std_msgs
    geometry_msgs
    rviz
    gazebo_ros
    gmapping
    move_base
    map_server
    amcl
)

catkin_package(

)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(capra_odometry_simulation_publisher src/capra_odometry_simulation_publisher.cpp)

target_link_libraries(capra_odometry_simulation_publisher
  ${catkin_LIBRARIES}
)
