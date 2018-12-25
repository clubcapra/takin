project(takin_msgs)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        geometry_msgs
        )

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
        FILES
        temp.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs message_runtime
)