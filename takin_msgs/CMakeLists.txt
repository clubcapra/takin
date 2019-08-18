project(takin_msgs)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
        message_generation
        )

#######################################
## Declare ROS messages and services ##
#######################################


add_service_files(
        FILES
        BrakeMode.srv
)
generate_messages(
        DEPENDENCIES
)
catkin_package(
        CATKIN_DEPENDS message_runtime
)
