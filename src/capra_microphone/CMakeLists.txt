cmake_minimum_required(VERSION 2.8.3)
project(capra_microphone)


find_package(catkin REQUIRED COMPONENTS roscpp audio_common_msgs)


catkin_package()


add_executable(capra_microphone src/capra_listener.cpp)
target_link_libraries(capra_microphone ${catkin_LIBRARIES})
include_directories(include ${catkin_INCLUDE_DIRS})
