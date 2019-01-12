project(takin_motors)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        geometry_msgs
        takin_msgs
        sensor_msgs
        )

find_package(Threads REQUIRED)

catkin_package(
        #INCLUDE_DIRS include
        #LIBRARIES takin_motors
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs takin_msgs
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(motors_control
        src/motors_control.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(motors_control
        ${catkin_EXPORTED_TARGETS}
        )

message("Your architecture is " ${CMAKE_LIBRARY_ARCHITECTURE})
message("Your path " ${PROJECT_SOURCE_DIR}/src)
if (${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "arm-linux-gnueabihf") # Jetson TK1 / Pi
    message("Jetson TK1 / Pi")
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_Phoenix.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCCI.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/raspberry/libCTRE_PhoenixCanutils.a)
elseif(${CMAKE_LIBRARY_ARCHITECTURE} STREQUAL "aarch64-linux-gnu") # Jetson TX2
    message("Jetson TX2")
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_Phoenix.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCCI.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCanutils.a)
else()
    message("x86-64")
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a)
endif()

target_link_libraries(motors_control Threads::Threads)
target_link_libraries(motors_control ${catkin_LIBRARIES})