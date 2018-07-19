cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})

# Build the stand-alone test program
add_library(capra_robot_controller SHARED capra_robot_controller.cc)
target_link_libraries(
    capra_robot_controller ${GAZEBO_libraries} ${roscpp_LIBRARIES})

if (${gazebo_VERSION_MAJOR} LESS 6)
    include(FindBoost)
    find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

    target_link_libraries(
        capra_robot_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

else()

    target_link_libraries(
        capra_robot_controller ${GAZEBO_LIBRARIES})
        
endif()