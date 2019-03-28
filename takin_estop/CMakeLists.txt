project(takin_estop)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        )

find_package(Threads REQUIRED)

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs 
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(estop
        src/estop.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(estop
        ${catkin_EXPORTED_TARGETS}
        )

