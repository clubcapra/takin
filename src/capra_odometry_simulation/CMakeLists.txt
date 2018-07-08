cmake_minimum_required(VERSION 2.8.3)
project(capra_odometry_simulation)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_odometry_simulation
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/capra_odometry_simulation.cpp
# )

add_executable(capra_odometry_simulation_publisher src/capra_odometry_simulation_publisher.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(capra_odometry_simulation_publisher
  ${catkin_LIBRARIES}
)