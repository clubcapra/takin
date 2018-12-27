project(takin_motors)

cmake_minimum_required(VERSION 3.1)
add_definitions(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        geometry_msgs
        takin_msgs
        sensor_msgs
        )

find_package(Threads REQUIRED)
#find_package(SDL2 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#        FILES
#        msg/temperature.msg
#        #   Message2.msg
#)
# add_message_files(
#    FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#        cfg/Motor.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        #INCLUDE_DIRS include
        #LIBRARIES takin_motors
        CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime sensor_msgs takin_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if (${CMAKE_SIZEOF_VOID_P} MATCHES "8")
        add_library(${PROJECT_NAME}_lib
                src/lib/x86-64/libCTRE_Phoenix.a
                src/lib/x86-64/libCTRE_PhoenixCanutils.a
                src/lib/x86-64/libCTRE_PhoenixCCI.a
                src/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a
                )
    else ()
        add_library(${PROJECT_NAME}_lib
                src/lib/aarch64/libCTRE_Phoenix.a
                src/lib/aarch64/libCTRE_PhoenixCanutils.a
                src/lib/aarch64/libCTRE_PhoenixCCI.a
                src/lib/aarch64/libCTRE_PhoenixPlatformLinuxSocketCan.a
                )
    endif ()
endif ()

set_target_properties(${PROJECT_NAME}_lib PROPERTIES LINKER_LANGUAGE CXX)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lk_rover ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
add_executable(motors_control
        src/motors_control.cpp
        )

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(motors_control
#        ${PROJECT_NAME}_gencfg
#        )

## Specify libraries to link a library or executable target against
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if (${CMAKE_SIZEOF_VOID_P} MATCHES "8")
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_Phoenix.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCCI.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/x86-64/libCTRE_PhoenixCanutils.a)
    else ()
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_Phoenix.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCCI.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
        target_link_libraries(motors_control ${PROJECT_SOURCE_DIR}/src/lib/aarch64/libCTRE_PhoenixCanutils.a)
    endif ()
endif ()

target_link_libraries(motors_control Threads::Threads)
#target_link_libraries(motors_control ${SDL2_LIBRARIES})
target_link_libraries(motors_control ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS lk_rover lk_rover_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lk_rover.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
