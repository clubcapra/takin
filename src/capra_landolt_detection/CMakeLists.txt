cmake_minimum_required(VERSION 2.8.3)
project(capra_landolt_detection)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  image_publisher
  roslint
  capra_msgs
)

catkin_package(
  LIBRARIES capra_nodelet_landolt
  CATKIN_DEPENDS 
    nodelet 
    cv_bridge 
    roscpp 
    sensor_msgs 
    capra_msgs 
    std_msgs 
    image_transport
  DEPENDS
    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(capra_nodelet_landolt src/LandoltNodelet.cpp)
target_link_libraries(capra_nodelet_landolt ${catkin_LIBRARIES})
add_dependencies(capra_nodelet_landolt capra_msgs_gencpp)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_executable_with_gtest(landolt_test test/landolt_test.cpp)
  add_rostest(test/capra_detection.test ARGS datapath:=${PROJECT_SOURCE_DIR}/test/sample/)
  #catkin_add_gtest(landolt_test test/landolt_test.cpp WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test/sample)
  #add_rostest(test/capra_detection.test DEPENDENCIES landolt_test)
  target_link_libraries(landolt_test ${catkin_LIBRARIES})
endif()

roslint_cpp()

install(TARGETS capra_nodelet_landolt
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)