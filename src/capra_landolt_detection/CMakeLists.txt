cmake_minimum_required(VERSION 2.8.3)
project(capra_landolt_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  std_msgs
  sensor_msgs
  image_publisher
  roslint
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES capra_nodelet_landolt
  CATKIN_DEPENDS nodelet cv_bridge roscpp sensor_msgs
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(capra_nodelet_landolt src/LandoltNodelet.cpp src/CMath.cpp src/CUtil.cpp)
target_link_libraries(capra_nodelet_landolt ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(capra_nodelet_landolt ${catkin_EXPORTED_LIBRARIES})
endif()

roslint_cpp()

install(TARGETS capra_nodelet_landolt
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)