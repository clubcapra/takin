cmake_minimum_required(VERSION 2.8.3)
project(capra_thermal)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capra_thermal
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ../../include/
  ./include/flir_lepton/
)

add_executable(video src/lepton_camera/src/video/lepton.cpp src/lepton_camera/src/SPI/SPI.cpp)
target_link_libraries(video ${catkin_LIBRARIES})

add_executable(i2c src/I2C_Lepton/main.cpp src/I2C_Lepton/I2C_Interface.cpp)
target_link_libraries(i2c ${catkin_LIBRARIES})

add_executable(capra_thermal src/capra_thermal.cpp)
target_link_libraries(capra_thermal ${catkin_LIBRARIES})