cmake_minimum_required(VERSION 3.0.2)
project(my_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

catkin_package(
  DEPENDS dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)