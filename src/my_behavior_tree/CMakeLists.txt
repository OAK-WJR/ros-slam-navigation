cmake_minimum_required(VERSION 3.0.2)
project(my_behavior_tree)

set(CMAKE_PREFIX_PATH "/usr/local;${CMAKE_PREFIX_PATH}")
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behaviortree_cpp_v3
  yaml-cpp
  actionlib
)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_behavior_tree
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include/my_behavior_tree/general_task
  ${catkin_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 使用 GLOB_RECURSE 递归查找所有 .cpp 文件
file(GLOB_RECURSE SRC_FILES
  src/*.cpp
)

add_executable(${PROJECT_NAME}_node
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${behaviortree_cpp_v3_LIBRARIES}
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
