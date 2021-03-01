cmake_minimum_required(VERSION 3.0.2)
project(aeb)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tracking_msg
  cv_bridge
  OpenCV REQUIRED
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aeb
  CATKIN_DEPENDS roscpp std_msgs tracking_msg
 # DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
#include_directories(${OpenCV_INCLUDE_DIR})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(aeb
  src/aeb.cpp
  src/aeb_node.cpp
)

target_link_libraries(aeb ${catkin_LIBRARIES})
target_link_libraries(aeb ${OpenCV_LIBRARIES})