cmake_minimum_required(VERSION 3.0.2)
project(movecar)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_path_planner
CATKIN_DEPENDS roscpp std_msgs visualization_msgs geometry_msgs
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
)
include_directories(${OpenCV_INCLUDE_DIR})

add_executable(movecar 
	src/movecar.cpp
	src/movecar_node.cpp
)

target_link_libraries(movecar ${catkin_LIBRARIES})
target_link_libraries(movecar ${OpenCV_LIBRARIES})
