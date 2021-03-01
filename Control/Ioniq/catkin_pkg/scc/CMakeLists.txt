cmake_minimum_required(VERSION 3.0.2)
project(scc)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  tracking_msg
  OpenCV REQUIRED
)

catkin_package(
#  INCLUDE_DIRS include
CATKIN_DEPENDS roscpp std_msgs tracking_msg geometry_msgs
#  DEPENDS system_lib
)

include_directories(${OpenCV_INCLUDE_DIR}
	include
	${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIR})

add_executable(scc
	src/logger.cpp
	src/scc.cpp
	src/scc_node.cpp
)

target_link_libraries(scc ${catkin_LIBRARIES})
target_link_libraries(scc ${OpenCV_LIBRARIES})