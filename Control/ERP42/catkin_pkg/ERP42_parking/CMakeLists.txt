cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])
project(ERP42_parking)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_path_planner
CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIR})



add_executable(parking src/parking.cpp)
target_link_libraries(parking ${OpenCV_LIBRARIES})
target_link_libraries(parking ${catkin_LIBRARIES})

