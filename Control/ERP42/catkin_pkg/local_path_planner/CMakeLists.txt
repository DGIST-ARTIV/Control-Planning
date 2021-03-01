cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])
project(local_path_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tracking_msg
  visualization_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_path_planner
CATKIN_DEPENDS roscpp rospy std_msgs tracking_msg visualization_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIR})
include_directories(${PYTHON_INCLUDE_DIRS})


add_executable(local_path_planner src/local_path_planner.cpp)
add_executable(local_path_planner_movecar src/local_path_planner_no_rviz_movecar.cpp)
add_executable(local_path_planner_movecar_navpilot src/local_path_planner_no_rviz_movecar_navpilot.cpp)
target_link_libraries(local_path_planner ${OpenCV_LIBRARIES})
target_link_libraries(local_path_planner ${catkin_LIBRARIES})

target_link_libraries(local_path_planner_movecar ${catkin_LIBRARIES})
target_link_libraries(local_path_planner_movecar ${PYTHON_LIBRARIES})
target_link_libraries(local_path_planner_movecar ${OpenCV_LIBRARIES})

target_link_libraries(local_path_planner_movecar_navpilot ${catkin_LIBRARIES})
target_link_libraries(local_path_planner_movecar_navpilot ${PYTHON_LIBRARIES})
target_link_libraries(local_path_planner_movecar_navpilot ${OpenCV_LIBRARIES})
