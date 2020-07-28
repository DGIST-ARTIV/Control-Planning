cmake_minimum_required(VERSION 3.0.2)
project(local_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES local_path_planner
CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(local_path_planner_node src/local_path_planner.cpp)
target_link_libraries(local_path_planner_node ${catkin_LIBRARIES})


