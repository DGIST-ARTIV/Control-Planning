cmake_minimum_required(VERSION 3.0.2)
project(lane_driving)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
)

find_package (OpenCV REQUIRED)

install(FILES
  launch/pure_pursuit.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pure_pursuit
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_driving hlk_switch lane_change
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_driving scripts/lane_driving.cpp)
add_executable(hlk_switch scripts/hlk_switch.cpp)
add_executable(lane_change scripts/lane_change.cpp)
target_link_libraries(lane_driving ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(hlk_switch ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(lane_change ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
