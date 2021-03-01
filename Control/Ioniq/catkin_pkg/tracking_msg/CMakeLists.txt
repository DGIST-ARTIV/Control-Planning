cmake_minimum_required(VERSION 3.0.2)
project(tracking_msg)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    sensor_msgs
    std_msgs
	geometry_msgs
	visualization_msgs
)

add_message_files(
  FILES
    TrackingObject.msg
    TrackingObjectArray.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
	geometry_msgs
	visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
	geometry_msgs
	visualization_msgs
)
