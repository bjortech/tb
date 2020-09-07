cmake_minimum_required(VERSION 2.8.12)
project(tb_msgsrv)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  octomap_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
	PathsStamped.msg
	PolygonsStamped.msg
	Paths.msg
	Polygons.msg
)
add_service_files(
  DIRECTORY srv
  FILES
  oct2imgdwn.srv
  oct2imgsidedown.srv
  #oct2img.srv
  oct2img3.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  octomap_msgs
  std_msgs
  sensor_msgs
)
catkin_package(
  CATKIN_DEPENDS sensor_msgs octomap_msgs geometry_msgs std_msgs message_runtime nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
