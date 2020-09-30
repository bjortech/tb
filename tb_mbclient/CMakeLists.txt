cmake_minimum_required(VERSION 2.8.3)
project(tb_mbclient)


find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  move_base_msgs
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
#INCLUDE_DIRS include
 LIBRARIES tb_mb
)
add_executable(tb_mbclient_node src/mbclient.cpp)
target_link_libraries(tb_mbclient_node ${catkin_LIBRARIES})
