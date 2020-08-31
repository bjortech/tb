cmake_minimum_required(VERSION 2.8.3)
project(tb_robot)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
#INCLUDE_DIRS include
 LIBRARIES tb_robot
)

add_executable(tb_robottf_node src/tf.cpp)
target_link_libraries(tb_robottf_node ${catkin_LIBRARIES} )
