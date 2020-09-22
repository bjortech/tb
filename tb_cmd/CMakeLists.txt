cmake_minimum_required(VERSION 2.8.3)
project(tb_cmd)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  move_base_msgs
)


catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS})

add_executable(tb_cmd_actuate_node src/cmd_actuate.cpp)
target_link_libraries(tb_cmd_actuate_node ${catkin_LIBRARIES} )

add_executable(tb_cmd_exequte_node src/cmd_exequte.cpp)
target_link_libraries(tb_cmd_exequte_node ${catkin_LIBRARIES} )

add_executable(tb_cmd_setpoint_node src/cmd_setpoint.cpp)
target_link_libraries(tb_cmd_setpoint_node ${catkin_LIBRARIES} )
