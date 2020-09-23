cmake_minimum_required(VERSION 2.8.3)
project(tb_simulate_hardware)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2_ros
	sensor_msgs
)

catkin_package(
)
include_directories(
    ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(tb_simulate_actuators_node src/actuators.cpp)
target_link_libraries(tb_simulate_actuators_node ${catkin_LIBRARIES})

add_executable(tb_simulate_drone_node src/drone.cpp)
target_link_libraries(tb_simulate_drone_node ${catkin_LIBRARIES})
