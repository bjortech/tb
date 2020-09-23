cmake_minimum_required(VERSION 2.8.3)
project(tb_simulate_perception)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  tf2_ros
	sensor_msgs
	pcl_conversions
	pcl_ros
)

catkin_package(
)
include_directories(
    ${catkin_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(tb_simulate_oct2sensor_node src/oct2sensor.cpp)
target_link_libraries(tb_simulate_oct2sensor_node
    ${PCL_LIBRARIES}
		${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(tb_simulate_img2sensor_node src/img2sensor.cpp)
target_link_libraries(tb_simulate_img2sensor_node
    ${PCL_LIBRARIES}
		${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
