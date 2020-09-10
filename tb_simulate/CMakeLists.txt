cmake_minimum_required(VERSION 2.8.3)
project(tb_simulate)

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

add_executable(tb_sim_heightimg_to_sensor_pointcloud_node src/heightimg_to_sensor_pointcloud.cpp)
target_link_libraries(tb_sim_heightimg_to_sensor_pointcloud_node
    ${PCL_LIBRARIES}
		${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(tb_simulate_actuatorresponse_node src/actuatorsresponse.cpp)
target_link_libraries(tb_simulate_actuatorresponse_node ${catkin_LIBRARIES})

add_executable(tb_simulate_djiresponse_node src/djiresponse.cpp)
target_link_libraries(tb_simulate_djiresponse_node ${catkin_LIBRARIES})

add_executable(tb_simulate_octrepub_node src/octrepub.cpp)
target_link_libraries(tb_simulate_octrepub_node ${catkin_LIBRARIES})
