cmake_minimum_required(VERSION 2.8.3)
project(tb_edtoauto)


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
  dynamic_reconfigure
  diagnostic_updater
  octomap_ros
  octomap_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
	tb_msgsrv
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
#INCLUDE_DIRS include
 LIBRARIES tb_edto
)

add_executable(tb_edto_poly_auto_node src/poly_auto.cpp)
target_link_libraries(tb_edto_poly_auto_node
    ${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
	add_executable(tb_edto_side_auto_node src/side_auto.cpp)
	target_link_libraries(tb_edto_side_auto_node
	    ${catkin_LIBRARIES}
			${OpenCV_LIBRARIES})
	add_executable(tb_edto_down_auto_node src/down_auto.cpp)
	target_link_libraries(tb_edto_down_auto_node
	    ${catkin_LIBRARIES}
			${OpenCV_LIBRARIES})
