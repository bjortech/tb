cmake_minimum_required(VERSION 2.8.3)
project(tb_costmap_orig)


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
  octomap_ros
  octomap_msgs
  cv_bridge
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
 LIBRARIES tb_sep
)
add_executable(tb_costmap_elevation_node src/costmap_elevation.cpp)
target_link_libraries(tb_costmap_elevation_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
add_executable(tb_costmap_updater_heightimg_node src/costmap_updater_heightimg.cpp)
target_link_libraries(tb_costmap_updater_heightimg_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
add_executable(tb_costmap_updater_edto2_node src/costmap_updater_edto2.cpp)
target_link_libraries(tb_costmap_updater_edto2_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
