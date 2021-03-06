cmake_minimum_required(VERSION 2.8.3)
project(tb_targeters)


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
  visualization_msgs
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
 LIBRARIES tb_abmap
)

add_executable(tb_targeters_side2d_node src/side2d.cpp)
target_link_libraries(tb_targeters_side2d_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_targeters_building_node src/test_standalone.cpp)
target_link_libraries(tb_targeters_building_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(tb_targeter2_node src/targeter.cpp)
target_link_libraries(tb_targeter2_node
    ${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
add_executable(tb_targeters2_node src/test.cpp)
target_link_libraries(tb_targeters2_node
    ${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})

add_executable(tb_vis_node src/vis.cpp)
target_link_libraries(tb_vis_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})

add_executable(tb_clusters_node src/clusters.cpp)
target_link_libraries(tb_clusters_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_clustersffill_node src/clustersffill.cpp)
target_link_libraries(tb_clustersffill_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_pathclusters_node src/pathclusters.cpp)
target_link_libraries(tb_pathclusters_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
