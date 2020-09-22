cmake_minimum_required(VERSION 2.8.3)
project(tb_autonomy_orig)


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
 LIBRARIES tb_div
 )
 add_executable(tb_pathclustering_client2_node src/pathclustering_with_client2.cpp)
 target_link_libraries(tb_pathclustering_client2_node
		${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
 add_executable(tb_pathclustering_client_node src/pathclustering_with_client.cpp)
 target_link_libraries(tb_pathclustering_client_node
		 ${catkin_LIBRARIES}
		 ${OpenCV_LIBRARIES})
 add_executable(tb_test_abmap_pathclusteringffill_node src/standalone_pathclustering_ffill.cpp)
 target_link_libraries(tb_test_abmap_pathclusteringffill_node
     ${catkin_LIBRARIES}
     ${OpenCV_LIBRARIES})
add_executable(tb_test_abmap_requester2_node src/test_requester.cpp)
target_link_libraries(tb_test_abmap_requester2_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_test_abmap_requester_node src/abmap_requester.cpp)
target_link_libraries(tb_test_abmap_requester_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_test_abmap_pathclustering_node src/abmap_pathclustering.cpp)
target_link_libraries(tb_test_abmap_pathclustering_node
    ${catkin_LIBRARIES}
		${OpenCV_LIBRARIES})
add_executable(tb_standalone_pathclustering_node src/standalone_pathclustering.cpp)
target_link_libraries(tb_standalone_pathclustering_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_test_abmap_downside_node src/abmap_downside.cpp)
target_link_libraries(tb_test_abmap_downside_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
