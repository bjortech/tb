cmake_minimum_required(VERSION 2.8.3)
project(tb_edto)


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
add_executable(tb_write_heightimg_node src/write_heightimg_octmap.cpp)
target_link_libraries(tb_write_heightimg_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_edto_obstacledistance_node src/obstacle.cpp)
target_link_libraries(tb_edto_obstacledistance_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_edto_down_node src/down.cpp)
target_link_libraries(tb_edto_down_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
add_executable(tb_edto_side_node src/side.cpp)
target_link_libraries(tb_edto_side_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
