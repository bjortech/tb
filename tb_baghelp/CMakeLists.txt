cmake_minimum_required(VERSION 2.8.3)
project(tb_baghelp)


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
#add_executable(tb_target_node src/target.cpp)
#target_link_libraries(tb_target_node
#    ${catkin_LIBRARIES}
#		${OpenCV_LIBRARIES})
#add_executable(tb_targeting_node src/targeting.cpp)
#target_link_libraries(tb_targeting_node
#    ${catkin_LIBRARIES}
#		${OpenCV_LIBRARIES})

add_executable(tb_baghelp_node src/baghelp.cpp)
target_link_libraries(tb_baghelp_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES})
