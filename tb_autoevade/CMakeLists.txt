cmake_minimum_required(VERSION 2.8.3)
project(tb_autoevade)


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
  diagnostic_updater
  octomap_ros
  octomap_msgs
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
add_executable(tb_autoevade_evasion_node src/evasion.cpp)
target_link_libraries(tb_autoevade_evasion_node
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_executable(tb_autoevade_obstaclefinder_node src/obstaclefinder.cpp)
target_link_libraries(tb_autoevade_obstaclefinder_node
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
add_executable(tb_autoevade_odomglobal_node src/odomglobal.cpp)
target_link_libraries(tb_autoevade_odomglobal_node
  ${catkin_LIBRARIES}
	${OpenCV_LIBRARIES})
