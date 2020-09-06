cmake_minimum_required(VERSION 2.8.3)
project(tb_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
