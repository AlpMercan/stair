cmake_minimum_required(VERSION 3.0.2)
project(stair)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  pluginlib
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stair_layer
  CATKIN_DEPENDS roscpp costmap_2d dynamic_reconfigure pluginlib nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(stair_layer
  src/stair_layer.cpp
)
target_link_libraries(stair_layer
  ${catkin_LIBRARIES}
)
