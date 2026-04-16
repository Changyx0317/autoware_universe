cmake_minimum_required(VERSION 3.14)
project(autoware_short_goal_pivot_coordinator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(short_goal_pivot_coordinator_node
  src/short_goal_pivot_coordinator_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
