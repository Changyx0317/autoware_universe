cmake_minimum_required(VERSION 3.14)
project(autoware_diff_drive_control_adapter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(diff_drive_control_adapter_node
  src/diff_drive_control_adapter_node.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
