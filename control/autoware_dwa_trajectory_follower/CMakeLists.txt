cmake_minimum_required(VERSION 3.14)
project(autoware_dwa_trajectory_follower)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(dwa_trajectory_follower_node
  src/dwa_trajectory_follower/main.cpp
  src/dwa_trajectory_follower/node_construction.cpp
  src/dwa_trajectory_follower/callbacks_and_utils.cpp
  src/dwa_trajectory_follower/dwa_solver.cpp
  src/dwa_trajectory_follower/timer.cpp
)

install(PROGRAMS
  scripts/override_controller_with_dwa.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  config
)
