cmake_minimum_required(VERSION 3.8)
project(cover_wyh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(rect_coverage_planner cover.cpp)
target_compile_features(rect_coverage_planner PUBLIC cxx_std_17)
ament_target_dependencies(
  rect_coverage_planner
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  visualization_msgs
)

install(
  TARGETS rect_coverage_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
