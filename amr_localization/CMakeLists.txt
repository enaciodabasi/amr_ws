cmake_minimum_required(VERSION 3.5)
project(amr_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(amr_interfaces REQUIRED)

set(ODOMETRY_NODE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  tf2_ros
  nav_msgs
  geometry_msgs
  amr_interfaces
)

add_executable(odometry_node src/odometry_calculator_node.cpp)
ament_target_dependencies(odometry_node ${ODOMETRY_NODE_INCLUDE_DEPENDS})

install(
  TARGETS
  odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  include
  src
  config
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()
