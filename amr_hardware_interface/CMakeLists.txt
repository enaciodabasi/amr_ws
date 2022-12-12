cmake_minimum_required(VERSION 3.5)
project(amr_hardware_interface)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(VELOCITY_PUBLISHER_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(velocity_publisher src/amr_velocity_publisher.cpp)
ament_target_dependencies(velocity_publisher ${VELOCITY_PUBLISHER_INCLUDE_DEPENDS})

install(
  TARGETS
  velocity_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  include
  src
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()
