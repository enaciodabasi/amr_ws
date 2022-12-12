cmake_minimum_required(VERSION 3.5)
project(amr_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

nav2_package()

set(MANUAL_CONTROLLER_NODE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(manual_controller_node src/amr_manual_controller.cpp)
ament_target_dependencies(manual_controller_node ${MANUAL_CONTROLLER_NODE_INCLUDE_DEPENDS})

add_executable(dummy_tree src/bt_deneme.cpp)
ament_target_dependencies(dummy_tree ament_index_cpp)
target_link_libraries(dummy_tree BT::behaviortree_cpp_v3)

install(
  TARGETS
  manual_controller_node
  dummy_tree
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  include
  src
  params
  launch
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()
