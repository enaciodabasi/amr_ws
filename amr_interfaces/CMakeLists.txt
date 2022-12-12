cmake_minimum_required(VERSION 3.5)
project(amr_interfaces)


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/EncoderData.msg"
  "msg/BtPositionMsg.msg"
)

set(srv_files
  "srv/BtPositionSrv.srv"
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs control_msgs
)


ament_package()
