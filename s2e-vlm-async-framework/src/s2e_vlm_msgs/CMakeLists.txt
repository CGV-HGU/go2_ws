cmake_minimum_required(VERSION 3.8)
project(s2e_vlm_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedPose.msg"
  "msg/Trajectory2D.msg"
  "msg/NodeStatus.msg"
  "msg/SystemHealth.msg"
  "action/Rotate.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
