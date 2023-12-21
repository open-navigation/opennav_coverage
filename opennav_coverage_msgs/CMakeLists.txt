cmake_minimum_required(VERSION 3.5)
project(opennav_coverage_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Coordinate.msg"
  "msg/Coordinates.msg"
  "msg/HeadlandMode.msg"
  "msg/SwathMode.msg"
  "msg/RowSwathMode.msg"
  "msg/RouteMode.msg"
  "msg/PathMode.msg"
  "msg/Swath.msg"
  "msg/PathComponents.msg"
  "action/ComputeCoveragePath.action"
  "action/NavigateCompleteCoverage.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
