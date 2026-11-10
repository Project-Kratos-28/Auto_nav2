cmake_minimum_required(VERSION 3.8)
project(waypoint_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddWaypoint.srv"
  "srv/GetWaypoint.srv"
  "srv/SaveWaypoints.srv"
  "srv/ListWaypoints.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
