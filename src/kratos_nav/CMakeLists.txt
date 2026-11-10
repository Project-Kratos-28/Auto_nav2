cmake_minimum_required(VERSION 3.8)
project(kratos_nav)

find_package(ament_cmake REQUIRED)

# Config: install the folders so launch files can find them under
# share/kratos_nav/ after `colcon build`.
install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME})

# Python nodes, installed so `ros2 run kratos_nav <script>.py` finds them.
install(PROGRAMS scripts/waypoint_mission.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
