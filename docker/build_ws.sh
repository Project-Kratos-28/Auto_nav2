#!/bin/bash
# In the kratos_glim container: build this workspace (Jazzy). Same package set as the Humble
# README section 1.6 plus kratos_perception and kratos_bringup.
set -e
cd /workspaces/kratos_glim
source /opt/ros/jazzy/setup.bash
source /opt/glim_ros_fix_ws/install/local_setup.bash   # waypoint_manager links glim_ros: build against the patched one
colcon build --symlink-install \
  --packages-select livox_ros_driver2 lidar_angle_filter \
    waypoint_interfaces waypoint_manager glim_dump_export pcd2pgm kratos_nav kratos_perception kratos_bringup \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release "$@"
echo "built; new shells source install/setup.bash automatically (then the GLIM fix overlay)"
