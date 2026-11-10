#!/usr/bin/env bash
# Build the host workspace separately from Docker's path-dependent symlink install.
set -eo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'ROS 2 Jazzy is required on the host.' >&2; exit 1; }
GLIM_OVERLAY=${KRATOS_GLIM_OVERLAY:-/opt/glim_ros_fix_ws/install/local_setup.bash}
[[ -f $GLIM_OVERLAY ]] || { echo "Patched GLIM overlay missing: $GLIM_OVERLAY" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
GLIM_CORE_OVERLAY=${KRATOS_GLIM_CORE_OVERLAY:-/opt/glim_core_fix_ws/install/local_setup.bash}
[[ ! -f $GLIM_CORE_OVERLAY ]] || source "$GLIM_CORE_OVERLAY"
source "$GLIM_OVERLAY"
cd "$REPO"
colcon --log-base log_bare build \
    --build-base build_bare --install-base install_bare --symlink-install \
    --packages-select livox_ros_driver2 lidar_angle_filter \
        waypoint_interfaces waypoint_manager glim_dump_export pcd2pgm \
        kratos_nav kratos_perception kratos_bringup \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy -DCMAKE_BUILD_TYPE=Release "$@"
echo 'Bare-metal workspace built. Start with ./start.sh --bare-metal.'
