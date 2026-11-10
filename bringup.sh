#!/usr/bin/env bash
# Bare-metal entry point for the same Jazzy launch used by Docker.
set -eo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if [[ ${1:-} == --help || ${1:-} == -h ]]; then
    cat <<'HELP'
Usage: ./bringup.sh [ROS launch arguments]
       ./start.sh --bare-metal [ROS launch arguments]

Runs the Kratos stack on the host in the foreground. Ctrl+C stops it and lets
GLIM save its session to /tmp/dump. Build first with ./build_bare.sh.
Examples: ./bringup.sh depth:=none
          ./bringup.sh depth:=ess cloudini:=true
HELP
    exit 0
fi

[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'ROS 2 Jazzy is required on the host.' >&2; exit 1; }
BARE_INSTALL=${KRATOS_BARE_INSTALL:-$REPO/install_bare}
[[ -f $BARE_INSTALL/setup.bash ]] || {
    echo "Bare-metal workspace missing: $BARE_INSTALL/setup.bash. Run ./build_bare.sh." >&2
    exit 1
}
GLIM_OVERLAY=${KRATOS_GLIM_OVERLAY:-/opt/glim_ros_fix_ws/install/local_setup.bash}
[[ -f $GLIM_OVERLAY ]] || {
    echo "Patched GLIM overlay missing: $GLIM_OVERLAY (set KRATOS_GLIM_OVERLAY)." >&2
    exit 1
}

source /opt/ros/jazzy/setup.bash
source "$BARE_INSTALL/setup.bash"
GLIM_CORE_OVERLAY=${KRATOS_GLIM_CORE_OVERLAY:-/opt/glim_core_fix_ws/install/local_setup.bash}
if [[ -f $GLIM_CORE_OVERLAY ]]; then
    source "$GLIM_CORE_OVERLAY"
else
    echo "WARNING: patched GLIM core overlay missing: $GLIM_CORE_OVERLAY (sensor gaps can abort GLIM)." >&2
fi
source "$GLIM_OVERLAY"
set -a
source "$REPO/ros_network.env"
set +a
export KRATOS_WORKSPACE="$REPO"
export KRATOS_ESS_ROOT="${KRATOS_ESS_ROOT:-$(dirname "$REPO")/kratos_nvblox/ess}"

for package in kratos_bringup glim_ros livox_ros_driver2; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || {
        echo "ROS package $package is unavailable on the host." >&2
        exit 1
    }
done
GLIM_PREFIX=$(ros2 pkg prefix glim_ros)
case "$GLIM_PREFIX" in
    "$(dirname "$GLIM_OVERLAY")"|"$(dirname "$GLIM_OVERLAY")"/*) ;;
    *) echo "glim_ros resolves to $GLIM_PREFIX, not the patched overlay $GLIM_OVERLAY." >&2; exit 1 ;;
esac
if [[ " $* " == *' cloudini:=true '* ]]; then
    ros2 pkg prefix cloudini_ros >/dev/null 2>&1 || {
        echo 'cloudini_ros is required for cloudini:=true on the host.' >&2
        exit 1
    }
fi
if [[ " $* " != *' depth:=none '* ]]; then
    for package in zed_wrapper nvblox_ros nvblox_nav2 nvblox_examples_bringup; do
        ros2 pkg prefix "$package" >/dev/null 2>&1 || {
            echo "ROS package $package is required for camera depth on the host." >&2
            exit 1
        }
    done
fi
if [[ " $* " == *' depth:=ess '* && ! -f $KRATOS_ESS_ROOT/.venv/bin/python ]]; then
    echo "ESS runtime missing: $KRATOS_ESS_ROOT/.venv/bin/python" >&2
    exit 1
fi

if command -v docker >/dev/null 2>&1 && docker ps --format '{{.Names}}' 2>/dev/null | grep -qx kratos_glim; then
    if docker exec kratos_glim pgrep -f '[r]os2 launch kratos_bringup' >/dev/null 2>&1; then
        echo 'The Docker stack is running. Stop it with ./stop.sh before starting bare metal.' >&2
        exit 1
    fi
fi

mkdir -p "$REPO/.home"
exec 9>"$REPO/.home/bare.lock"
flock -n 9 || { echo 'Another bare-metal stack is running.' >&2; exit 1; }
mkdir -p "$REPO/log/bringup"
export ROS_LOG_DIR="$REPO/log/bringup/bare_$(date +%Y%m%d_%H%M%S)"
echo "Starting bare metal ROS 2 Jazzy stack. ROS logs: $ROS_LOG_DIR"
echo 'Keep the rover still for the first ~10 seconds while GLIM initializes.'
exec ros2 launch kratos_bringup kratos.launch.py "$@"
