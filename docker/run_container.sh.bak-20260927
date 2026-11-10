#!/bin/bash
# Host: start (or attach to) the kratos_glim container. Everything of the rover stack runs in it:
# Livox driver, angle filter, GLIM, pcd2pgm, Nav2 (+ nvblox_layer), ZED 2i, ESS, nvblox.
#   docker/run_container.sh                 # start if needed, then open a shell
#   docker/run_container.sh <command...>    # run one command in it
# Mounts: this repo at /workspaces/kratos_glim, ~/kratos_nvblox at /workspaces/kratos_nvblox
# (ESS node, TensorRT engines and its Python venv). Host network: the MID-360 is reached over
# the Orin's Ethernet port directly, and the laptop over the Ubiquiti link.
# ROS networking (domain ID, discovery range, static peers) comes from ../ros_network.env; the
# default makes the rover's nodes visible on the local network so the laptop can command them.
# Commands run as the host user with its video/render groups: the Jetson GPU devices
# (/dev/nvmap, /dev/nvhost-*) are group video, and without it CUDA, TensorRT and GLIM's
# OpenGL viewer fail ("NvRmMemInitNvmap failed: Permission denied").
# The ZED 2i needs the host udev rule ~/kratos_nvblox/docker/99-slabs.rules, and a container
# copies /dev permissions when it starts: restart it after re-plugging the camera.
# The host's /tmp is shared (X11 socket, and GLIM's Ctrl+C dump in /tmp/dump; start.sh files it
# under maps/).
set -e
NAME=kratos_glim
REPO=$(cd "$(dirname "$0")/.." && pwd)
if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    [ -n "${DISPLAY:-}" ] && { xhost +local: >/dev/null 2>&1 || true; }
    mkdir -p "$REPO/.home"
    # ZED SDK data shared with ~/kratos_nvblox: optimized NEURAL models (minutes to rebuild per
    # container otherwise) and the camera calibration.
    ZED_DATA="${KRATOS_ZED_DATA:-$HOME/kratos_nvblox/zed}"
    mkdir -p "$ZED_DATA/resources" "$ZED_DATA/settings"
    docker run -d --rm --name "$NAME" \
        --privileged --network host --ipc host --runtime nvidia --gpus all \
        --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)" \
        ${DISPLAY:+-e DISPLAY="$DISPLAY"} -e NVIDIA_DRIVER_CAPABILITIES=all \
        --env-file "$REPO/ros_network.env" \
        -e HOME=/workspaces/kratos_glim/.home \
        -v /tmp:/tmp -v /dev/bus/usb:/dev/bus/usb -v /dev/input:/dev/input \
        -v /usr/bin/tegrastats:/usr/bin/tegrastats \
        -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
        -v "$REPO":/workspaces/kratos_glim \
        -v "$HOME/kratos_nvblox":/workspaces/kratos_nvblox \
        -v "$ZED_DATA/resources":/usr/local/zed/resources \
        -v "$ZED_DATA/settings":/usr/local/zed/settings \
        --entrypoint /bin/bash kratos/glim_nvblox:latest -c 'sleep infinity' >/dev/null
    echo "started $NAME"
fi
if [ $# -gt 0 ]; then
    exec docker exec -u "$(id -u):$(id -g)" -w /workspaces/kratos_glim "$NAME" bash -lc "$*"
fi
exec docker exec -it -u "$(id -u):$(id -g)" -w /workspaces/kratos_glim "$NAME" bash -l
