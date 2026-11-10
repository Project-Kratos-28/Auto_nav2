#!/bin/bash
# Host: build kratos/glim_nvblox:latest on top of the kratos nvblox image
# (~/kratos_nvblox/docker/build_image.sh builds that one first).
set -e
cd "$(dirname "$0")/.."
docker image inspect kratos/isaac_ros_nvblox:latest >/dev/null 2>&1 \
    || { echo "base image kratos/isaac_ros_nvblox:latest not found: build ~/kratos_nvblox/docker/build_image.sh first"; exit 1; }
docker build -f docker/Dockerfile.glim -t kratos/glim_nvblox:latest "$@" .
echo "built kratos/glim_nvblox:latest"
