#!/bin/bash
# Host: stop the Kratos stack cleanly (SIGINT to the launch, so GLIM saves its session), file
# GLIM's session under ~/kratos_glim/maps/<date_time>, then stop the kratos_glim container.
# After this the ZED 2i and the GPU are free for other containers. Safe to run over SSH.
REPO=$(cd "$(dirname "$0")" && pwd)
NAME=kratos_glim
if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "$NAME is not running"
    exit 0
fi
# [r] keeps pgrep from matching its own command line.
PIDS=$(docker exec "$NAME" pgrep -f '[r]os2 launch kratos_bringup' || true)
if [ -n "$PIDS" ]; then
    echo "stopping the stack (GLIM writes its session; up to 60 s)..."
    docker exec "$NAME" kill -INT $PIDS
    for _ in $(seq 1 60); do
        docker exec "$NAME" pgrep -f '[g]lim_rosnode|[z]ed_wrapper|[n]vblox|[e]ss_stereo|[c]omponent_container' >/dev/null || break
        sleep 1
    done
fi
docker stop "$NAME" >/dev/null && echo "stopped $NAME (ZED and GPU released)"

# GLIM writes its session to /tmp/dump (the host's /tmp, shared with the container) on SIGINT.
STARTED=$(cat "$REPO/log/bringup/started_at" 2>/dev/null || echo 0)
if [ -d /tmp/dump ] && [ "$(stat -c %Y /tmp/dump)" -ge "$STARTED" ]; then
    mkdir -p "$REPO/maps"
    DEST="$REPO/maps/$(date +%Y%m%d_%H%M%S)"
    mv /tmp/dump "$DEST" && echo "GLIM session saved to $DEST"
fi

# The ZED SDK can leave the camera's HID interface detached from usbhid; reattach it so the
# next container (this stack or another) gets a /dev/hidraw node for the ZED.
"$REPO/tools/zed_hid_rebind.sh"
