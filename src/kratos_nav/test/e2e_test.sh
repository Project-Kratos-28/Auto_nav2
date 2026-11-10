#!/bin/bash
# End-to-end smoke test: fake GLIM (+pcd2pgm+rover stand-in) + real Nav2 +
# real waypoint_mission.py, driving to wp1 then wp2 around a wall obstacle.
#
# Run from anywhere on the VM:
#   bash src/kratos_nav/test/e2e_test.sh   (from the repo; or the VM copy in ~/Kratos/kratos_nav_ws)
# Logs (not this script) go to /tmp - the VM's /tmp is wiped on reboot, this
# script and fake_glim.py are not.
set -o pipefail
export ROS_DOMAIN_ID=43

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${E2E_LOG_DIR:-/tmp/kratos_nav_e2e}"
rm -rf "$LOG_DIR"; mkdir -p "$LOG_DIR"

chmod +x "$DIR/../scripts/waypoint_mission.py"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
# Workspaces that provide waypoint_interfaces and kratos_nav. Defaults are the
# VM layout; override with e.g. KRATOS_SETUP="/path/to/auto_fastlio2/install/setup.bash".
for f in ${KRATOS_SETUP:-$HOME/Kratos/glim_ext_ws/install/setup.bash $HOME/Kratos/kratos_nav_ws/install/setup.bash}; do
  source "$f"
done

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "executables: $(ros2 pkg executables kratos_nav)"

setsid python3 "$DIR/fake_glim.py" > "$LOG_DIR/fake.log" 2>&1 &
FAKE=$!
setsid ros2 launch kratos_nav nav.launch.py > "$LOG_DIR/nav.log" 2>&1 &
NAV=$!

NAV2_OK=0
for i in $(seq 1 60); do
  if grep -q "Managed nodes are active" "$LOG_DIR/nav.log" 2>/dev/null; then
    echo "nav2 active after ${i}s"; NAV2_OK=1; break
  fi
  if grep -q "Aborting bringup" "$LOG_DIR/nav.log" 2>/dev/null; then
    echo "NAV2 ABORTED"; break
  fi
  sleep 1
done

MISSION_RC=1
if [ "$NAV2_OK" = "1" ]; then
  # MISSION_ARGS: extra waypoint_mission.py arguments, e.g. MISSION_ARGS="-p mode:=through"
  timeout -s INT 240 ros2 run kratos_nav waypoint_mission.py ${MISSION_ARGS:+--ros-args $MISSION_ARGS} > "$LOG_DIR/mission.log" 2>&1
  MISSION_RC=$?
  echo "mission exit code: $MISSION_RC"
fi

kill -INT -$FAKE -$NAV 2>/dev/null
sleep 6
kill -KILL -$FAKE -$NAV 2>/dev/null

echo "=== mission.log (tail) ==="
grep -vE "^\s*$" "$LOG_DIR/mission.log" 2>/dev/null | tail -30
echo "=== fake.log (tail) ==="
tail -20 "$LOG_DIR/fake.log" 2>/dev/null
echo "=== nav errors before shutdown ==="
awk '/process has died|signal_handler/{exit} {print}' "$LOG_DIR/nav.log" 2>/dev/null \
  | grep -E "\[ERROR\]|\[FATAL\]" | grep -v "livox" | sort | uniq -c | head -20

# --- Assertions ---
FAIL=0

if [ "$MISSION_RC" != "0" ]; then
  echo "FAIL: mission exit code $MISSION_RC (expected 0)"; FAIL=1
fi

if ! grep -q "reached 'wp1'" "$LOG_DIR/mission.log" 2>/dev/null; then
  echo "FAIL: wp1 was never reached"; FAIL=1
fi
if ! grep -q "reached 'wp2'" "$LOG_DIR/mission.log" 2>/dev/null; then
  echo "FAIL: wp2 was never reached"; FAIL=1
fi

WP1_LINE=$(grep -n "reached 'wp1'" "$LOG_DIR/mission.log" 2>/dev/null | head -1 | cut -d: -f1)
WP2_LINE=$(grep -n "reached 'wp2'" "$LOG_DIR/mission.log" 2>/dev/null | head -1 | cut -d: -f1)
if [ -n "${WP1_LINE:-}" ] && [ -n "${WP2_LINE:-}" ] && [ "$WP1_LINE" -gt "$WP2_LINE" ]; then
  echo "FAIL: wp2 was reached before wp1 (out of order)"; FAIL=1
fi

if grep -q "COLLISION:" "$LOG_DIR/fake.log" 2>/dev/null; then
  echo "FAIL: robot entered an occupied (wall) cell"; FAIL=1
  grep "COLLISION:" "$LOG_DIR/fake.log"
fi

if [ "$FAIL" = "0" ]; then
  echo "=== E2E TEST PASSED ==="
  exit 0
else
  echo "=== E2E TEST FAILED ==="
  exit 1
fi
