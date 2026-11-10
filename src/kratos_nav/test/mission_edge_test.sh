#!/bin/bash
# waypoint_mission.py must never leave the rover driving on its own:
#   A) Nav2 accepts a goal only after the 5 s send timeout -> mission cancels it.
#   B) Ctrl+C mid-goal -> mission cancels the active goal.
# Uses fake_glim.py (waypoint services + TF) and fake_nav_server.py (no real Nav2).
#   bash src/kratos_nav/test/mission_edge_test.sh   (from the repo; or the VM copy in ~/Kratos/kratos_nav_ws)
export ROS_DOMAIN_ID=45
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${EDGE_LOG_DIR:-/tmp/kratos_nav_edge}"
rm -rf "$LOG_DIR"; mkdir -p "$LOG_DIR"
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
# Workspaces that provide waypoint_interfaces and kratos_nav. Defaults are the
# VM layout; override with e.g. KRATOS_SETUP="/path/to/auto_fastlio2/install/setup.bash".
for f in ${KRATOS_SETUP:-$HOME/Kratos/glim_ext_ws/install/setup.bash $HOME/Kratos/kratos_nav_ws/install/setup.bash}; do
  source "$f"
done

setsid python3 "$DIR/fake_glim.py" > "$LOG_DIR/fake_glim.log" 2>&1 &
GLIM=$!
setsid ros2 run tf2_ros static_transform_publisher --z 0.6 --frame-id base_link --child-frame-id livox_frame > "$LOG_DIR/tf.log" 2>&1 &
TF=$!
FAIL=0

run_case() {  # name delay mission_timeout
  setsid python3 "$DIR/fake_nav_server.py" "$2" > "$LOG_DIR/$1_nav.log" 2>&1 &
  local NAV=$!
  sleep 2
  timeout -s INT "$3" ros2 run kratos_nav waypoint_mission.py --ros-args -p waypoints:="['wp1']" > "$LOG_DIR/$1_mission.log" 2>&1
  sleep 2
  kill -INT -$NAV 2>/dev/null; sleep 1; kill -KILL -$NAV 2>/dev/null
  echo "--- $1: fake nav";     cat "$LOG_DIR/$1_nav.log"
  echo "--- $1: mission";      grep -vE '^\s*$' "$LOG_DIR/$1_mission.log" | tail -8
}

# A) accepted 6 s after the request (mission gives up at 5 s)
run_case late 6 30
grep -q "did not answer the goal request" "$LOG_DIR/late_mission.log" || { echo "FAIL A: no timeout message"; FAIL=1; }
grep -q "GOAL_CANCELED" "$LOG_DIR/late_nav.log" || { echo "FAIL A: late goal was left running"; FAIL=1; }

# B) accepted at once, Ctrl+C after 6 s
run_case sigint 0 6
grep -q "cancelling current Nav2 goal" "$LOG_DIR/sigint_mission.log" || { echo "FAIL B: no cancel on Ctrl+C"; FAIL=1; }
grep -q "GOAL_CANCELED" "$LOG_DIR/sigint_nav.log" || { echo "FAIL B: goal kept running after Ctrl+C"; FAIL=1; }

kill -INT -$GLIM -$TF 2>/dev/null; sleep 1; kill -KILL -$GLIM -$TF 2>/dev/null
[ "$FAIL" = 0 ] && echo "=== MISSION EDGE TEST PASSED ===" || echo "=== MISSION EDGE TEST FAILED ==="
exit $FAIL
