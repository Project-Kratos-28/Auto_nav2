#!/bin/bash
# Runner for the pcd2pgm live-mode test suite. Not wired into CMake/colcon -
# run directly. See README.md for what each scenario checks.
#
# Usage: ./run_all.sh          (PCD2PGM_SETUP=<install/setup.bash> to pick the build)
export ROS_DOMAIN_ID=44

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG="$SCRIPT_DIR/../config/pcd2pgm_live.yaml"
# Workspace that has pcd2pgm built. Default is the VM layout; override with
# PCD2PGM_SETUP=/path/to/auto_fastlio2/install/setup.bash
SETUP="${PCD2PGM_SETUP:-$HOME/Kratos/pcd2pgm_live_ws/install/setup.bash}"

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source "$SETUP"

LOGDIR="$(mktemp -d /tmp/pcd2pgm_livetest.XXXXXX)"
echo "Logs: $LOGDIR"

cleanup() {
  export ROS_DOMAIN_ID=44
  pkill -f pcd2pgm_node 2>/dev/null
  pkill -f pcd2pgm_test_driver 2>/dev/null
  pkill -f fake_glim_g2 2>/dev/null
}
trap cleanup EXIT

# Make sure nothing stale from a previous run is still around.
cleanup
sleep 1

OVERALL=0

echo "############################################################"
echo "# Part 1/2: test_main.py (scenarios A-F, H) against one node"
echo "############################################################"
NODE_LOG="$LOGDIR/node_main.log"
ros2 run pcd2pgm pcd2pgm_node --ros-args --params-file "$CONFIG" > "$NODE_LOG" 2>&1 &
NODE_PID=$!
sleep 2
if ! kill -0 "$NODE_PID" 2>/dev/null; then
  echo "FATAL: pcd2pgm_node failed to start - see $NODE_LOG"
  cat "$NODE_LOG"
  exit 1
fi

python3 "$SCRIPT_DIR/test_main.py" "$NODE_LOG"
MAIN_RC=$?
OVERALL=$((OVERALL || MAIN_RC))

kill "$NODE_PID" 2>/dev/null
wait "$NODE_PID" 2>/dev/null
sleep 1

echo
echo "############################################################"
echo "# Part 2/2: test_late_join_qos.py (scenario G, manages own node)"
echo "############################################################"
python3 "$SCRIPT_DIR/test_late_join_qos.py" "$CONFIG" "$LOGDIR"
G_RC=$?
OVERALL=$((OVERALL || G_RC))

echo
echo "############################################################"
if [ "$MAIN_RC" -eq 0 ] && [ "$G_RC" -eq 0 ]; then
  echo "# RESULT: ALL SCENARIOS PASSED  (logs: $LOGDIR)"
else
  echo "# RESULT: FAILURES PRESENT  (main_rc=$MAIN_RC late_join_rc=$G_RC, logs: $LOGDIR)"
fi
echo "############################################################"

exit $((MAIN_RC != 0 || G_RC != 0))
