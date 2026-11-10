#!/usr/bin/env bash
# Isaac Sim + rover bringup + Nav2 (this repo's kratos_nav: RotationShim + MPPI), all in one.
#
#   tools/isaac_nav2_sim.sh [up] [--glim] [--no-rviz]   start Isaac (scene loaded, Play pressed),
#                                                       bringup and Nav2, then check them
#   tools/isaac_nav2_sim.sh goal 5.0,1.8 -2.0,1.5       send waypoints (map frame; see send_waypoints)
#   tools/isaac_nav2_sim.sh goal --relative 2,0         2 m ahead of the rover
#   tools/isaac_nav2_sim.sh status                      check /clock, Nav2 and the controller
#   tools/isaac_nav2_sim.sh logs                        follow Nav2's log
#   tools/isaac_nav2_sim.sh stop                        stop Nav2, bringup and Isaac
#
#   --glim      GLIM SLAM localization instead of Isaac's ground-truth odometry (not tested with MPPI)
#   --no-rviz   no RViz (default: RViz when there is a display)
#
# Needs the Isaac Sim build, ~/Kratos/Auto_Isaac_Sim (scene, athena_nav_sim), ~/athena_ws and
# ~/kratos_gui_ws built (see Auto_Isaac_Sim/docs/SETUP_FROM_SCRATCH.md). kratos_nav is built from
# this repo into ~/kratos_nav_mppi (symlinked, so YAML edits apply on the next start) and sourced
# after ~/kratos_gui_ws, whose kratos_nav is an older RPP copy.
#
# Overrides (environment): ISAAC_DIR, SIM_REPO, MPPI_WS, SIM_LOGS.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_DIR="${ISAAC_DIR:-$HOME/IsaacSim/_build/linux-aarch64/release}"
SIM_REPO="${SIM_REPO:-$HOME/Kratos/Auto_Isaac_Sim}"
MPPI_WS="${MPPI_WS:-$HOME/kratos_nav_mppi}"
LOGS="${SIM_LOGS:-$MPPI_WS/sim_logs}"
GLIM_OVERLAY="$HOME/glim_ros_fix_ws/install/local_setup.bash"
export ATHENA_SCENE="$SIM_REPO/description/athena_description/urdf/athena_rover-6_3/athena_rover-6.usda"

say()  { printf '\033[1;36m[sim]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[sim] %s\033[0m\n' "$*"; }
fail() { printf '\033[1;31m[sim] %s\033[0m\n' "$*"; exit 1; }

# Options (any position); what is left is the command and its arguments.
LOCALIZATION=ground_truth
RVIZ=$([[ "${DISPLAY:-}" == :* ]] && echo true || echo false)
args=()
for a in "$@"; do
    case "$a" in
        --glim)    LOCALIZATION=glim ;;
        --no-rviz) RVIZ=false ;;
        *)         args+=("$a") ;;
    esac
done
set -- "${args[@]+"${args[@]}"}"
CMD="${1:-up}"; shift || true

ros_env() {
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$HOME/athena_ws/install/setup.bash"
    source "$HOME/kratos_gui_ws/install/setup.bash"
    source "$MPPI_WS/install/setup.bash"
    [[ $LOCALIZATION == glim ]] && source "$GLIM_OVERLAY"   # patched glim_ros, must be last
    source "$SIM_REPO/scripts/ros_env.sh"
    set -u
}

# Each part runs in its own process group (setsid), so stop can end all of it.
start_group() {  # name, command...
    local name=$1; shift
    setsid "$@" > "$LOGS/$name.log" 2>&1 < /dev/null &
    echo $! > "$LOGS/$name.pid"
}
group_alive() { kill -0 -- "-$1" 2>/dev/null; }
stop_group() {  # name, seconds per signal
    local name=$1 secs=$2 pgid sig i
    [[ -f "$LOGS/$name.pid" ]] || return 0
    pgid=$(cat "$LOGS/$name.pid")
    for sig in INT TERM KILL; do
        group_alive "$pgid" || break
        say "  $name: SIG$sig"
        kill -"$sig" -- "-$pgid" 2>/dev/null
        for ((i = 0; i < secs * 2; i++)); do group_alive "$pgid" || break; sleep 0.5; done
    done
    group_alive "$pgid" && warn "  $name (group $pgid) still running"
    rm -f "$LOGS/$name.pid"
}
wait_for() {  # seconds, description, test command...
    local secs=$1 what=$2 i; shift 2
    for ((i = 0; i < secs; i++)); do "$@" && return 0; sleep 1; done
    fail "timed out after ${secs}s waiting for $what (logs: $LOGS)"
}

# Processes of this stack, also ones started by hand or by dgx.sh. The [x] brackets stop the
# pattern from matching this script's own command line.
STACK='[k]it/kit .*isaacsim\.exp\.full|[r]os2 launch (athena_controller|athena_nav_sim)|/opt/ros/jazzy/lib/[n]av2_[a-z_]+/|/athena_ws/install/[a]thena_(controller|nav_sim)/|/opt/ros/jazzy/lib/[g]lim_ros/'
stack_running() { pgrep -f "$STACK" > /dev/null; }

build_mppi() {
    [[ -f "$MPPI_WS/install/kratos_nav/share/kratos_nav/config/nav2_params_sim.yaml" ]] && return 0
    say "building kratos_nav from $REPO into $MPPI_WS (first time only)"
    (set +u; source /opt/ros/jazzy/setup.bash; set -u
     colcon --log-base "$MPPI_WS/log" build --build-base "$MPPI_WS/build" --install-base "$MPPI_WS/install" \
         --symlink-install --base-paths "$REPO/src/kratos_nav" --packages-select kratos_nav) \
        || fail "kratos_nav build failed"
}

clock_ticking() { timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1; }

check() {
    local ok=0 ctrl
    if clock_ticking; then say "  ok      /clock ticking (sim playing)"; else say "  FAIL    /clock not ticking"; ok=1; fi
    if ros2 action list 2>/dev/null | grep -qx /navigate_through_poses; then
        say "  ok      Nav2 actions up"
    else
        say "  FAIL    no /navigate_through_poses action"; ok=1
    fi
    ctrl=$(timeout 10 ros2 param get /controller_server FollowPath.primary_controller 2>/dev/null)
    if [[ $ctrl == *MPPIController* ]]; then
        say "  ok      controller: RotationShim + MPPI"
    else
        say "  FAIL    controller: ${ctrl:-not found}"; ok=1
    fi
    return $ok
}

up() {
    [[ -x "$ISAAC_DIR/isaac-sim.sh" ]] || fail "Isaac Sim not found at $ISAAC_DIR (set ISAAC_DIR)"
    [[ -f "$ATHENA_SCENE" ]] || fail "scene missing: $ATHENA_SCENE (set SIM_REPO)"
    [[ -n "${DISPLAY:-}" ]] || fail "no DISPLAY: Isaac Sim opens a window (run from the desktop)"
    [[ $LOCALIZATION != glim || -f $GLIM_OVERLAY ]] || fail "--glim needs the patched glim_ros overlay: $GLIM_OVERLAY"
    if stack_running; then
        warn "parts of the sim stack are already running (an Isaac window, bringup or Nav2):"
        pgrep -af "$STACK" | cut -c1-140
        fail "save your Isaac work, then run: $0 stop"
    fi
    mkdir -p "$LOGS"
    build_mppi

    say "starting Isaac Sim (log: $LOGS/isaac.log)"
    # Isaac selects its own bundled ROS 2 libraries: give it only the network settings.
    (set +u; source "$SIM_REPO/scripts/ros_env.sh"; set -u
     cd "$ISAAC_DIR" && start_group isaac ./isaac-sim.sh --exec "$SIM_REPO/scripts/isaac_autoplay.py")
    ros_env
    ros2 daemon stop > /dev/null 2>&1
    wait_for 300 "Isaac to load the scene and press Play" \
        grep -q -E "\[autoplay\] timeline playing|\[autoplay\] open_stage .*ok=False" "$LOGS/isaac.log"
    grep -q "ok=False" "$LOGS/isaac.log" && fail "Isaac could not open $ATHENA_SCENE"
    wait_for 60 "/clock" clock_ticking

    say "starting bringup (log: $LOGS/bringup.log)"
    start_group bringup ros2 launch athena_controller bringup.launch.py rviz:=false
    sleep 6

    say "starting Nav2, localization:=$LOCALIZATION (log: $LOGS/nav.log)"
    start_group nav ros2 launch athena_nav_sim nav_sim.launch.py \
        localization:="$LOCALIZATION" use_bridge:=false rviz:="$RVIZ" glim_viewer:=false
    wait_for 180 "Nav2 to become active" grep -q "Managed nodes are active" "$LOGS/nav.log"
    [[ $LOCALIZATION == glim ]] && say "GLIM: keep the rover still until it logs 'initial IMU state estimation result' ($0 logs)"

    say "checking"
    if check; then say "all up"; else warn "some checks failed, see $LOGS"; fi
    cat <<EOF

  Send goals:   $0 goal 5.0,1.8            (map frame)
                $0 goal -2.0,1.5 0,0
                $0 goal --relative 2,0     (rover frame)
                or 2D Goal Pose in RViz
  Nav2 log:     $0 logs
  Stop:         $0 stop
EOF
}

down() {
    say "stopping Nav2, bringup and Isaac"
    stop_group nav 10
    stop_group bringup 10
    stop_group isaac 30
    # Leftovers: parts started by hand, or children that left their group.
    if stack_running; then
        say "  stopping leftover processes"
        local sig i
        for sig in INT TERM KILL; do
            stack_running || break
            pkill -"$sig" -f "$STACK" 2>/dev/null
            for ((i = 0; i < 20; i++)); do stack_running || break; sleep 0.5; done
        done
    fi
    ros2 daemon stop > /dev/null 2>&1
    command -v fastdds > /dev/null && fastdds shm clean > /dev/null 2>&1
    if stack_running; then warn "still running:"; pgrep -af "$STACK" | cut -c1-140; return 1; fi
    say "stopped"
}

case "$CMD" in
    up)     up ;;
    goal)   ros_env; [[ $# -gt 0 ]] || fail "usage: $0 goal x,y [x,y ...]  (or --relative x,y)"
            exec ros2 run athena_nav_sim send_waypoints "$@" ;;
    status) ros_env; check ;;
    logs)   tail -n 200 -F "$LOGS/nav.log" | grep --line-buffered -vE "TF_OLD_DATA|Possible reasons are listed" ;;
    stop|down) ros_env; down ;;
    *)      sed -n '2,20p' "$0"; exit 2 ;;
esac
