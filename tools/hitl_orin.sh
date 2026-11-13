#!/usr/bin/env bash
# Orin side of hardware-in-the-loop testing (docs/HITL.md). The autonomy nodes run here, in the
# kratos_glim container, on the LiDAR, depth camera, odometry and /clock of Isaac Sim on the DGX
# Spark (started there with tools/isaac_nav2_sim.sh up --orin=<this Orin's IP>). Nav2's /cmd_vel
# goes back over the switch to the sim's swerve controller.
#
#   tools/hitl_orin.sh up --sim=ADDR [--glim] [--no-zed]   start (waits up to 10 min for the sim)
#   tools/hitl_orin.sh status                              check /clock, Nav2 and the controller
#   tools/hitl_orin.sh logs                                follow Nav2's log
#   tools/hitl_orin.sh stop                                stop it (the sim keeps running)
#   tools/hitl_orin.sh update                              pull the sim repo copy, rebuild athena_nav_sim
#
#   --sim=ADDR  the DGX's IP on the switch (remembered for the other commands)
#   --glim      GLIM SLAM (the sim's GLIM config) + pcd2pgm /map instead of Isaac's ground-truth pose
#   --no-zed    LiDAR only. Default: the depth camera too (terrain mapper + nvblox), if the sim
#               publishes it (it does unless the DGX side was started with --no-zed)
#
# What runs is athena_nav_sim's nav_sim.launch.py, the bringup tools/isaac_nav2_sim.sh runs on the
# DGX without --orin: this repo's kratos_nav (nav2_params_sim.yaml, 7 Nav2 nodes, RotationShim +
# MPPI) and kratos_traversability, plus nvblox_sim.launch.py. Not kratos.launch.py: the sim rover
# has its own frames, mounts and GLIM config, and there is no Livox driver or ZED SDK in the loop.
#
# The sim repo is cloned (sparse: athena_nav_sim, athena_zed_nav/config, glim) into .hitl/, which
# git ignores, and athena_nav_sim is built into .hitl/ws. The rover workspace (install/) is only
# built if kratos_nav or kratos_traversability is missing from it.
# Fast DDS is limited to the switch interface (.hitl/fastdds.xml): otherwise the DGX also sends the
# point clouds to this Orin's Tailscale and Docker addresses. HITL_DDS_XML=0 turns that off.
#
# Overrides (environment): SIM_GIT, SIM_BRANCH (default zed2i_testing), HITL_DDS_XML.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME=kratos_glim
W=/workspaces/kratos_glim                 # $REPO inside the container (docker/run_container.sh)
HITL="$REPO/.hitl"
SIM="$HITL/Auto_Isaac_Sim"
LOGS="$HITL/logs"
SIM_GIT="${SIM_GIT:-git@github.com:Project-Kratos-28/Auto_Isaac_Sim.git}"
SIM_BRANCH="${SIM_BRANCH:-zed2i_testing}"
NVBLOX_SLICE_MIN=-0.15                    # same as tools/isaac_nav2_sim.sh

say()  { printf '\033[1;36m[hitl]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[hitl] %s\033[0m\n' "$*"; }
fail() { printf '\033[1;31m[hitl] %s\033[0m\n' "$*"; exit 1; }

LOCALIZATION=ground_truth
ZED=true
SIM_IP=""
args=()
for a in "$@"; do
    case "$a" in
        --sim=*)  SIM_IP=${a#--sim=} ;;
        --sim)    fail "--sim needs an address: --sim=192.168.1.50" ;;
        --glim)   LOCALIZATION=glim ;;
        --no-zed) ZED=false ;;
        *)        args+=("$a") ;;
    esac
done
set -- "${args[@]+"${args[@]}"}"
CMD="${1:-up}"
[[ -z $SIM_IP && -f $HITL/sim_ip ]] && SIM_IP=$(cat "$HITL/sim_ip")

# Shell prefix for ROS commands in the container: the sim's network settings (domain 0 and Fast
# DDS, as Auto_Isaac_Sim/scripts/ros_env.sh), the DGX as a static peer, athena_nav_sim.
ctr_env() {
    local xml=""
    [[ ${HITL_DDS_XML:-1} != 0 ]] && xml="export FASTDDS_DEFAULT_PROFILES_FILE=$W/.hitl/fastdds.xml;"
    printf 'export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET; '
    printf 'export ROS_STATIC_PEERS="%s${ROS_STATIC_PEERS:+;$ROS_STATIC_PEERS}"; %s ' "$SIM_IP" "$xml"
    printf 'source %s/.hitl/ws/install/local_setup.bash; ' "$W"
}
in_ctr() {     # command string, run in the container (login shell: ROS, workspace, GLIM overlay)
    docker exec -u "$(id -u):$(id -g)" -w "$W" "$NAME" bash -lc "$(ctr_env) $*"
}
in_ctr_bg() {  # log name, command string: detached, output to .hitl/logs/<name>.log
    local name=$1; shift
    docker exec -d -u "$(id -u):$(id -g)" -w "$W" "$NAME" \
        bash -lc "$(ctr_env) exec $* > $W/.hitl/logs/$name.log 2>&1"
}

container_up() { docker ps --format '{{.Names}}' | grep -qx "$NAME"; }
in_container() { container_up && docker exec "$NAME" pgrep -f "$1" > /dev/null; }  # pattern
hitl_running() { in_container '[r]os2 launch athena_nav_sim'; }

ensure_sim_repo() {
    mkdir -p "$HITL" "$LOGS"
    [[ -f $HITL/.gitignore ]] || echo '*' > "$HITL/.gitignore"
    [[ -d $SIM/.git ]] && return 0
    say "cloning $SIM_GIT ($SIM_BRANCH) into $SIM (sparse, first time only)"
    git clone --depth 1 --filter=blob:none --sparse --branch "$SIM_BRANCH" "$SIM_GIT" "$SIM" \
        && git -C "$SIM" sparse-checkout set src/athena_nav_sim src/athena_zed_nav/config glim \
        || { rm -rf "$SIM"; fail "clone failed (GitHub SSH key on this Orin? SIM_GIT / SIM_BRANCH)"; }
}

build_sim() {
    say "building athena_nav_sim into .hitl/ws"
    docker exec -u "$(id -u):$(id -g)" -w "$W" "$NAME" bash -lc \
        "colcon --log-base .hitl/ws/log build --build-base .hitl/ws/build --install-base .hitl/ws/install \
             --symlink-install --base-paths .hitl/Auto_Isaac_Sim/src/athena_nav_sim --packages-select athena_nav_sim" \
        > "$LOGS/build.log" 2>&1 || fail "athena_nav_sim build failed: $LOGS/build.log"
}

ensure_built() {
    if [[ ! -f $REPO/install/kratos_nav/share/kratos_nav/config/nav2_params_sim.yaml ||
          ! -d $REPO/install/kratos_traversability ]]; then
        say "building the rover workspace (kratos_nav / kratos_traversability missing from install/)"
        "$REPO/docker/run_container.sh" docker/build_ws.sh || fail "docker/build_ws.sh failed"
    fi
    [[ -d $HITL/ws/install/athena_nav_sim ]] || build_sim
}

# Fast DDS on the switch interface only (address $1), plus shared memory inside this machine.
# Socket buffers ask for 4 MB; the kernel caps them at net.core.rmem_max / wmem_max.
write_dds_xml() {
    cat > "$HITL/fastdds.xml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- Written by tools/hitl_orin.sh: Fast DDS on the HITL switch interface ($1) only. -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>hitl_shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>hitl_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>$1</address></interfaceWhiteList>
      <sendBufferSize>4194304</sendBufferSize>
      <receiveBufferSize>4194304</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="hitl_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>hitl_shm</transport_id>
        <transport_id>hitl_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
}

clock_ticking() { in_ctr 'timeout 5 ros2 topic echo --once /clock' > /dev/null 2>&1; }

check() {
    local ok=0 ctrl
    if clock_ticking; then say "  ok      /clock from the sim"; else say "  FAIL    no /clock from the sim"; ok=1; fi
    if in_ctr 'ros2 action list' 2>/dev/null | grep -qx /navigate_through_poses; then
        say "  ok      Nav2 actions up"
    else
        say "  FAIL    no /navigate_through_poses action"; ok=1
    fi
    ctrl=$(in_ctr 'timeout 10 ros2 param get /controller_server FollowPath.primary_controller' 2>/dev/null)
    if [[ $ctrl == *MPPIController* ]]; then
        say "  ok      controller: RotationShim + MPPI"
    else
        say "  FAIL    controller: ${ctrl:-not found}"; ok=1
    fi
    return $ok
}

up() {
    [[ -n $SIM_IP ]] || fail "usage: $0 up --sim=<the DGX's IP on the switch> [--glim] [--no-zed]"
    ping -c 1 -W 2 "$SIM_IP" > /dev/null 2>&1 || fail "the DGX ($SIM_IP) does not answer ping (cable, switch, IPs on one subnet?)"
    local route orin_ip iface
    route=$(ip -4 route get "$SIM_IP")
    orin_ip=$(sed -n 's/.* src \([0-9.]*\).*/\1/p' <<< "$route")
    iface=$(sed -n 's/.* dev \([^ ]*\).*/\1/p' <<< "$route")
    [[ $iface == tailscale* ]] && fail "the route to $SIM_IP goes over Tailscale ($iface), not the switch: use the DGX's switch IP"
    say "DGX $SIM_IP via $iface, this Orin is $orin_ip"
    (( $(sysctl -n net.core.rmem_max) >= 4194304 )) \
        || warn "small socket buffers drop point clouds: sudo sysctl -w net.core.rmem_max=4194304 net.core.wmem_max=4194304"
    if in_container '[r]os2 launch kratos_bringup'; then fail "the rover stack is running in $NAME: ./stop.sh first"; fi
    if hitl_running; then fail "already running: $0 status, $0 logs, or $0 stop"; fi

    ensure_sim_repo
    echo "$SIM_IP" > "$HITL/sim_ip"
    "$REPO/docker/run_container.sh" true || fail "could not start the $NAME container"
    ensure_built
    if [[ $LOCALIZATION == glim ]]; then   # glim_map_odom (athena_nav_sim) needs SciPy
        docker exec "$NAME" python3 -c 'import scipy' 2> /dev/null \
            || fail "--glim needs python3-scipy in $NAME: rebuild the image (docker/build_image.sh), or docker exec -u root $NAME bash -c 'apt-get update && apt-get install -y python3-scipy'"
    fi
    write_dds_xml "$orin_ip"
    in_ctr 'ros2 daemon stop' > /dev/null 2>&1

    say "waiting up to 10 min for the sim's /clock (on the DGX: tools/isaac_nav2_sim.sh up --orin=$orin_ip)"
    local i
    for ((i = 0; i < 600; i += 5)); do clock_ticking && break; sleep 5; done
    clock_ticking || fail "no /clock from $SIM_IP (sim not playing? ROS_DOMAIN_ID? try HITL_DDS_XML=0)"
    say "sim clock ticking"

    local camera_cloud=none nvblox=false
    if [[ $ZED == true ]]; then
        for ((i = 0; i < 10; i++)); do
            in_ctr 'ros2 topic list' 2>/dev/null | grep -qx /sim_camera/depth/image && { camera_cloud=/sim_camera/depth/points; break; }
            sleep 1
        done
        if [[ $camera_cloud == none ]]; then
            warn "the sim publishes no /sim_camera/depth/image (DGX side started with --no-zed?): LiDAR only"
        elif in_ctr 'ros2 pkg prefix nvblox_ros && ros2 pkg prefix nvblox_nav2' > /dev/null 2>&1; then
            nvblox=true
        else
            warn "nvblox_ros / nvblox_nav2 missing in $NAME: depth camera for terrain only, no nvblox layer"
        fi
    fi

    if [[ $nvblox == true ]]; then
        say "starting nvblox (log: $LOGS/nvblox.log)"
        in_ctr_bg nvblox ros2 launch athena_nav_sim nvblox_sim.launch.py \
            sim_config:=$W/.hitl/Auto_Isaac_Sim/src/athena_zed_nav/config/nvblox_sim.yaml slice_min:=$NVBLOX_SLICE_MIN
    fi
    say "starting Nav2, localization:=$LOCALIZATION, camera: $camera_cloud (log: $LOGS/nav.log)"
    in_ctr_bg nav ros2 launch athena_nav_sim nav_sim.launch.py \
        localization:=$LOCALIZATION use_bridge:=false rviz:=false glim_viewer:=false \
        camera_cloud:=$camera_cloud nvblox:=$nvblox glim_config:=$W/.hitl/Auto_Isaac_Sim/glim/glim_config
    for ((i = 0; i < 180; i++)); do
        grep -q "Managed nodes are active" "$LOGS/nav.log" 2>/dev/null && break
        (( i > 10 )) && ! hitl_running && fail "the launch exited: $LOGS/nav.log"
        sleep 1
    done
    grep -q "Managed nodes are active" "$LOGS/nav.log" || fail "Nav2 not active after 180 s: $LOGS/nav.log"
    [[ $LOCALIZATION == glim ]] && say "GLIM: leave the sim rover still until 'initial IMU state estimation result' ($0 logs)"

    say "checking"
    if check; then say "all up"; else warn "some checks failed, see $LOGS"; fi
    cat <<EOF

  Goals and RViz: on the DGX (tools/isaac_nav2_sim.sh goal gates, or 2D Goal Pose in RViz)
  Nav2 log:       $0 logs
  Stop:           $0 stop   (also after Stop/Play in Isaac: sim time restarts at 0, then up again)
EOF
}

stop() {
    container_up || { say "$NAME is not running"; return 0; }
    say "stopping"
    local sig i
    for sig in INT TERM KILL; do
        hitl_running || break
        docker exec "$NAME" pkill -"$sig" -f 'ros2 launch athena_nav_sim'
        for ((i = 0; i < 30; i++)); do hitl_running || break; sleep 0.5; done
    done
    # Nodes that outlived their launch: Nav2 (sim params copy), GLIM, nvblox, the sim's helpers.
    local left='[a]thena_nav2_|/\.hitl/ws/install/|__node:=(nvblox_container|traversability_mapper|lifecycle_manager_navigation|glim_ros|pcd2pgm|map_to_odom|lidar_to_imu|base_link_to_[a-z_]+)'
    for sig in INT TERM KILL; do
        in_container "$left" || break
        docker exec "$NAME" pkill -"$sig" -f "$left"
        for ((i = 0; i < 10; i++)); do in_container "$left" || break; sleep 0.5; done
    done
    in_ctr 'ros2 daemon stop' > /dev/null 2>&1
    if in_container "$left"; then warn "still running:"; docker exec "$NAME" pgrep -af "$left" | cut -c1-140; return 1; fi
    say "stopped (the sim on the DGX keeps running)"
}

case "$CMD" in
    up)     up ;;
    status) container_up || fail "$NAME is not running"; [[ -n $SIM_IP ]] || fail "never started: $0 up --sim=ADDR"; check ;;
    logs)   tail -n 200 -F "$LOGS/nav.log" | grep --line-buffered -vE "TF_OLD_DATA|Possible reasons are listed" ;;
    stop|down) stop ;;
    update) [[ -d $SIM/.git ]] || fail "no sim repo yet: $0 up --sim=ADDR clones it"
            git -C "$SIM" pull --ff-only || fail "git pull failed in $SIM"
            "$REPO/docker/run_container.sh" true && build_sim && say "updated; restart with $0 stop && $0 up" ;;
    *)      sed -n '2,30p' "$0"; exit 2 ;;
esac
