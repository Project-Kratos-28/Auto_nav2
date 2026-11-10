#!/bin/bash
# Host: terminal UI around start.sh. Edit the network/launch settings, diagnose the LiDAR, ZED,
# container and ROS network (and offer a fix for each problem), start the stack, then check that
# GLIM and nvblox are actually publishing, and open RViz on them.
#   ~/kratos_glim/start_tui.sh
# Settings are written where the stack reads them: LiDAR/Orin IPs -> MID360_config.json,
# ROS domain/peer -> ros_network.env, launch arguments -> .home/tui.env (passed to start.sh).
set -u
REPO=$(cd "$(dirname "$0")" && pwd)
NAME=kratos_glim
LIVOX="$REPO/src/livox_ros_driver2/config/MID360_config.json"
NETENV="$REPO/ros_network.env"
TUIENV="$REPO/.home/tui.env"
DEPTH=zed LIDAR_Z=0.60 GUI=auto CLOUDINI=false
[ -f "$TUIENV" ] && . "$TUIENV"

# ── colours (#1A1A1A background, #FFFF55 accent, #333333 rules, #CCCCCC text) ─────────────────
rgb() { printf '\e[38;2;%d;%d;%dm' "0x${1:0:2}" "0x${1:2:2}" "0x${1:4:2}"; }
BG=$'\e[48;2;26;26;26m' TX=$(rgb CCCCCC) DIM=$(rgb 888888) ACC=$(rgb FFFF55) WHT=$(rgb FFFFFF)
LN=$(rgb 333333) OK=$(rgb 7FD962) BAD=$(rgb FF5555) B=$'\e[1m' R=$'\e[22m'
printf '\e[?1049h%s%s\e[2J\e[H' "$BG" "$TX"
stty -echo 2>/dev/null   # keys typed while a check runs must not smear the screen
trap 'stty echo 2>/dev/null; printf "\e[0m\e[2J\e[?1049l"' EXIT
say()  { printf '%s\e[K\n' "$BG$TX$*"; }
rule() { say "$LN$(printf '─%.0s' $(seq 1 "$(( $(tput cols) < 90 ? $(tput cols) : 90 ))"))$TX"; }
head_() { printf '\e[2J\e[H'; say; say "  $ACC${B}KRATOS$R$TX  ${DIM}GLIM + nvblox bring-up$TX   $*"; rule; }
key()  { local k; while read -rsn1 -t 0.01 k; do :; done; read -rsn1 k; printf '%s' "$k"; }  # drop keys typed ahead
ask()  { local v; printf '%s' "$BG  $ACC$1$TX [$DIM$2$TX]: " >&2; stty echo; read -r v; stty -echo; printf '%s' "${v:-$2}"; }
pause() { say; say "  ${DIM}press any key$TX"; key >/dev/null; }

# ── settings ──────────────────────────────────────────────────────────────────────────────────
lidar_ip() { grep -oE '"ip" *: *"[0-9.]+"' "$LIVOX" | grep -oE '[0-9.]+"$' | tr -d '"'; }
host_ip()  { grep -oE '"cmd_data_ip" *: *"[0-9.]+"' "$LIVOX" | grep -oE '[0-9.]+"$' | tr -d '"'; }
set_lidar_ip() { sed -i -E "s/(\"ip\" *: *\")[0-9.]+/\1$1/" "$LIVOX"; }
set_host_ip()  { sed -i -E "s/(\"(cmd_data|push_msg|point_data|imu_data)_ip\" *: *\")[0-9.]+/\1$1/" "$LIVOX"; }
netvar() { sed -nE "s/^$1=//p" "$NETENV"; }
set_netvar() {  # in place; an empty value comments the line out (ROS_STATIC_PEERS)
    local l="$1=$2"; [ -n "$2" ] || l="# $1=192.168.1.20"
    grep -qE "^#? *$1=" "$NETENV" && sed -i -E "0,/^#? *$1=.*/s||$l|" "$NETENV" || echo "$l" >> "$NETENV"
}
save() { mkdir -p "$REPO/.home"; printf 'DEPTH=%s\nLIDAR_Z=%s\nGUI=%s\nCLOUDINI=%s\n' "$DEPTH" "$LIDAR_Z" "$GUI" "$CLOUDINI" > "$TUIENV"; }
launch_args() {
    local a="depth:=$DEPTH lidar_z:=$LIDAR_Z"
    [ "$GUI" != auto ] && a+=" gui:=$GUI"
    [ "$CLOUDINI" = true ] && a+=" cloudini:=true"
    echo "$a"
}
is_ip() { [[ $1 =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; }
local_display() { [[ ${DISPLAY:-} == :* ]]; }

# ── state ─────────────────────────────────────────────────────────────────────────────────────
container_up() { docker ps --format '{{.Names}}' | grep -qx "$NAME"; }
stack_up() { container_up && docker exec "$NAME" pgrep -f '[r]os2 launch kratos_bringup' >/dev/null; }
in_ws() { docker exec -u "$(id -u):$(id -g)" -w /workspaces/kratos_glim "$NAME" bash -lc "$*"; }
dot() { if "$@"; then printf '%s' "$OK●$TX"; else printf '%s' "$DIM○$TX"; fi; }

# ── diagnostics: every check counts; a failure offers a fix ───────────────────────────────────
N=0 P=0 W=0 F=0 T0=0
begin() { N=0 P=0 W=0 F=0 T0=$(date +%s%N); }
item() { N=$((N + 1)); printf '%s' "$BG  $DIM[$(printf '%2d' $N)]$TX $(printf '%-36s' "$1") "; }
pass() { P=$((P + 1)); say "$OK✔$TX ${DIM}$*$TX"; }
warn() { W=$((W + 1)); say "$ACC!$TX $*"; }
fail() { F=$((F + 1)); say "$BAD✘$TX $*"; }
tally() {
    rule
    say "  checks $WHT$N$TX   ${OK}pass $P$TX   ${ACC}warn $W$TX   ${BAD}fail $F$TX   ${DIM}$(( ($(date +%s%N) - T0) / 1000000 )) ms$TX"
}
choose() {  # choose "question" "k:label" ... -> prints the key pressed
    local o; say "      $ACC?$TX $1"
    for o in "${@:2}"; do say "        $ACC${o%%:*}$TX ${o#*:}"; done
    printf '%s' "$BG        > "; local k; k=$(key); say "$k"; printf '%s' "$k" >&3
}

diagnose() {
    head_ "${DIM}diagnostics$TX"; begin
    local lip hip iface cur k c

    item "docker + image"
    if ! docker info >/dev/null 2>&1; then fail "docker not reachable (is $USER in group docker?)"
    elif ! docker image inspect kratos/glim_nvblox:latest >/dev/null 2>&1; then fail "image missing: docker/build_image.sh"
    else pass "kratos/glim_nvblox:latest"; fi

    item "workspace built"
    # -L: symlink-install links point into the container's /workspaces path, dangling on the host.
    if [ -L "$REPO/install/kratos_bringup/share/kratos_bringup/launch/kratos.launch.py" ]; then pass install/
    else
        fail "install/ has no kratos_bringup"
        k=$(choose "build it now (several minutes)?" "b:build" "s:skip" 3>&1 >/dev/tty)
        [ "$k" = b ] && "$REPO/docker/run_container.sh" docker/build_ws.sh
    fi

    # Orin port that can reach the LiDAR (same /24) must carry the IP the LiDAR sends to.
    while :; do
        lip=$(lidar_ip) hip=$(host_ip)
        item "Orin LiDAR port ($hip)"
        iface=$(ip -4 -o addr | awk -v p="${lip%.*}." 'index($4, p) == 1 {print $2; exit}')
        cur=$(ip -4 -o addr show dev "${iface:-lo}" | awk -v p="${lip%.*}." 'index($4, p) == 1 {sub("/.*", "", $4); print $4; exit}')
        if [ -z "$iface" ]; then
            fail "no interface in ${lip%.*}.0/24"
            ip -br -4 addr | grep -v '^lo' | while read -r l; do say "        $DIM$l$TX"; done
            k=$(choose "the LiDAR port needs $hip/24" "a:add $hip/24 to an interface (sudo)" "s:skip" 3>&1 >/dev/tty)
            [ "$k" = a ] || break
            iface=$(ask "interface" "$(ip -br link | awk '$1 ~ /^(end|eth|enP)/ {print $1; exit}')")
            sudo ip addr add "$hip/24" dev "$iface" && sudo ip link set "$iface" up; continue
        elif ip -4 -o addr show dev "$iface" | grep -q " $hip/"; then pass "$iface has $hip"; break
        else
            fail "$iface is $cur, MID360_config.json sends to $hip"
            k=$(choose "fix" "c:set config to $cur (recommended)" "a:add $hip/24 to $iface (sudo, until reboot)" "s:skip" 3>&1 >/dev/tty)
            case $k in c) set_host_ip "$cur" ;; a) sudo ip addr add "$hip/24" dev "$iface" ;; *) break ;; esac
        fi
    done

    while :; do
        lip=$(lidar_ip)
        item "MID-360 ping ($lip)"
        if ping -c 2 -W 1 "$lip" >/dev/null 2>&1; then pass "answers"; break; fi
        fail "no answer"
        c=$(ip neigh show | awk -v p="${lip%.*}." 'index($1, p) == 1 && $1 != "'"$(host_ip)"'" && /REACHABLE|STALE|DELAY|PROBE/ {print $1}' | tr '\n' ' ')
        [ -n "$c" ] && say "        ${DIM}seen on the link: $WHT$c$TX"
        say "        ${DIM}MID-360 default IP is 192.168.1.1XX (XX = last 2 digits of its serial); check power + cable$TX"
        k=$(choose "fix" "e:enter the LiDAR IP" "r:retry" "s:skip" 3>&1 >/dev/tty)
        case $k in
            e) c=$(ask "LiDAR IP" "$lip"); is_ip "$c" && set_lidar_ip "$c" ;;
            r) ;;
            *) break ;;
        esac
    done

    if [ "$DEPTH" = none ]; then item "ZED 2i"; pass "skipped (depth:=none)"
    elif stack_up; then item "ZED 2i"; pass "in use by this stack"
    else
        while :; do
            item "ZED 2i on USB"
            if lsusb | grep -qi '2b03:f880'; then pass "2b03:f880"; break; fi
            fail "not found (USB 3 port, cable)"
            k=$(choose "fix" "r:retry (after re-plugging)" "n:run LiDAR only (depth:=none)" "s:skip" 3>&1 >/dev/tty)
            case $k in r) ;; n) DEPTH=none; save; break ;; *) break ;; esac
        done
        item "ZED free (no other container)"
        c=""
        for k in $(docker ps --format '{{.Names}}' | grep -vx "$NAME"); do
            docker top "$k" -eo pid,args 2>/dev/null | grep -qE 'zed_wrapper|zed_node|component_container.*zed' && c+="$k "
        done
        if [ -z "$c" ]; then pass "nobody else holds it"
        else
            fail "held by: $c"
            k=$(choose "only one process can open the ZED" "x:docker stop $c" "n:run LiDAR only (depth:=none)" "s:skip" 3>&1 >/dev/tty)
            case $k in x) docker stop $c >/dev/null && say "        stopped $c" ;; n) DEPTH=none; save ;; esac
        fi
        item "ZED HID bound"
        c=$("$REPO/tools/zed_hid_rebind.sh" 2>&1 | tail -1)
        if [[ $c == *sudo* ]]; then warn "$c"; else pass "${c:-no HID interface listed}"; fi
    fi

    item "ROS network"
    c=$(netvar ROS_STATIC_PEERS)
    if container_up && [ "$(docker exec "$NAME" printenv ROS_DOMAIN_ID)/$(docker exec "$NAME" printenv ROS_STATIC_PEERS)" \
        != "$(netvar ROS_DOMAIN_ID)/$c" ]; then
        warn "container runs an old ros_network.env"
        if ! stack_up; then
            k=$(choose "restart the container to apply it?" "y:restart" "s:skip" 3>&1 >/dev/tty)
            [ "$k" = y ] && docker stop "$NAME" >/dev/null && say "        stopped; start re-creates it"
        else say "        ${DIM}stop the stack first ([x]) to apply it$TX"; fi
    elif [ -n "$c" ] && ! ping -c 1 -W 1 "$c" >/dev/null 2>&1; then warn "laptop peer $c not answering"
    else pass "domain $(netvar ROS_DOMAIN_ID), ${c:-multicast discovery}"; fi

    item "lidar_z"
    if [ "$LIDAR_Z" = 0.60 ]; then pass "0.60 (matches nav2_params / pcd2pgm_live)"
    else warn "$LIDAR_Z: also update nav2_params.yaml and pcd2pgm_live.yaml heights"; fi

    tally
}

# ── after start: are GLIM and nvblox publishing? ──────────────────────────────────────────────
health() {
    head_ "${DIM}health$TX"
    stack_up || { say "  ${BAD}stack not running$TX"; pause; return; }
    local t topics="/livox/lidar /livox/imu /livox/lidar_filtered /glim_ros/odom /glim_ros/map /map \
/local_costmap/costmap"
    [ "$DEPTH" != none ] && topics+=" /zed/zed_node/rgb/color/rect/image /nvblox_node/static_esdf_pointcloud /nvblox_node/mesh"
    [ "$DEPTH" = ess ] && topics+=" /ess/depth"
    say "  ${DIM}topics appear as they answer, up to 15 s (GLIM's map needs the IMU init: keep still)$TX"
    begin
    # One ros2 CLI per topic, in parallel, in one container shell; each line printed as it finishes.
    local s
    while read -r s t; do
        item "$t"
        if [ "$s" = ok ]; then pass; else fail "no message"; fi
    done < <(in_ws "for t in $topics; do (timeout 15 ros2 topic echo --once --no-arr \$t >/dev/null 2>&1 \
        && echo ok \$t || echo no \$t) & done; wait" 2>/dev/null)
    tally
    if [ "$F" -gt 0 ]; then
        say "  ${DIM}last errors in the log:$TX"
        grep -aE '\[(ERROR|FATAL)\]|[Ee]xception' "$REPO/log/bringup/latest.log" 2>/dev/null | tail -8 | cut -c1-$(( $(tput cols) - 5 )) \
            | while read -r l; do say "    $BAD$l$TX"; done
    fi
    pause
}

rviz() {
    head_ "${DIM}rviz$TX"
    if local_display; then
        docker exec -d -u "$(id -u):$(id -g)" -e DISPLAY="$DISPLAY" -w /workspaces/kratos_glim "$NAME" \
            bash -lc 'exec rviz2 -d install/kratos_bringup/share/kratos_bringup/rviz/kratos.rviz' >/dev/null
        say "  RViz started on $DISPLAY (GLIM map, LiDAR, nvblox mesh + ESDF, costmaps)"
    else
        say "  ${DIM}no local display (SSH). On the laptop, same ROS_DOMAIN_ID=$(netvar ROS_DOMAIN_ID):$TX"
        say "  ${WHT}scp $USER@$(hostname -I | awk '{print $1}'):kratos_glim/src/kratos_bringup/rviz/kratos.rviz . && rviz2 -d kratos.rviz$TX"
        say "  ${DIM}(nvblox mesh needs nvblox_rviz_plugin on the laptop; laptop.rviz is the light view)$TX"
    fi
    pause
}

start() {
    diagnose
    if [ "$F" -gt 0 ]; then
        [ "$(choose "$BAD$F check(s) failed.$TX start anyway?" "y:start" "n:back to the menu" 3>&1 >/dev/tty)" = y ] || return
    else say "  ${OK}all clear$TX"; fi
    say; say "  ${DIM}./start.sh --no-follow $(launch_args)$TX"
    "$REPO/start.sh" --no-follow $(launch_args) 2>&1 | while read -r l; do say "  $l"; done
    stack_up || { pause; return; }
    for t in 10 9 8 7 6 5 4 3 2 1; do printf '\r%s\e[K' "$BG  ${ACC}keep the rover still $t s$TX (GLIM IMU init)"; sleep 1; done; say
    health
    local_display && [ "$GUI" != false ] || rviz
}

edit() {
    local v
    case $1 in
        1) v=$(ask "MID-360 IP" "$(lidar_ip)"); is_ip "$v" && set_lidar_ip "$v" ;;
        2) v=$(ask "Orin LiDAR-port IP" "$(host_ip)"); is_ip "$v" && set_host_ip "$v" ;;
        3) v=$(ask "ROS_DOMAIN_ID (same as laptop)" "$(netvar ROS_DOMAIN_ID)"); [[ $v =~ ^[0-9]+$ ]] && set_netvar ROS_DOMAIN_ID "$v" ;;
        4) v=$(ask "laptop IP for unicast discovery, '-' for none" "$(netvar ROS_STATIC_PEERS)")
           [ "$v" = - ] && set_netvar ROS_STATIC_PEERS "" || { is_ip "$v" && set_netvar ROS_STATIC_PEERS "$v"; } ;;
        5) case $DEPTH in zed) DEPTH=ess ;; ess) DEPTH=none ;; *) DEPTH=zed ;; esac ;;
        6) v=$(ask "lidar_z (m above ground)" "$LIDAR_Z"); [[ $v =~ ^[0-9.]+$ ]] && LIDAR_Z=$v ;;
        7) case $GUI in auto) GUI=true ;; true) GUI=false ;; *) GUI=auto ;; esac ;;
        8) [ "$CLOUDINI" = true ] && CLOUDINI=false || CLOUDINI=true ;;
    esac
    save
}

row() { say "   $ACC$1$TX  $(printf '%-16s' "$2") $WHT$(printf '%-15s' "${3:--}")$TX $DIM${4:-}$TX"; }
while :; do
    head_ "container $(dot container_up)  stack $(dot stack_up)"
    row 1 "MID-360 IP" "$(lidar_ip)"
    row 2 "Orin LiDAR IP" "$(host_ip)" "(the MID-360 sends here)"
    row 3 "ROS domain" "$(netvar ROS_DOMAIN_ID)"
    row 4 "laptop peer" "$(netvar ROS_STATIC_PEERS)" "(unicast discovery; empty = multicast)"
    row 5 "depth" "$DEPTH" "zed | ess | none"
    row 6 "lidar_z" "$LIDAR_Z"
    row 7 "gui" "$GUI" "auto = only with a local display"
    row 8 "cloudini" "$CLOUDINI" "compressed clouds for the radio"
    rule
    say "   ${ACC}s$TX start   ${ACC}d$TX diagnose   ${ACC}h$TX health   ${ACC}v$TX rviz   ${ACC}l$TX log   ${ACC}x$TX stop   ${ACC}q$TX quit"
    k=$(key)
    case $k in
        [1-8]) say; edit "$k" ;;
        s) if stack_up; then say "  already running"; pause; else start; fi ;;
        d) diagnose; pause ;;
        h) health ;;
        v) rviz ;;
        l) trap : INT; printf '\e[0m'; "$REPO/logs.sh"; trap - INT; printf '%s' "$BG$TX" ;;  # Ctrl+C ends the log only
        x) say; "$REPO/stop.sh" 2>&1 | while read -r l; do say "  $l"; done; pause ;;
        q) exit 0 ;;
    esac
done
