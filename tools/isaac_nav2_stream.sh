#!/usr/bin/env bash
# The latest sim stack (tools/isaac_nav2_sim.sh: IRC arena, ZED depth + nvblox, traversability,
# RotationShim + MPPI) with Isaac headless and streamed over WebRTC, like
# Auto_Isaac_Sim/scripts/dgx.sh. Works over SSH (no DISPLAY needed).
#
#   tools/isaac_nav2_stream.sh                     start everything, stream on this machine's Tailscale IP
#   tools/isaac_nav2_stream.sh --ip 10.8.1.157     stream on another address (e.g. the LAN)
#   tools/isaac_nav2_stream.sh goal gates          every other command/option of isaac_nav2_sim.sh
#   tools/isaac_nav2_stream.sh stop                works here too (--flat, --no-zed, --glim, logs, ...)
#
# Then on a laptop: Isaac Sim WebRTC Streaming Client -> server <IP> (TCP 49100, UDP 47998; one
# viewer at a time), and ~/client.sh for RViz (Auto_Isaac_Sim/scripts/client.sh).
# Overrides (environment): STREAM_IP, plus everything isaac_nav2_sim.sh takes.
set -uo pipefail

ip_addr="${STREAM_IP:-}"
args=()
while (($#)); do
    case "$1" in
        --ip)   [[ -n "${2:-}" ]] || { echo "--ip needs an address" >&2; exit 2; }
                ip_addr=$2; shift 2 ;;
        --ip=*) ip_addr=${1#--ip=}; shift ;;
        *)      args+=("$1"); shift ;;
    esac
done

exec "$(dirname "${BASH_SOURCE[0]}")/isaac_nav2_sim.sh" "--stream${ip_addr:+=$ip_addr}" "${args[@]+"${args[@]}"}"
