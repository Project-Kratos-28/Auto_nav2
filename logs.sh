#!/bin/bash
# Host: show the running stack's log (the latest bring-up). Ctrl+C stops showing it; the stack
# keeps running. ./stop.sh stops the stack.
#   ~/kratos_glim/logs.sh            # follow the whole log
#   ~/kratos_glim/logs.sh glim       # only lines matching a pattern (grep -E), e.g. 'glim|ess_stereo'
REPO=$(cd "$(dirname "$0")" && pwd)
LOG="$REPO/log/bringup/latest.log"
[ -e "$LOG" ] || { echo "no bring-up log yet (./start.sh)"; exit 1; }
echo "following $(readlink -f "$LOG")  (Ctrl+C stops following; the stack keeps running)"
if [ -n "${1:-}" ]; then
    exec tail -n 200 -F "$LOG" | grep --line-buffered -E "$1"
fi
exec tail -n 200 -F "$LOG"
