#!/bin/bash
# Fault injection for the simulation: freeze a process group started by sim_up.sh for N seconds (SIGSTOP/SIGCONT).
#   simulation/fault.sh link-loss 6      # the companion's MAVLink gate frozen (follower on tcp:127.0.0.1:5770): heartbeats stop
#   simulation/fault.sh video-freeze 5   # the video relay frozen: no new frames reach the companion
# Only ever signals the process groups recorded in $RUN_DIR/pids (never by name).
RUN_DIR="${RUN_DIR:-$HOME/sim_run}"
kind="${1:-}"; secs="${2:-5}"
case "$kind" in
  link-loss)    name=mavgate;;
  video-freeze) name=relay;;
  *) echo "usage: $0 link-loss|video-freeze [seconds]"; exit 2;;
esac
pid=$(awk -v n="$name" '$1==n {print $2}' "$RUN_DIR/pids" 2>/dev/null | tail -1)
[ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || { echo "$name is not running (see $RUN_DIR/pids)"; exit 1; }
echo "[fault] $kind: freezing $name for ${secs}s"
kill -STOP -- "-$pid"
trap 'kill -CONT -- "-$pid" 2>/dev/null' EXIT
sleep "$secs"
kill -CONT -- "-$pid"; trap - EXIT
echo "[fault] $kind: $name resumed"
