#!/bin/bash
# Stop ONLY what simulation/sim_up.sh started (process groups recorded in $RUN_DIR/pids). Never kills by name.
RUN_DIR="${RUN_DIR:-$HOME/sim_run}"
[ -s "$RUN_DIR/pids" ] || { echo "nothing recorded in $RUN_DIR/pids"; exit 0; }
tac "$RUN_DIR/pids" | while read -r name pid; do
  if kill -0 "$pid" 2>/dev/null; then
    echo "stopping $name (pgid $pid)"; kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
  fi
done
sleep 3
tac "$RUN_DIR/pids" | while read -r name pid; do
  if kill -0 "$pid" 2>/dev/null; then echo "forcing $name"; kill -KILL -- "-$pid" 2>/dev/null; fi
done
: > "$RUN_DIR/pids"; rm -f "$RUN_DIR/pids"
echo "stopped. (Logs are kept in $RUN_DIR.)"
