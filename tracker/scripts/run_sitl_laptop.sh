#!/bin/bash
# LAPTOP: ArduCopter SITL + MAVProxy forwarding MAVLink to the Jetson.
# Usage: JETSON_IP=192.168.0.117 scripts/run_sitl_laptop.sh
#        SIM=gazebo JETSON_IP=... scripts/run_sitl_laptop.sh   # with Gazebo (start run_gazebo_laptop.sh first)
# Starts SITL without its own MAVProxy, then one MAVProxy that feeds QGC
# (127.0.0.1:14550) and the Jetson (UDP 14560).
JETSON_IP="${JETSON_IP:-192.168.0.117}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
cd "$ARDUPILOT_DIR"
SIM_ARGS=""
[ "${SIM:-}" = "gazebo" ] && SIM_ARGS="-f gazebo-iris --model JSON"
./Tools/autotest/sim_vehicle.py -v ArduCopter $SIM_ARGS --no-mavproxy &
SITL_PID=$!
trap 'kill $SITL_PID 2>/dev/null' EXIT
sleep 15
mavproxy.py --master=tcp:127.0.0.1:5760 \
            --out=127.0.0.1:14550 \
            --out=udp:"$JETSON_IP":14560 \
            --console
