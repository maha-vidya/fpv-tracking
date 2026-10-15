#!/bin/bash
# LAPTOP: start Gazebo Harmonic with the ArduPilot iris (gimbal camera) world.
# Requires ardupilot_gazebo built at $ARDUPILOT_GAZEBO_DIR (default ~/ardupilot_gazebo).
# Env vars are set for this process only; ~/.bashrc is not modified.
# Start this BEFORE run_sitl_laptop.sh (SITL waits for the Gazebo plugin).
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
WORLD="${WORLD:-iris_runway.sdf}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:$GZ_SIM_RESOURCE_PATH"
exec gz sim -v3 -r "$WORLD"
