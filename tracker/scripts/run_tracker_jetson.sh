#!/bin/bash
# Run target_tracker on a companion computer (Jetson Nano or Raspberry Pi 5) attached to the Cube Orange's
# TELEM2 UART. Default camera is the SkyDroid C12's own RTSP feed (--profile real-c12); CSI/USB are
# alternate camera sources for a different rig (e.g. bench-testing on a Jetson's own camera, not the C12).
#
#   scripts/run_tracker_jetson.sh                       # real-c12 profile: C12 RTSP + TELEM2 UART
#   PROFILE=real-c12-gimbal scripts/run_tracker_jetson.sh  # follow the C12's own gimbal tracking instead of image-mode CSRT
#   CAMERA=csi scripts/run_tracker_jetson.sh             # Jetson CSI camera instead of the C12 (no --profile)
#   CAMERA=usb USB_SOURCE=2 scripts/run_tracker_jetson.sh
#   CAMERA=gst VIDEO_PORT=5600 scripts/run_tracker_jetson.sh
#   MAV_SERIAL=/dev/ttyUSB0 scripts/run_tracker_jetson.sh   # override the TELEM2 device (see below)
#
# MAV_SERIAL: /dev/serial0 is a RASPBERRY PI-ONLY symlink (set up by raspi-config) -- it does not exist on
# Jetson by default, found live (a real Jetson run failed with "cannot open serial port /dev/serial0: No
# such file or directory" even with the Cube correctly wired). Defaulting here to /dev/ttyTHS1, the common
# Jetson Nano 40-pin-header UART1 device -- but VERIFY this for your actual board before trusting it: run
# `ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM*` on the Jetson itself and check `dmesg` for which one the Cube
# actually enumerates as. On a Raspberry Pi 5, override with MAV_SERIAL=/dev/serial0 (or /dev/ttyAMA0).
MAV_SERIAL="${MAV_SERIAL:-/dev/ttyTHS1}"
cd "$(dirname "$0")/.."
CAMERA="${CAMERA:-}"
if [ -z "$CAMERA" ]; then
    exec ./build/target_tracker --profile "${PROFILE:-real-c12}" --connection "serial:${MAV_SERIAL}:921600" "$@"
fi
case "$CAMERA" in
    csi) CAM_ARGS="--camera-source csi --csi-sensor-id ${CSI_SENSOR:-0}" ;;
    gst) CAM_ARGS="--camera-source gst --gst-udp-port ${VIDEO_PORT:-5600}" ;;
    usb) CAM_ARGS="--camera-source usb --source ${USB_SOURCE:-2}" ;;
    *)   echo "CAMERA must be csi, usb or gst (or unset, for the default --profile real-c12)"; exit 1 ;;
esac
exec ./build/target_tracker \
    --connection "serial:${MAV_SERIAL}:921600" \
    $CAM_ARGS "$@"
