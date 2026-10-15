#!/bin/bash
# Laptop test bench: SITL + MAVProxy (to QGC) + fake tracking camera + a test video, so QGC sees a "camera".
# Then start QGroundControl yourself (it auto-connects on UDP 14550) and follow README_QGC_TEST.md.
# Usage:  ./run_qgc_camera_test.sh            (Ctrl+C stops everything this script started)
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
HERE="$(cd "$(dirname "$0")" && pwd)"
WORK="$(mktemp -d /tmp/qgc_cam_test.XXXXXX)"; cd "$WORK"
pids=()
cleanup() { echo; echo "stopping..."; for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT INT TERM

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" -w --model + --speedup 1 \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm" --sim-address=127.0.0.1 -I0 > sitl.log 2>&1 &
pids+=($!); sleep 4

# MAVProxy = the "ground link": connects to SITL (this also opens SITL's companion port 5762) and feeds QGC on 14550
mavproxy.py --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --daemon > mavproxy.log 2>&1 &
pids+=($!); sleep 3

# Test pattern as UDP H.264 on port 5600  (QGC: Application Settings > Video > UDP h.264, port 5600)
gst-launch-1.0 -q videotestsrc is-live=true pattern=ball ! video/x-raw,width=1280,height=720,framerate=30/1 \
    ! x264enc tune=zerolatency bitrate=2000 key-int-max=30 ! rtph264pay config-interval=1 pt=96 \
    ! udpsink host=127.0.0.1 port=5600 > video.log 2>&1 &
pids+=($!)

echo "SITL + MAVProxy + test video running (logs in $WORK)."
echo "Starting the fake camera component (it prints every tracking command it receives)..."
echo "NOW start QGroundControl and follow README_QGC_TEST.md"
python3 -u "$HERE/fake_camera_component.py" --connect tcp:127.0.0.1:5762 --video-uri udp://127.0.0.1:5600
