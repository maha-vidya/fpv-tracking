#!/bin/bash
# LAPTOP: bring the whole Gazebo + SITL simulation up with ONE command, headless-friendly.
#
#   simulation/sim_up.sh                      # Gazebo, SITL, MAVProxy -> Jetson, camera relay -> Jetson, static target
#   simulation/sim_up.sh --no-gui             # EXPERIMENTAL server only (no window); flaky on this laptop, default is the windowed mode
#   simulation/sim_up.sh --target moving      # the red box (and an orange lookalike) move side to side
#   simulation/sim_up.sh --gimbal             # ArduPilot drives the camera gimbal (MNT1), so the tracker can tilt it (--gimbal-centre)
#   simulation/sim_up.sh --fast               # fast GO dashes: ArduPilot speed/descent/acceleration limits raised (simulation/fast.parm)
#   simulation/sim_up.sh --c12-camera         # a C12-like 1280x720 ~100 deg camera instead of 640x480 115 deg (people ~2.6x bigger)
#   simulation/sim_up.sh --fpv-camera         # FPV drone: the same 1280x720 camera FIXED to the airframe (no gimbal), tilted
#                                             # FPV_CAM_PITCH degrees (default -15, + up), and FPV speed (simulation/fpv.parm:
#                                             # 30 m/s, 60 deg lean) -- target_tracker --profile sim-fpv
#   simulation/sim_up.sh --video-to 127.0.0.1:5601 --mav-to 127.0.0.1:14561   # local-only dry run (no Jetson)
#
# Starts, each in its own process group, logging to $RUN_DIR (default ~/sim_run):
#   1 gazebo   2 sitl   3 mavproxy (daemon: QGC on 127.0.0.1:14550 + the companion UDP)
#   5 video relay (RTP/H.264 from Gazebo's camera plugin on 127.0.0.1:5600 -> the companion)   6 target
# Stop it with simulation/sim_down.sh, which kills ONLY the process groups recorded here (never by name).
#
# Documentation: ardupilot_gazebo GstCameraPlugin (RTP H.264 to udp_host:udp_port, enable via the
# camera's .../enable_streaming topic); ArduPilot SITL (sim_vehicle.py, -f gazebo-iris --model JSON,
# SITL TCP ports 5760/5762/5763); MAVProxy (--master, --out, --daemon).
set -u
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
JETSON_IP="${JETSON_IP:-192.168.0.117}"
VIDEO_TO="${VIDEO_TO:-$JETSON_IP:5600}"
MAV_TO="${MAV_TO:-$JETSON_IP:14560}"
RUN_DIR="${RUN_DIR:-$HOME/sim_run}"
WORLD="${WORLD:-iris_runway.sdf}"
TARGET="static"
GUI=1
GIMBAL_PARAMS=""
FAST_PARAMS=""
C12_CAMERA=0
FPV_CAMERA=0
FPV_CAM_PITCH="${FPV_CAM_PITCH:--15}"
FPV_PARAMS=""
HERE="$(cd "$(dirname "$0")" && pwd)"
while [ $# -gt 0 ]; do
  case "$1" in
    --video-to) VIDEO_TO="$2"; shift 2;;
    --mav-to)   MAV_TO="$2"; shift 2;;
    --target)   TARGET="$2"; shift 2;;       # static | moving | none
    --run-dir)  RUN_DIR="$2"; shift 2;;
    --fast)     FAST_PARAMS="--add-param-file $HERE/fast.parm"; shift;;   # fast GO dashes: higher speed / descent / acceleration limits (simulation/fast.parm)
    --gimbal)   GIMBAL_PARAMS="--add-param-file $ARDUPILOT_GAZEBO_DIR/config/gazebo-iris-gimbal.parm"; shift;;   # ArduPilot drives the camera gimbal (target_tracker --gimbal-centre)
    --c12-camera) C12_CAMERA=1; shift;;     # a C12-like camera: 1280x720, ~100 deg (target_tracker --profile sim-c12cam)
    --fpv-camera) FPV_CAMERA=1; C12_CAMERA=1; FPV_PARAMS="--add-param-file $HERE/fpv.parm"; shift;;   # no gimbal, FPV speed (target_tracker --profile sim-fpv)
    --no-gui)   GUI=0; shift;;                # physics + camera only: saves about two CPU cores (open the GUI later with: gz sim -g)
    *) echo "unknown option $1"; exit 2;;
  esac
done
CAM_TOPIC="/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
mkdir -p "$RUN_DIR"
if [ -s "$RUN_DIR/pids" ]; then
  echo "[sim_up] $RUN_DIR/pids exists: run simulation/sim_down.sh first (or remove the file if nothing is running)"; exit 1
fi
: > "$RUN_DIR/pids"

start() {  # name, command...   (own process group; PGID recorded)
  local name="$1"; shift
  setsid "$@" > "$RUN_DIR/$name.log" 2>&1 < /dev/null &
  local pid=$!
  echo "$name $pid" >> "$RUN_DIR/pids"
  echo "[sim_up] started $name (pid $pid), log $RUN_DIR/$name.log"
}
wait_for() {  # description, timeout_s, command...
  local what="$1" t="$2"; shift 2
  local i=0
  while ! "$@" >/dev/null 2>&1; do
    sleep 2; i=$((i+2))
    if [ $i -ge "$t" ]; then echo "[sim_up] TIMEOUT waiting for $what"; return 1; fi
  done
  echo "[sim_up] ready: $what (${i}s)"
}

# 0. --c12-camera: the stock gimbal camera is 640x480 with a 115 deg view -- a person 15 m away is ~12 px, too small to track
#    reliably; the real C12 is 1280x720 at ~100 deg (~2.6x the pixels per degree). Copy the installed gimbal model into this run's
#    folder with the camera patched to that, and put it first on Gazebo's model path (nothing in ardupilot_gazebo is modified).
if [ "$C12_CAMERA" = "1" ]; then
  mkdir -p "$RUN_DIR/models"; rm -rf "$RUN_DIR/models/gimbal_small_3d"
  cp -r "$ARDUPILOT_GAZEBO_DIR/models/gimbal_small_3d" "$RUN_DIR/models/"
  python3 - "$RUN_DIR/models/gimbal_small_3d/model.sdf" <<'PYEOF'
import re, sys
p = sys.argv[1]; s = open(p).read()
s = re.sub(r"<horizontal_fov>[^<]*</horizontal_fov>", "<horizontal_fov>1.745</horizontal_fov>", s, count=1)
s = re.sub(r"<width>\s*640\s*</width>", "<width>1280</width>", s, count=1)
s = re.sub(r"<height>\s*480\s*</height>", "<height>720</height>", s, count=1)
# the camera's RTP stream on its OWN port (5599): QGroundControl binds UDP 5600 (its default video port) and then took the camera's
# packets from the relay -- the tracker got no video at all, or 1-5 s freezes when they were shared (found 2026-10-15)
s = re.sub(r"<udp_port>\s*5600\s*</udp_port>", "<udp_port>5599</udp_port>", s, count=1)
open(p, "w").write(s)
PYEOF
  echo "[sim_up] C12-like camera: 1280x720, 100 deg, RTP on udp 5599 (model copy in $RUN_DIR/models)"
  CAM_UDP_PORT=5599
fi

# 0b. --fpv-camera: an FPV drone has no gimbal -- the camera is bolted to the airframe, so the drone itself turns / climbs to keep
#     the target in view. The copied gimbal's three joints are welded (type fixed) and the camera sensor is rotated to look forward,
#     tilted FPV_CAM_PITCH degrees (at zero joint angles it looks straight ahead, level). The copy of iris_with_gimbal drops the
#     gimbal's servo channels and joint controllers. Same model names, so the world and the camera topic are unchanged.
if [ "$FPV_CAMERA" = "1" ]; then
  if [ -n "$GIMBAL_PARAMS" ]; then echo "[sim_up] --fpv-camera has no gimbal: drop --gimbal"; exit 2; fi
  rm -rf "$RUN_DIR/models/iris_with_gimbal"; cp -r "$ARDUPILOT_GAZEBO_DIR/models/iris_with_gimbal" "$RUN_DIR/models/"
  python3 - "$RUN_DIR/models" "$FPV_CAM_PITCH" <<'PYEOF'
import math, re, sys
d, tilt = sys.argv[1], float(sys.argv[2])
def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]
mul = lambda A, B: [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
tr = lambda A: [list(r) for r in zip(*A)]
# the gimbal is mounted at rpy (90, 0, 90) deg on the airframe; the camera should look along the body's x axis, pitched by `tilt`
# (+ up; a rotation about the body's y axis by -tilt in Gazebo's FLU frame). Sensor rotation = mount^-1 * wanted.
S = mul(tr(rpy(math.pi / 2, 0, math.pi / 2)), rpy(0, math.radians(-tilt), 0))
r, p, y = math.atan2(S[2][1], S[2][2]), math.asin(-S[2][0]), math.atan2(S[1][0], S[0][0])
g = d + "/gimbal_small_3d/model.sdf"; s = open(g).read()
s, n = re.subn(r'<joint name="(yaw|roll|pitch)_joint" type="revolute">', r'<joint name="\1_joint" type="fixed">', s); assert n == 3, n
s = re.sub(r'(<joint name="(?:yaw|roll|pitch)_joint" type="fixed">.*?)<axis>.*?</axis>', r'\1', s, flags=re.S)
s, n = re.subn(r'(<sensor name="camera" type="camera">\s*(?:<[^>]+>[^<]*</[^>]+>\s*)*?)<pose>[^<]*</pose>', r'\g<1><pose>0 0 0 %.6f %.6f %.6f</pose>' % (r, p, y), s, count=1); assert n == 1
open(g, "w").write(s)
m = d + "/iris_with_gimbal/model.sdf"; s = open(m).read()
s = s.replace('<joint name="gimbal_joint" type="revolute">', '<joint name="gimbal_joint" type="fixed">')
s = re.sub(r'(<joint name="gimbal_joint" type="fixed">.*?)<axis>.*?</axis>', r'\1', s, flags=re.S)
s, n = re.subn(r'\s*(?:<!--[^>]*-->\s*)?<control channel="(?:8|9|10)">.*?</control>', '', s, flags=re.S); assert n == 3, n
s, n = re.subn(r'\s*<plugin\s+filename="gz-sim-joint-position-controller-system".*?</plugin>', '', s, flags=re.S); assert n == 3, n
open(m, "w").write(s)
print("[sim_up] FPV camera: fixed to the airframe, tilted %.0f deg (sensor rpy %.4f %.4f %.4f)" % (tilt, r, p, y))
PYEOF
  [ $? = 0 ] || exit 1
fi

# 1. Gazebo (NVIDIA offload avoids the slow CPU renderer; see scripts/run_gazebo_laptop.sh)
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GAZEBO_DIR/models:$ARDUPILOT_GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"
[ "$C12_CAMERA" = "1" ] && export GZ_SIM_RESOURCE_PATH="$RUN_DIR/models:$GZ_SIM_RESOURCE_PATH"   # the patched camera model first
if [ "${GZ_GPU:-nvidia}" = "nvidia" ]; then
  export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
if [ "$GUI" = "1" ]; then start gazebo gz sim -v3 -r "$WORLD"; else start gazebo gz sim -v3 -s -r "$WORLD"; fi   # EXPERIMENTAL: failed once with a GLX error; --headless-rendering segfaults here
wait_for "Gazebo camera topic" 150 bash -c "gz topic -l | grep -q '${CAM_TOPIC}\$'" || exit 1

# 2. SITL (no MAVProxy of its own); parameters layered explicitly as in scripts/run_sitl_laptop.sh
PARAMS="$ARDUPILOT_DIR/Tools/autotest/default_params"
( cd "$ARDUPILOT_DIR" && start sitl ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w \
    --add-param-file "$PARAMS/copter.parm" --add-param-file "$PARAMS/gazebo-iris.parm" $GIMBAL_PARAMS $FAST_PARAMS $FPV_PARAMS --no-mavproxy )
# SITL waits for its first client on TCP 5760 before it finishes starting (the other ports open afterwards),
# so: wait for 5760 -> start MAVProxy (the client) -> wait for the heartbeat on 5762.
wait_for "SITL listening on tcp:5760" 600 bash -c "ss -tln | grep -q ':5760 '" || exit 1

# 3. MAVProxy: QGC (127.0.0.1:14550) and the companion
start mavproxy mavproxy.py --master=tcp:127.0.0.1:5760 --out=127.0.0.1:14550 --out=udp:"$MAV_TO" --daemon
wait_for "SITL heartbeat on tcp:5762" 180 python3 -c "
from pymavlink import mavutil
import sys
m=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
sys.exit(0 if m.wait_heartbeat(timeout=4) else 1)" || exit 1

# 4. (the camera stream is switched on ONCE, after the relay is listening: see step 5. Re-sending the enable message
#    restarts the plugin's GStreamer pipeline each time -- a 5-second keep-alive loop hung the Gazebo server.)

# 4b. companion link gate: the follower connects to tcp:127.0.0.1:5770; fault.sh link-loss freezes this process to cut the link
start mavgate python3 "$HERE/mav_gate.py" --listen 5770 --to 127.0.0.1:5763

# 4c. simulated C12 gimbal camera (TOP protocol on udp://127.0.0.1:5000); it tracks the Gazebo model "target" when asked
start c12sim env PYTHONPATH="$HERE/c12" python3 "$HERE/sim_c12.py" --port 5000 --target target --target-height 0.9

# 5. video relay: Gazebo RTP/H.264 on 127.0.0.1:5600 -> the companion (same as scripts/relay_video_to_jetson.sh)
VH="${VIDEO_TO%:*}"; VP="${VIDEO_TO##*:}"
# A second copy goes to 127.0.0.1:5601 so a clip can be recorded locally without touching the companion's stream.
start relay gst-launch-1.0 -q udpsrc port="${CAM_UDP_PORT:-5600}" caps="application/x-rtp, media=video, encoding-name=H264, payload=96" ! tee name=t ! queue ! udpsink host="$VH" port="$VP" t. ! queue ! udpsink host=127.0.0.1 port=5601

# 5b. enable the camera stream once
sleep 1
gz topic -t "${CAM_TOPIC}/enable_streaming" -m gz.msgs.Boolean -p 'data: 1' >/dev/null 2>&1 && echo "[sim_up] camera streaming enabled (once)"

# 6. target boxes
case "$TARGET" in
  moving) start target python3 "$HERE/move_target.py";;
  static) python3 "$HERE/move_target.py" --once 0 15 > "$RUN_DIR/target.log" 2>&1; echo "[sim_up] target placed (static)";;
  none)   ;;
esac
echo "[sim_up] DONE. Next: python3 $HERE/takeoff.py   |  status: $HERE/sim_status.sh  |  stop: $HERE/sim_down.sh"
