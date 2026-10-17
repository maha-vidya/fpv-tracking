#!/usr/bin/env python3
"""FPV STABILITY TEST: take off -> hover -> lock + follow -> GO, measured, with PASS / FAIL per phase.

Run it after ANY change to simulation/fpv.parm, the GO SETTINGS block, or the follow / GO control code -- it catches the
shaking and wobbling that were fixed on 2026-10-16 if they ever come back.

  cd ~/fpv-tracking/simulation && ./sim_up.sh --fpv-camera --target moving --no-gui --video-to 127.0.0.1:5603 --mav-to 127.0.0.1:14561
  python3 stability_test.py                 # the drone must be on the ground (fresh simulator); takes ~2 minutes
  ./sim_down.sh

Nothing else may be connected: no other target_tracker, no QGroundControl (this listens on UDP 14550, where QGC would).
If a Gazebo window you opened yourself is running, start the simulator AND this test with the same GZ_PARTITION=<name> so
the two do not mix.

What it measures (from the autopilot's attitude, its position, and the simulated camera's own images):
  shake       fast roll/pitch rate jitter (deg/s) -- the airframe vibrating
  pitch/roll  the extremes (deg) -- a fixed camera tilts with the drone, so these move the picture
  nods/min    how often the pitch swings back and forth by more than 3 deg -- the "nodding"
  jitter      frame-to-frame up/down movement of the whole picture (px) -- what the pilot sees shaking

Limits (2026-10-16 in SITL: good settings measured roll +-0.2 deg on take-off, follow pitch +-7 deg, GO pitch +-26 deg; the
bad ones roll -52 deg, follow pitch +-22 deg at 39 nods/min, GO pitch -42..+45 deg):
"""
import sys, os, time, math, json, glob, threading, subprocess, statistics as st
import numpy as np, cv2
from pymavlink import mavutil

LIMITS = {   # phase: (max |roll| deg, max |pitch| deg, max shake deg/s, max nods/min, max picture jitter px)
    "takeoff": (3.0, 3.0, 2.0, 5.0, 15.0),
    "hover":   (3.0, 3.0, 2.0, 5.0, 1.0),
    "follow":  (5.0, 12.0, 10.0, 20.0, 1.5),
    "go":      (20.0, 55.0, 25.0, 20.0, 6.0),
}
MIN_ALT = 0.0   # no artificial safety floor (FPV target dive)
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.dirname(HERE)
BIN = os.path.join(REPO, "tracker", "build", "target_tracker")
OUT = os.path.expanduser("~/sim_run_stability"); os.makedirs(OUT, exist_ok=True)
IMG = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

phase = ["boot"]; att = []; alts = []; shifts = []; last_img = [None]; prev_small = [None]; stop = [False]
m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=250)
if m.wait_heartbeat(timeout=30) is None: sys.exit("no heartbeat on UDP 14550: is the simulator up (sim_up.sh), and QGC closed?")
m.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 25, 1)
def rx():
    while not stop[0]:
        x = m.recv_match(type=["ATTITUDE", "LOCAL_POSITION_NED"], blocking=True, timeout=0.5)
        if x is not None and x.get_type() == "LOCAL_POSITION_NED": alts.append((phase[0], -x.z)); continue
        if x is not None: att.append((time.time(), phase[0], math.degrees(x.roll), math.degrees(x.pitch), math.degrees(x.rollspeed), math.degrees(x.pitchspeed)))
threading.Thread(target=rx, daemon=True).start()

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
win = cv2.createHanningWindow((320, 180), cv2.CV_32F)
def cam_cb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    last_img[0] = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    g = cv2.cvtColor(cv2.resize(a, (320, 180)), cv2.COLOR_RGB2GRAY).astype(np.float32)
    if prev_small[0] is not None:
        (dx, dy), _ = cv2.phaseCorrelate(prev_small[0], g, win); shifts.append((phase[0], dy * msg.height / 180))
    prev_small[0] = g
node = Node(); node.subscribe(Image, IMG, cam_cb)

def red_box(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mk = (((hsv[..., 0] < 6) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 50)).astype(np.uint8)
    n, lab, s, c = cv2.connectedComponentsWithStats(mk)
    if n < 2: return None
    j = max(range(1, n), key=lambda q: s[q, 4]); return tuple(int(v) for v in s[j, :4]) if s[j, 4] >= 40 else None

def reversals(v, dead):
    n = 0; last = 0
    for a in v:
        s = 1 if a > dead else -1 if a < -dead else 0
        if s and last and s != last: n += 1
        if s: last = s
    return n

def hf(v, w):   # jitter: the signal minus its own moving average (removes deliberate, slow manoeuvres)
    return st.pstdev([v[i] - st.mean(v[max(0, i - w):i + w + 1]) for i in range(len(v))]) if len(v) > 2 else 0.0

def measure(name):
    A = [a for a in att if a[1] == name]; S = [s[1] for s in shifts if s[0] == name]
    if len(A) < 10: return dict(phase=name, ok=False, why="no attitude data")
    dur = A[-1][0] - A[0][0]; pit = [a[3] for a in A]; rol = [a[2] for a in A]
    r = dict(phase=name, roll=max(map(abs, rol)), pitch=max(map(abs, pit)),
             shake=max(hf([a[4] for a in A], 12), hf([a[5] for a in A], 12)),
             nods=reversals([p - st.mean(pit) for p in pit], 3.0) / max(dur, 1) * 60, jitter=hf(S, 3))
    H = [a[1] for a in alts if a[0] == name]; r["min_alt"] = min(H) if H else float("nan")
    lim = LIMITS[name]; bad = []
    if name in ("follow", "go") and H and min(H) < MIN_ALT: bad.append("CRASH RISK: down to %.1f m (< %.1f m)" % (min(H), MIN_ALT))
    for key, l in zip(("roll", "pitch", "shake", "nods", "jitter"), lim):
        if r[key] > l: bad.append("%s %.1f > %.1f" % (key, r[key], l))
    r["ok"] = not bad; r["why"] = ", ".join(bad); return r

results = []
phase[0] = "takeoff"
tk = subprocess.run([sys.executable, os.path.join(HERE, "takeoff.py"), "--alt", "12"], capture_output=True, text=True, timeout=180)
if tk.returncode != 0: print("takeoff failed", flush=True); sys.exit("takeoff failed: " + tk.stdout[-300:] + tk.stderr[-300:])
results.append(measure("takeoff"))
phase[0] = "hover"; time.sleep(10); results.append(measure("hover"))

phase[0] = "follow"
for f in glob.glob(OUT + "/follow_*.jsonl"): os.remove(f)
trk = subprocess.Popen([BIN, "--profile", "sim-fpv", "--no-display", "--enable-dash", "--log-dir", OUT],
                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1, cwd=os.path.join(REPO, "tracker"))
lines = []
threading.Thread(target=lambda: [lines.append(l.rstrip()) for l in trk.stdout], daemon=True).start()
t0 = time.time()
while time.time() - t0 < 25 and not any("link up" in l for l in lines): time.sleep(0.2)
time.sleep(2)
b = red_box(last_img[0]) if last_img[0] is not None else None
if b is None:
    trk.kill(); sys.exit("no red target in the camera view: start the simulator with --target moving (or static)")
trk.stdin.write("click %d %d\n" % (b[0] + b[2] // 2, b[1] + b[3] // 2)); trk.stdin.flush()
time.sleep(25); results.append(measure("follow"))

phase[0] = "go"; n_before = len(lines)
trk.stdin.write("go\n"); trk.stdin.flush(); time.sleep(15); results.append(measure("go"))
go_lines = lines[n_before:]
if not any("GO-TO sent" in l for l in go_lines):
    results[-1]["ok"] = False; results[-1]["why"] = "GO was not sent: " + "; ".join(l for l in go_lines if "GO" in l)[:200]
phase[0] = "end"
try: trk.stdin.write("quit\n"); trk.stdin.flush(); trk.wait(timeout=15)
except Exception: trk.kill()
stop[0] = True

print("\nFPV STABILITY TEST  (limits: roll / pitch deg, shake deg/s, nods/min, picture jitter px)")
for r in results:
    if "roll" not in r: print("  %-8s FAIL  %s" % (r["phase"], r["why"])); continue
    lim = LIMITS[r["phase"]]
    print("  %-8s %s  roll %5.1f/%-4.0f pitch %5.1f/%-4.0f shake %5.1f/%-4.0f nods %5.1f/%-4.0f jitter %4.1f/%-4.1f %s" % (
        r["phase"], "PASS" if r["ok"] else "FAIL", r["roll"], lim[0], r["pitch"], lim[1], r["shake"], lim[2], r["nods"], lim[3],
        r["jitter"], lim[4], ("<- " + r["why"]) if not r["ok"] else ""), " min alt %.1f m" % r["min_alt"])
ok = all(r["ok"] for r in results)
print("RESULT:", "PASS" if ok else "FAIL")
sys.stdout.flush(); os._exit(0 if ok else 1)   # os._exit: the Gazebo transport thread must not hang the exit
