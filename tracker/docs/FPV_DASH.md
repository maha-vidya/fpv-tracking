# Running the FPV tracking & dash drone in simulation

This is the step-by-step runbook for the `tracker/` companion program: bring up Gazebo + ArduPilot SITL,
run the companion program, lock/follow/go-to-target from the keyboard, stdin, RC, or QGC, and what to
check. Every command below was run live while writing this doc. All paths are relative to the repository
root (`drone-gazebo/`) — open a new terminal and `cd` there first for each numbered step.

## 0. One-time setup

```bash
cd tracker
scripts/fetch_mavlink.sh        # pins the MAVLink C headers (only needed once)
mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)"
./target_tracker --selftest     # expect: 466 checks, 0 failed
```

## 1. Start the simulator (laptop, one terminal)

```bash
cd simulation
./sim_up.sh --fpv-camera --target moving        # Gazebo + SITL + MAVProxy + video relay
```

Wait for `[sim_up] DONE.` (about 30-40s the first time). If anything looks wrong:

```bash
./sim_status.sh      # what's up, drone/target pose, SITL link
./sim_down.sh        # stop everything cleanly (only kills what sim_up.sh started)
```

**`--target`** options: `static` (default, one box) | `moving` (the box + a lookalike drift side to side) |
`none`. For a multi-object scene with mixed fast/slow traffic instead, skip `--target` and use
`spawn_traffic.py` (step 5).

## 2. Take off

**Takeoff and altitude are the pilot's job, both in real operation and in this program — `main.cpp`
contains no takeoff or altitude-control logic at all.** It only ever sends follow velocity commands (once
`LOCKED`) or the one FPV Dash command, and only after the vehicle is already in GUIDED, which the pilot
puts it into. Prefer flying takeoff through QGroundControl (step 6 covers connecting it) rather than a
script, so the test matches real operation:

1. Connect QGC to the simulation (`127.0.0.1:14550`, see step 6) — it should auto-connect once
   `sim_up.sh` is running.
2. Arm and take off from QGC as you would on the real aircraft, and climb to whatever altitude you want
   to test at (10-15m is a recommended test altitude for good camera perspective).
3. Leave the vehicle in GUIDED once airborne — that's the mode the companion program requires before it
   will accept a lock.

A faster option for quick, no-pilot dev iteration (not a substitute for the QGC path when testing
anything timing- or operator-sensitive):

```bash
cd simulation
python3 ./takeoff.py --alt 12     # arms, climbs to 12m, holds in GUIDED
```

Either way, if you want a higher test altitude and/or faster GUIDED transits than the default SITL
parameters give you, raise the waypoint speed once airborne:

```bash
python3 - <<'EOF'
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
m.wait_heartbeat()
for p, v in [("WP_SPD", 6.0), ("WP_SPD_UP", 4.0), ("WP_SPD_DN", 4.0), ("WP_ACC", 3.0)]:
    m.mav.param_set_send(m.target_system, m.target_component, p.encode(), float(v),
                          mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.1)
EOF
```

## 3. Place/move the target

```bash
cd simulation
python3 ./move_target.py --once 0 15       # one fixed box at (0, 15) -- good for accuracy checks
# or, already running continuously from `sim_up.sh --target moving`:
#   red + lookalike orange drift side to side, crossing in view
```

## 4. Run the companion program (new terminal)

```bash
cd tracker
./build/target_tracker --profile sim --enable-rescue --rescue-hover-height 3 --log-dir /tmp/rescue_log
```

With a display available, a window opens (`Follow (click = instant lock, drag+s = precise lock, u =
unlock, g = go-to-target, x = abort, q = quit)`) — **click once** on a moving target for an instant,
auto-sized lock (no drag, no confirm key — faster than a precise drag for a fast mover), or **drag a box**
around it and press **`s`** to confirm a precise one. Headless (no display: add `--no-display`), use stdin
text commands instead (one per line, `Enter` to send):

| Command | Effect |
|---|---|
| `lock X Y W H` | lock a box at pixel (X,Y) size (W,H) — e.g. `lock 290 362 60 70` |
| `unlock` | release the lock |
| `go` | **GO now**, one press (needs LOCKED ≥2s and every readiness gate). `--go-confirm`: first press arms, second within 3s confirms |
| `abort` | abort an ARMED rescue, or cancel+BRAKE an in-flight dash |
| `quit` | clean shutdown |

Key equivalents when a display is open: `s`=lock (after dragging), `u`/`r`=unlock, `g`=go (press/confirm),
`x`=abort, `q`=quit.

**Example full sequence (headless), after the target is at (0,15) and the drone is hovering at (0,0,-12)
NED:**

```
lock 290 362 60 70      <- adjust to where the target actually appears; watch for "tracker box rejected"
                           or "LOCKED on a ..." in the log to tell if it took
go                       <- arms (watch for "RESCUE ARMED")
go                       <- confirms within 3s (watch for "GO-TO sent: N m diagonal at 5 m/s")
```

Verify the result against ground truth:

```bash
timeout 5 gz topic -e -t /world/iris_runway/pose/info -n 1 | python3 -c "
import sys,re
t=sys.stdin.read()
for n in ('iris_with_gimbal','target_red'):
    i=t.find('name: \"%s\"'%n); b=t[i:i+500]
    p=re.search(r'position \{(.*?)\}',b,re.S)
    print(n, dict(re.findall(r'([xyzw]): (\S+)',p.group(1))) if p else None)
"
```

The drone should end up hovering **`--rescue-hover-height`** (default 3 m) metres above the target's
altitude, not on top of it.

### Centring and keeping up with a fast target (follow)
- **Centring:** the yaw follows the target's own turn rate around the drone, measured from where it was in the world when
  each frame was taken. The steering uses where the target is now, accounting for the ~0.2 s video delay. This is on in
  `--profile sim-c12cam` (`--yaw-ff-mode world`, `--kyaw 3`). The gimbal re-centres vertically within 1 degree (`--gimbal-db`).
  The window shows a crosshair and `CENTRE off x,y px`: green when close, yellow when further off, red when far off.
- **Speed match (on by default):** the drone flies at the target's own speed along the line of sight, so a car driving off
  at 12 m/s is kept up with. It eases in 1-2.5 s after the lock and eases out over ~1 s if the target is lost. It backs away
  at most `--speed-match-back` m/s (3), and is limited by `--ceil-vx` (8 m/s by default: raise it for faster targets, e.g.
  `--ceil-vx 15`). Turn it off with `--no-speed-match`.
- **Limits in `--profile sim-c12cam`:** up to 18 m/s forward (`--ceil-vx`), 2 rad/s turn (`--ceil-yaw`), 6 m/s² speed change,
  5 rad/s² turn change (`--yaw-accel`), and distance corrections at up to 4 m/s.
- **View height:** a fast target (≥ 8 m/s, or crossing quickly) is zoomed out a step at a time, and once at the widest view the
  drone climbs for a wider view. A target bigger than the view (`--view-max-px`) makes it climb too. A small target at full zoom
  brings it back down, never below the height it had at the lock. Climb rate `--view-climb-rate` (1.5 m/s), ceiling
  `--view-max-alt` (30 m); off with `--no-view-climb`.
- **Link gaps:** the link counts as alive while any autopilot message arrives (attitude is 50 per second). A gap of up to 5 s
  with the vehicle still in GUIDED resumes the same lock. A longer gap, or a mode change, drops it as before.

### Prediction, GO guidance and identity (2026-10-14)
- **Image motion filter: IMM.** Two Kalman filters run together: calm, and manoeuvring (turning, braking, reversing, speeding
  up). They are blended by how well each predicted each box.
- **Ground track: IMM-UKF.** A straight-motion model plus a coordinated-turn model (unscented). Predicting where a turning
  vehicle will be in 1 s: 1.5 m error instead of 6.4 m (self-test).
- **GO guidance** (`--go-guidance pursuit|lead|intercept|pn`, default `pn`, `--pn-gain 3`). The old law (`pursuit`) braked as if
  the target stood still, and never reached a target crossing at 5 m/s (simulated). The new modes match the target's motion and
  close on it relative to it: about 5.5 s from 40 m on a crossing or turning target.
- **Small targets** (under `--full-res-below` 20 px) are tracked at full resolution.
- **ReID identity check** (with `--reid`): about every half second the tracked box is fingerprinted. Two checks in a row
  that don't match the target make the tracker search for it again, instead of following a lookalike it slid onto.
- **Gimbal:** it now moves a little every frame instead of in 10 Hz steps, and keeps following the prediction for 0.3 s when
  a frame is doubtful. If the image-based camera-motion estimate fails, the commanded gimbal tilt and the drone's turn are used.
- **ReID model in the tracker (no Python sidecar).** When the build finds OpenVINO (the pip package ships its C++ files),
  `--reid` loads DINOv2 inside `target_tracker` through the OpenVINO C++ API, on its own thread. Its fingerprints are identical
  to the sidecar's (cosine 1.0000), with no UDP or JPEG hop. Loading takes about 0.5–1.5 s at start; then about 45–75 ms per crop
  on the laptop. Model folder: `--reid-model ~/models/dinov2`. `--ai sidecar` uses the Python sidecar instead (a build without
  OpenVINO). `target_tracker --ai-bench IMAGE X Y` times the model on an image.
- **Range from the box size** (D = f·H/h): the target's size H is learned from the reliable fixes (camera ≥ 25° down). The
  size range is then blended with the angle-based range according to how much each can be trusted, and used alone where the
  view is too shallow for an angle-based fix.
- **Filter limit for each new box:** the jump from the prediction must lie within the larger of 4 standard deviations of
  the filter's uncertainty and 0.8 box + 2.5× the target's speed per frame, against CSRT hopping onto a neighbour.
- **Late clicks fixed:** a click or tap is locked on the frame the operator actually saw, with the last 0.6 s of full-size
  frames kept for this, and the result is replayed to the live frame. A click made while the previous one is still being
  processed is queued (the newest one wins), not dropped. An unlock cancels pending clicks and completions.
- **No wavering after a loss:**
  - a re-anchored box (the prediction itself) is no longer fed back into the filter, and at most 3 re-anchors in a row are
    allowed before the target counts as hidden;
  - coasting no longer turns the drone toward the prediction (`--coast-yaw` to re-enable), and the gimbal holds
    (`--gimbal-follow-prediction` to re-enable);
  - the yellow predicted box is shown only on real measurements.
- **No locking of a same-colour object:**
  - a hidden target is re-found only where the IMM says it can be;
  - with `--reid`, a re-find needs a DINOv2 match (similarity ≥ 0.6), and the identity check rejects a box it doesn't
    recognise;
  - the ReID gallery only learns verified frames;
  - nothing is re-found during a disturbance (gimbal moving, zoom change, hard pitch or turn), and ReID re-checks the lock as
    soon as the image is steady again;
  - the crossing guard keeps the target when it turns and the IMM prediction agrees.
- **Digital zoom is off by default** in `sim-c12cam` (`--zoom-max 2` turns it on). Small targets are tracked at full
  resolution instead.
- **No overflight:** with the target more than 70° below, the drone stops moving forward; beyond 80° it backs off.
- **Log fields:** each `follow_*.jsonl` line now has `gim` (commanded and reported gimbal pitch), `cm` (camera motion: 0 none,
  1 image, 2 fallback) and `kfv` (filter velocity).

## 5. Multiple moving objects (cars + people, fast + slow)

```bash
cd simulation
python3 ./spawn_traffic.py --cars 3 --people 4
# or with a repeatable layout and custom speed ranges:
python3 ./spawn_traffic.py --cars 5 --people 6 --seed 1 --car-speed 3 14 --person-speed 0.8 2.2
# no network / no Fuel cache available: fall back to plain colored boxes
python3 ./spawn_traffic.py --boxes
```

**People are animated `<actor>`s**: Gazebo's skinned walking person (Fuel `Mingfei/actor`) with real
walk/run skeletal animation (`run` above 2.2 m/s), legs kept in step with their speed (`interpolate_x`),
following a looping scripted `<trajectory>` along their straight or curvy path and turning round at each
end. Gazebo animates and moves them itself, so their limbs and silhouette change every frame — a much
more realistic test for CSRT and the colour check than a rigid mesh sliding along. `--people-mesh`
restores the old rigid people. **Cars** stay real OpenRobotics meshes moved by this script (car meshes
have no skeleton to animate). Everything is fetched from Gazebo Fuel once and cached under `~/.gz/fuel/`.
Prints each actor's name, speed and starting position (`traffic_car_0`, `traffic_person_2`, ...).
Ctrl+C stops the cars; the walkers keep walking until the simulator is restarted.

**Many-colour vehicles roaming at random** (no download: built from simple shapes):

```bash
python3 ./spawn_traffic.py --cars 0 --people 0 --vehicles 14          # alongside a people run: its own lock
python3 ./spawn_traffic.py --vehicles 20 --vehicle-area 100 --vehicle-tiers fast,medium,slow --vehicle-types car,bus,truck
```

Cars, vans, buses (11 m), trucks (cab + cargo box in another colour) and motorbikes with a rider, in 12 colours. Each drives
to random points inside +/-`--vehicle-area` m (default 70) in any direction, turning like a vehicle and slowing for sharp
turns. The speed tiers are given in turn: slow 1.5-4, medium 5-9, fast 11-18 m/s (buses and trucks ~25% slower, motorbikes
~15% faster). The cruise speed changes now and then within the tier. Names: `traffic_veh_N`.

**Walkers in different shirt colours** (ReID / identity tests): `--shirts red,blue,yellow,white,...`.

**ReID in the tracker** (`--reid`): the DINOv2 model runs inside the tracker (no helper program needed when it is built with
OpenVINO):

```bash
./build/target_tracker --profile sim-c12cam --enable-rescue --reid
```

Without OpenVINO in the build: start the helper first, `python3 tracker/detector_sidecar.py --reid ~/models/dinov2`, then add
`--ai sidecar`.

The target's fingerprints are collected while it is clearly visible (up to 6). They are used only when a hidden target has
to be found again among lookalikes: the candidates are compared with them, and one with a clearly different shape is never
taken.

**A real-mesh world is noticeably heavier to render than the default flat `iris_runway.sdf`.** Confirmed
live on a mid-range laptop: a 16-tile paved road + 6 buildings pegged `gz sim`'s server/gui processes at
150-200% CPU sustained with a climbing load average, and `sim_down.sh` had to force-kill Gazebo. If you
build a custom world with real-mesh scenery (see `ardupilot_gazebo/worlds/iris_road_scene.sdf` for the
pattern — tiled `Asphalt Plane` includes + a couple of building includes, same `<world name="iris_runway">`
so none of this tooling needs to change), start minimal (one ground tile, one building) and scale up only
while watching `ps aux | grep "gz sim"` CPU% and `uptime`'s load average stay flat, not climbing. `--boxes`
has none of this cost and is the safer default on weaker hardware.

## 6. QGroundControl

QGC is a separate GUI application, started from the simulator's QGC connection (MAVProxy already forwards
to `127.0.0.1:14550`, and the companion's camera component listens for QGC's camera/tracking protocol):

1. Launch QGC on the laptop; it should auto-connect over `127.0.0.1:14550`.
2. **Don't set the video source manually.** QGC gets its video port from the companion's own
   `VIDEO_STREAM_INFORMATION` MAVLink message (`cam_video_uri`, sent automatically once the camera
   component is enabled) -- for `--profile sim` that's **port 5602**, not QGC's usual default of 5600
   (5600 is already taken by `sim_up.sh`'s own Gazebo-camera relay in this profile; confirmed live by
   checking which port QGC actually binds -- `ss -ulpn | grep 5602` -- it is NOT the port QGC's manual
   video-source UI would default to). Just launch `target_tracker` with a matching
   `--video-out udp://127.0.0.1:5602` so there's an annotated feed for QGC to show; if you forget
   `--video-out` entirely, QGC's camera UI sits there with no picture even though everything else (MAVLink,
   Gazebo, SITL) is working fine -- this has been mistaken for "QGC and Gazebo not connecting" when really
   it's "nothing is sending video to the port QGC is already correctly listening on". On
   `--profile real-c12`/`real-c12-gimbal`, `cam_video_uri` is the usual 5600 instead (no relay to collide
   with on real hardware), so `--video-out udp://127.0.0.1:5600` is correct there.
3. **Tap or draw a rectangle** on the video to lock (same effect as dragging in the local window; latency-
   compensated via `--tap-latency-ms`, default 300 — measure the real value for your setup by comparing
   the on-screen `t=...` timestamp against the wall clock at the moment of a tap, and adjust).
4. **Rescue control from QGC**: copy `tracker/qgc/RescueActions.json` into QGC's `MavlinkActions` folder
   (Linux: `~/Documents/QGroundControl/MavlinkActions`) and restart QGC. It adds **Rescue GO**
   (`MAV_CMD_USER_1`: one press = GO; with `--go-confirm` the 1st press arms and shows
   `ARMED <dist> up/dn<h> ETA<s> h:<height source>`, the 2nd within 3 s confirms), **Rescue ABORT** (`USER_2`), **Track UNLOCK** (`USER_3`) and **BRAKE** (`USER_4`, works
   even if the rescue isn't armed), all sent to component **191**; the same actions can be mapped to joystick
   buttons. Only commands from `--gcs-sysid` (default 255, QGC's own id) are accepted. The file format is
   QGC's documented `MavlinkActions` format, but it has **not** been clicked through in a real QGC window yet.
5. Changing mode from QGC's dropdown drops the lock and stops commands (leaving GUIDED always does).
6. QGC's own "Go to location"/"Orbit"/"ROI" during an active dash will make the companion cancel its own
   resends (not fight QGC) — watch for `GO-TO cancelled: another position target was detected`.

### Lock / unlock / GO in real time from QGC (no companion window needed)

| What | How in QGC |
|---|---|
| **Lock** | Point the camera at the target (sticks/gimbal), press the **Track LOCK** action (`MAV_CMD_USER_5`): it locks the object at the centre of the image (ground-colour model, else GrabCut; snapped to the one nearest object if the centre misses). Or tap/draw on the video if your QGC build shows the camera-tracking controls. |
| **Unlock** | **Track UNLOCK** action, or stop tracking in the video widget. |
| **GO** | **Rescue GO** once: GO at once. With `--go-confirm`, twice: the first press arms and shows `ARMED <dist> up/dn<h> ETA<s> h:<height>` *and* a **`DEST ?`** marker on the map; the second press (within 3 s) confirms. |
| **Abort / panic** | **Rescue ABORT**, or **BRAKE**. |
| **See the target** | The map shows a **`TARGET`** icon at the estimated ground position (with its heading and speed once moving), relabelled `TGT ?` while coasting, `TGT LOST` / `TGT OFF` after a loss/unlock, and a **`DEST`** icon for an active dash. |

Put `tracker/qgc/RescueActions.json` in QGC's `MavlinkActions` folder (Linux:
`~/Documents/QGroundControl/MavlinkActions`), restart QGC, and pick it as the Fly View custom-actions file;
the same actions can be bound to joystick/gamepad buttons for one-press use.

The map icons are `ADSB_VEHICLE` messages, which QGC draws as traffic icons with a callsign. They can never be
mistaken for real traffic by the **flight controller**: ArduPilot's ADS-B code drops reports whose ICAO address is
above 24 bits or that lack the altitude flag, and these use both (ICAO `0x7E000001` / `0x7E000002`, no altitude).
Verified in SITL: the marker reaches QGC's port and lands within ~1 m of the target's true position. QGC removes
an icon 120 s after its last update (no explicit delete exists), which is why the final label says `OFF`.
`--no-map-marker` turns them off.

### How the lock box is found

A rough selection (click, tap, centre LOCK, RC centre-lock) becomes a box in two steps, both in the tracker itself:
1. **Ground-colour model** (~15 ms): the colours around the click are learned as "ground"; the object is what differs from them.
   If that fails, **GrabCut** (OpenCV) separates the object around the click.
2. In the background (~0.2 s later), **GrabCut completion** grows the box to the whole object (a truck's cab + cargo, a bus, a van
   seen from close up), with shadows trimmed and growth capped so a neighbouring object is not swallowed.

A carefully drawn rectangle is tightened by GrabCut.

## 7. RC switches (either instead of, or alongside, QGC)

```bash
cd tracker
./build/target_tracker --profile sim --enable-rescue \
    --rc-centerlock-channel 6 --rc-rescue-channel 7
```

Drive channel values from another terminal with `simulation/sim_events.py` (check its `--help`; it sets RC
override values via MAVLink). The rescue channel is read as a **3-position switch**: low = idle/abort,
any move up (low→mid or →high) = **GO**; back to low during a dash = stop (BRAKE). With `--go-confirm`: low→mid = arm,
mid→high = confirm, so a 2-position switch cannot complete it.

## 8. Shutting down

```bash
# Ctrl+C the companion program (sends 3x zero velocity before exiting)
cd simulation
./sim_down.sh
```

## Troubleshooting

- **`g` does nothing.** Nine times out of ten this is `--enable-rescue` missing from the launch command —
  without it the whole rescue module is off and `g`/`go` is silently inert. As of this fix it now says so
  explicitly (`'g' has no effect: rescue is disabled...`) instead of staying silent. If rescue *is* enabled
  and `g` still seems to do nothing, check the console/log for `RESCUE IDLE (not ready: ...)` — it now
  names the specific blocking condition (not locked, not GUIDED, no GPS, no destination estimate yet, etc.)
  instead of just "not ready".
- **Tracker keeps losing fast-moving or crossing targets.** Not a ByteTrack/BoT-SORT situation — those are
  multi-object trackers built on top of a per-frame detector (normally YOLO), which this project deliberately
  removed (operator-selected, class-agnostic tracking, no NVIDIA/closed-class dependency). Three things that
  actually help here, in order of how much they moved the needle when measured live:
  1. **Camera frame rate was the dominant cause.** `--log-dir`'s JSONL records measured `fps`; a live sim run
     averaged only ~5.6 fps end-to-end, well below the camera sensor's own 10 Hz `update_rate` (now raised to
     24 in `ardupilot_gazebo/models/gimbal_small_3d/model.sdf` — a *different* repo, rebuild not required,
     just restart the sim stack to pick it up). At 5.6 fps a 10-12 m/s car moves 30-45 px between frames,
     easily enough to overrun a correlation tracker's search window regardless of which one you use.
  2. **`--csrt-padding`** (default 3.0, OpenCV's own CSRT default) widens the area CSRT searches around the
     previous box, directly trading off against overrun from fast motion. Phase 0's "default parameters, no
     safe change found" finding predates this specific failure mode (fast crossing traffic) — worth
     re-benchmarking at a higher value (e.g. `--csrt-padding 4.5`) against the Phase 0 sequences before
     trusting it as a new default, not just accepting it on faith for this one scenario.
  3. A plain click now locks instantly (see above) — re-locking after a lost target is the main practical way
     to recover from a crossing-object mixup today. The tracker also cross-checks the locked box's colour
     against the box at lock time on every frame; a box that's geometrically plausible but a different colour
     (CSRT having drifted onto a crossing object) is rejected and ages into COASTING → LOST rather than being
     silently followed — this catches a *different-coloured* crosser, not a same-coloured one (e.g. two red
     cars), which no cheap check here can tell apart; only a detector-based ReID approach like BoT-SORT's
     actually solves that, at the cost of reintroducing everything above that it was removed to avoid.
- **Drone descends while following, then loses the target in ground clutter.** The follow loop (non-gimbal
  `--profile sim`) commands descent to keep the target vertically centred in frame, down to `--alt-floor`
  (default 1.5 m — too low to matter against ground traffic). Raise it, e.g. `--alt-floor 8`, to keep the
  drone up where it has a clearer, more separated view of ground-level actors. `--profile sim-gimbal` never
  commands vertical movement at all (altitude stays the pilot's, same as takeoff).

### Box accuracy: background GrabCut re-fit (opt-in, `--box-refit`)

**Off by default.** In live use the re-fit grew boxes onto the surroundings (one went from 64x20 to 98x55), and fresh
locks were rejected within seconds. Use it only for slow, well-separated targets until that is solved.

CSRT keeps roughly the box it started with and slowly slides towards an edge or a shadow. Five times a second
a GrabCut cut runs **in the background** on a region 1.3x the tracked box. The answer is shifted by however far
the box moved while it ran. It is taken only when it is the same object: within half a box, size within
0.6–1.6x, and appearance not worse than CSRT's own box. Size moves halfway, or fully when two cuts agree or when
the cut fills its region (the object is growing as the drone approaches). It is skipped while a lookalike is near.


Measured in SITL against the red cube's true pixel extent:

| | median IoU | frames with IoU >= 0.7 |
|---|---|---|
| slow cube, before | 0.64 | 39% |
| slow cube, with re-fit | 0.81 | 74% |
| 4 m/s chase, before | 0.48 | 11–25% |
| 4 m/s chase, with re-fit | 0.6–0.7 | 25–53% |

What is left on the chase is mostly the cube's shadow on the bright runway, which GrabCut counts as part of the
object. Trimming shadows would also cut dark-clothed people and dark cars, so it is not done.

### FPV chase (opt-in, `--fpv`)

**Off by default.** Moving the drone hard right after a lock made fresh locks fragile in live use.

The drone flies with the target instead of only turning towards it. Along the line of sight the speed is the
target's own estimated ground speed x 0.7, plus a pull that holds `--fpv-standoff` metres (default 12, 1 m
dead-band). The pull is capped at `--fpv-approach` (4 m/s), and at 1.5 m/s while the target is under ~15 px.
Sideways target motion is followed by yaw, with feed-forward of the bearing rate from the target's world track.
`--fpv-lateral` also matches the sideways velocity: livelier, but it turns estimate noise into sideways lurches.
Commands ramp at 2.5 m/s². If the estimate drops out for a moment, the last chase command fades out over 1.5 s
instead of stopping dead.

What made it work (each measured against Gazebo's truth, red cube on a 10x12 m ellipse at 4 m/s):

- **The autopilot gimbal's angles are body-frame** for ArduPilot servo mounts (`--mount-frame body`). Read as
  earth-frame, the depression error was 0.94x the vehicle pitch: an accelerating drone saw the target twice as
  far away, the velocity estimate hit 8–12 m/s, and the chase ran away.
- **The video lags the telemetry by ~0.2 s** (`--video-latency`): the geometry uses the pose from when the frame
  was taken.
- **The sim gimbal reaches a commanded tilt within ~0.07 s**, so the 0.8 s "gimbal still" gate (it dropped two
  of three geometry fixes during a chase) is 0.15 s in the sim profile.
- **Smooth ramps beat hard ones.** With ArduPilot's `WP_ACC` 5 / `WP_JERK` 5 the drone pitched ±25–29° and threw
  the target out of the view. Keep the defaults (`WP_ACC` 2.5 m/s², `WP_JERK` 1).

Results (60 s runs, default profile):
- Started near (~10–25 m): held for the full 60 s, distance held at 10–11 m.
- Geometry error ~1.6–1.9 m.
- Velocity estimate 3.6–3.9 m/s for a true 4.0.
- Command steps p99 0.35 m/s per frame.
- Target ~60–75 px sideways from the image centre on the continuously circling cube (yaw lag).
- A start ~35 m away, with the cube ~12 px wide, is still lost within seconds: the box cannot shrink below
  ~16 px at half-resolution tracking, and the appearance check then fails.

On real hardware: the C12 reports its own earth-frame attitude, so `--mount-frame` does not apply to it. Measure
the real video latency, for example by turning the drone and watching when the picture moves. Start with
`--fpv-speed 5 --fpv-approach 2`, props-off and bench checks first.

### Same-looking objects: crossings and hidden targets

All the sim's walkers share one skin, so colour alone cannot tell two of them apart. `LockTracker` therefore also
uses *where the target can be*:

- **Crossing guard.** While another blob that would pass as the target is within about two box sizes of it,
  CSRT's box must stay on the path the target was on when the other one came close (its position then, plus its
  velocity then, over the whole crossing). CSRT slides onto the other person gradually, a few px per frame, so
  each single step looks fine. A box that leaves that path is not trusted; the tracker coasts on the path instead.
  The guard stays on for 8 frames after the other one leaves, because at full overlap it is not "next to" the
  target any more.
- **Search while hidden** covers everywhere the target can have walked since it was last *seen*: a circle that
  grows with its speed and the hidden time. A candidate is "close" when it is near either the straight-line
  prediction or the last-seen spot, so a person who stops or turns back behind a car is still found.
- **No coin tosses.** When two lookalikes are about equally good (same colours and about as close), nothing is
  re-acquired. The tracker keeps searching and, if it stays ambiguous, goes LOST for the operator to click again.
  A re-acquisition needs 2 consistent frames, or 3 when another lookalike is in view. It restarts the Kalman
  filter, because the target may now be moving another way.

Self-test: five scripted crossings of identical targets (other one in front or behind, various speeds), plus a
target that turns back behind a car with an identical person waiting beyond it. The previous logic swapped
targets in 4 of the 5 crossings and locked the waiting person. `TRK_DEBUG=1` prints `crowd`/`path` and
`AMBIGUOUS` per frame.

### GO = pursuit (default)

GO flies **straight at where the target is now**. Every frame it re-aims at the target's current position: the live track while
locked, the track predicted forward for up to 1.5 s while hidden. It flies at the GO speed and stops `--goto-standoff` (1 m) short
of the target on the straight line from the drone. It is never lower than `--alt-floor` above the target's ground. There is no
hover height and no pacing. If nothing fresh comes in for 3 s, it holds the last point. ABORT still brakes.

If the geometry fix happens to be missing in the frame GO is pressed, the track estimate from the last 1 s is used. Before this,
GO was refused with "no destination estimate yet".

Measured in SITL, red box moving ~1 m/s, start 12 m up and ~13 m away:
- Closest approach 1.2–1.6 m from the box centre, ending at ~1.7 m altitude (the floor).
- Arrival after 9–14 s.

Speed: the straight line descends ~10 m over ~13 m, and ArduPilot slows the whole line so that the descent stays within
`WP_SPD_DN` (default 1.5 m/s).
- With the default: top speed 2.2 m/s.
- With `WP_SPD_DN` 3.5 (sim only, then put back): 3.3–4.1 m/s.

To reach the GO speed on a steep line, raise `WP_SPD_DN` on the flight controller (your decision: fast descents have their own
risks), or start the GO from lower down.

After arrival the target is under the camera and usually lost. The drone then holds instead of chasing blind.

### Fast GO and ABORT

**How GO flies.** Velocity commands straight at the end point at `--rescue-speed`. Braking is computed from the drone's actual
distance, allowing ~0.5 s for the autopilot to respond. Two earlier designs were measured and dropped:
- A destination re-sent every frame made ArduPilot re-plan from near rest each time: ~1 m/s average on a "5–10 m/s" GO.
- A moving position+velocity setpoint: ArduPilot trailed it by ~1.5 s, so the drone surged and stalled, and overshot into the
  ground.

Velocity commands are only sent while the pursuit refreshes them (< 0.3 s old). After that the link sends a plain hold where the
drone is. ArduPilot itself zeroes guided velocity after `GUID_TIMEOUT`.

**Simulator speed limits.** `simulation/sim_up.sh --fast` loads `simulation/fast.parm` (sim only):

| Parameter | fast.parm | ArduPilot default |
|---|---|---|
| WP_SPD | 15 | 10 |
| WP_SPD_DN | 6 | 1.5 |
| WP_SPD_UP | 5 | 2.5 |
| WP_ACC | 5 | 2.5 |
| WP_ACC_Z | 3 | 1 |
| WP_JERK | 5 | 1 |
| ATC_ANGLE_MAX | 45 | 30 |
| PSC_NE_JERK | 15 | 5 |
| PSC_D_JERK | 10 | 5 |

Measured with 8 m/s GO, 15 m line from 12 m up:
- default limits: top 2.2 m/s.
- position setpoints + fast limits: top 5.2, ~2 m/s average.
- velocity commands: top 7.4 m/s, arrival in 8.6 s.
- velocity commands + tilt/jerk: top 6.8 m/s, arrival in **7.2 s**.

**ABORT** (window button, `x`, QGC "Rescue ABORT", stdin `abort`) **leaves the target and climbs**: whatever the drone is doing
(a GO dash, following, or holding after a GO), the GO and the follow loop stop at once. The drone climbs straight up
`--abort-climb` metres (default 10, never above `--alt-ceiling`) from where it is and holds there in GUIDED. The banner reads
"ABORTED - CLIMBING AWAY FROM THE TARGET". Nothing re-engages without a new LOCK/GO; the pilot takes over with a mode switch.
QGC's separate "BRAKE" action still brakes in place. The RC rescue switch going back to low keeps its old meaning.

Measured, ABORT from QGC 2.5 s into an 8 m/s GO at 6.2 m:
- It sank 2.3 m more while braking the descent, then climbed to 16.4 m and held in GUIDED.
- Before `WP_ACC_Z` 5 was added to the fast profile, it sank 4 m.

**Target lost → hover in the same place.** While following, a lost target no longer switches to LOITER. The follow loop goes
silent and the drone holds a position target right where it is, in GUIDED. Measured: within 6 cm for 8 s. A new lock releases
the hold. `--lost-loiter` restores the old hand-back to LOITER.

**QGC.** `tracker/qgc/RescueActions.json` must be in `~/Documents/QGroundControl/MavlinkActions/` (also copied to
`CustomActions/` for older QGC versions), and selected in QGC's settings (Fly View → MAVLink Actions). Tested with QGC's own ids
(system 255) through ArduPilot:
- LOCK / GO / ABORT / UNLOCK / BRAKE: all acknowledged and acted on.
- A tap on QGC's video (`CAMERA_TRACK_POINT`, camera component 100) now runs the same lock as a window click: 46 ms.

**Live telemetry line** at the bottom of the video: ground speed, climb rate, altitude, GUIDED or not, the GO speed, the abort
climb, and `[GO ACTIVE]` / `[HOLDING]`.

Measured (before the zero-velocity change): ABORT 3 s into an 8 m/s dash → `GO-TO aborted: BRAKE commanded`, vehicle in BRAKE.

`simulation/reset_drone.py` (sim only) stands a crashed drone upright, takes off again and climbs to 12 m.

The simulated red box is moved by teleporting it 10 times a second, so it can knock over a drone parked 1 m from it.

### Lock and tracking accuracy (2026-10-13, measured)

**Lock** (`tracker/tests/lock_bench.py`, 432 saved clicks):
- Red-box median IoU **0.93** (the previous lock path 0.51; 0.88 before the shadow trim); lock time ~40 ms warm.
- Shadow trim: object pixels with the ground's colour but darker are dropped. This is skipped if the click would be removed or
  less than 40% of the object would remain, so a black car is never erased.
- Snapped locks need a real object; a click on sky or runway is refused.
- Lock windows up to 480 px for large objects.

**Large objects:**
- Camera-motion estimation now leaves the target out. A car filling a third of the view had pulled the estimate (self-test:
  within 1 px of the true background motion).
- The appearance comparison runs on a ≤ 48 px copy (its cost had grown with the box size).

**CSRT size tracking** was disabled by our own speed setting: 17 scale steps instead of 33.
- With 17 steps, a textured target growing 56 → 116 px kept a 58 px box. With 33 steps: 112 px (IoU 0.93 at full, 0.80 at half
  resolution).
- Cost: +1.6 ms per frame.
- On very small targets (≤ 15 px at the tracking scale) CSRT still doesn't follow slow growth. Re-measuring the size with the
  ground model every 0.5 s helped one run but hurt two and doubled the prediction error, so it is not used.

**Kalman prediction** (`track_bench.py`): median 0.09x the target size from the true position (p90 0.23x). No setting moved it;
it is limited by the box itself.

### GO speed, stop point and the arrival hook
The defaults are in one place: the `GO SETTINGS -- EDIT HERE` block at the top of `tracker/main.cpp` (`kSpeed`, `kStopOffset`,
`kAccel`, and the limits `kMaxSpeed` / `kMaxOffset`). The stop offset is signed: `+10` = stop 10 m before the target, `0` = at it,
`-10` = fly over it and stop 10 m past it (in the direction GO started flying; the nose then points at the stop point, so it does
not spin round over the target). A fixed FPV camera loses the target once the drone is over it, so a past-the-target GO finishes
on the target's last track (a standing target) or holds after 3 s (a moving one).
- Per launch: `--rescue-speed S --go-offset M --goto-accel A`.
- Per GO: stdin `go SPEED OFFSET` (e.g. `go 15 -10`; 0 = configured), or QGC GO with param1 = speed, param2 = offset.
- The actions file has "GO 5 m/s, stop 3 m short" and "GO fast 8 m/s, at target" as editable examples.
- `on_go_arrived()` in main.cpp is called once on arrival and is deliberately empty: the operator's landing goes there.
- During GO the gimbal is aimed at the target's world position (up to 60°/s), and the expected box size follows the closing
  range, so the target stays in view during the dive.

## Known open items

- **Go-to horizontal accuracy: verified in SITL.** Box from the real camera frame, `--height-source home`: the
  drone stopped 0.45 m horizontally from the target and 3.2 m above the ground (hover height 3 m).
- **Losing the box at arrival: fixed.** CSRT often loses a very close target as a dash finishes. That used to
  trigger the auto-LOITER, and an unattended vehicle then sank. Now a completed dash keeps holding its end
  point in GUIDED and reports `Target lost at arrival: holding position` (verified in SITL: no mode change,
  altitude held). Mid-follow losses still switch to LOITER.
- **Relocking after or during a dash: fixed.** A new selection now cancels the old go-to, restarts the
  follow-velocity session and restarts the 2 s lock clock (verified in SITL: after a relock mid-dash the FC
  received only velocity targets, and an immediate GO was refused with "lock held 0.3s").
- **SITL's simulated rangefinder reads ~1.3 m short** (2.93 m at a true 4.31 m), so the sim profiles use
  `--height-source home`. On a real aircraft with a downward rangefinder, `auto` prefers it.
- **QGC bring-up**: the actions file and MAVLink handling exist, but nothing has been clicked through in a
  real QGC window.
- **Real hardware**: the C12's gimbal-angle reference frame (body vs earth; this is exactly `--camera-mount`),
  what it does on target loss, and the C12 thermal stream URL all still need a bench check before trusting
  them on a real aircraft. Bench-test the MAVLink link and RC failsafes with props off.
- **New, opt-in, simulator-tested only by unit tests so far**: `--follow-ff`, `--rescue-lead`,
  `--rescue-stay-over`, `--lock-without-guided`. Tune `--follow-ff` and the follow gains (still the
  untuned starting values) in `scenario_runner.py` before relying on them.
- **Not implemented** (decisions needed, not silently dropped): automatic re-acquisition after LOST (it
  conflicts with the hand-back-to-LOITER posture), MAVLink2 signing (needs keys provisioned on the FC and
  QGC too; `--gcs-sysid` is only a filter), showing the estimated target on QGC's map, obstacle awareness,
  Pi 5 codec/resolution retuning.
