# Drone Gazebo — vision-guided rescue drone (companion software + simulation)

**New here? Start with [GUIDE.md](GUIDE.md)** — a plain-language, step-by-step guide.

A companion-computer program that lets a rescue officer fly a drone manually, **select a target in the
video**, have the drone **track and follow** it automatically, and on command have the drone **fly fast,
directly to that target's real-world location** (a diagonal 3D line, not a slow level-then-drop) so the
officer can deliver a payload (e.g. a fire extinguisher) using their **own hardware switch wired straight
to the flight controller** — entirely outside this software. There is deliberately **no payload-release,
trigger, or "hold on arrival" logic anywhere in this code.** The drone computes where the target is and
flies there; nothing more. Runs the same program against a Gazebo/ArduPilot simulation and against the
real aircraft.

If you are new to this project, read this file top to bottom once, then go straight to
**[`tracker/docs/RESCUE.md`](tracker/docs/RESCUE.md)** for exact, copy-pasteable commands.

## 1. The big picture

```
 Officer (G20 remote + GR01 video link, or QGroundControl)
        │  manual flight, target selection (drag/tap), rescue arm/confirm/abort
        ▼
 Companion computer (Jetson Nano now, Raspberry Pi 5 later; "Gazebo + a laptop" in simulation)
   runs ONE program: tracker/main.cpp ("target_tracker")
        │  MAVLink (velocity setpoints while following, ONE position+speed command for the rescue dash)
        ▼
 Flight controller (Cube Orange, running ArduPilot Copter, GUIDED mode)
        │
        ▼
 Aircraft (quadrotor; camera = SkyDroid C12, a 3-axis gimbal, RTSP video + its own UDP tracking protocol)
```

**Why a single C++ file.** `tracker/main.cpp` is one ~2,400-line C++17 translation unit by deliberate
choice (per the project's own history): the previous iteration split things into headers/libraries and it
became harder to reason about on a resource-constrained companion board. Everything — MAVLink link,
camera capture, the CSRT tracker, the control laws, the rescue state machine, the SkyDroid C12 protocol
client, and a 290-check self-test — lives in this one file, organized into clearly commented sections.

## 2. What the program actually does (4 modules + QGC integration)

| # | Module | What it does |
|---|---|---|
| 1 | **Select / lock** | Operator drags a box (local window), taps/draws a rectangle in QGroundControl, or flips an RC switch that locks a box at the image centre. Requests GUIDED, then initialises the tracker. |
| 2 | **Track** | OpenCV CSRT on a downscaled frame copy, smoothed by a constant-velocity Kalman filter, guarded by a plausibility check (rejects a box that jumps too far, leaves the frame, or changes size too abruptly in one frame — CSRT can report "success" on a bad jump). A state machine (`LOCKED → COASTING → LOST`) decides when the lock is really gone. |
| 3 | **Follow** | Keeps the target centred and at the lock-time apparent size (image-based servoing), or — with the SkyDroid C12 — follows the gimbal's own tracking angles instead. Velocity commands are ceilinged (`--ceil-vx/vy/vz/yaw`, default 8/8/3 m/s, 1.2 rad/s — raise per mission for a faster target class), slew-rate limited, and gated to zero unless GUIDED, armed, linked, and actually locked. Optional `--best-view`: climbs if the target gets too close/large in frame, descends/approaches if too far/small — an autonomous altitude decision, off by default (see Safety posture). |
| 4 | **Go-to-target ("rescue")** | The officer arms (one press) then confirms (a second press within 3s). The program computes the target's real ground position from the camera geometry (bearing + depression angle + vehicle height, or the C12 gimbal's own angles) and sends **one** destination, paced along a straight 3D line so horizontal and vertical motion arrive together, stopping a configurable height **above** the target rather than on it. The dash can be aborted (switches to BRAKE) and keeps refining its endpoint from the live tracked box until it's committed. |
| — | **QGroundControl integration** | QGC can select a target (camera protocol v2: tap/rectangle/stop-tracking) and, via a Custom Action or joystick button mapped to `MAV_CMD_USER_1..4`, arm/confirm/abort/unlock/brake the rescue. Every notable event is sent to QGC as STATUSTEXT, not just this program's own console. If QGC's own "Go to location"/Orbit/ROI competes with an active dash, the program detects it and backs off instead of fighting it. |

### Safety posture (read this before anything else)

- **No release/trigger/payload logic in software, anywhere.** That decision was made explicitly after an
  earlier design (an autonomous "approach and strike" behaviour) was recognised as operationally
  resembling a loitering munition and was deliberately simplified away. The payload (e.g. extinguisher) is
  released by the officer's own hardware switch wired directly to the flight controller — this program
  never touches it.
- **No takeoff or altitude-control logic either, by default.** The program never arms, never requests a
  mode change to get airborne, never picks a climb height on its own — that is the pilot's job, both on
  the real aircraft and when testing in simulation (fly it through QGroundControl, not a script, when
  testing anything timing- or operator-sensitive). **The one deliberate, explicit exception**: `--best-view`
  (off by default) lets the follow loop climb/descend on its own to keep the target's apparent size in a
  healthy framing band — a genuine autonomous-altitude decision, scoped to this one opt-in flag rather than
  the default behavior, and still passed through `--alt-floor`/`--alt-ceiling` and the velocity ceilings
  like any other command. Without `--best-view`, the program only ever sends follow velocity commands or
  the one rescue go-to command, and only once GUIDED is already active.
- Hard velocity ceilings and slew limits on the follow loop; a direction-preserving clamp so saturating one
  axis doesn't bend the path. `--ceil-vx`/`--ceil-vy`/`--ceil-vz`/`--ceil-yaw` set the actual ceiling
  (default 8/8/3 m/s, 1.2 rad/s — enough for a car/bike-speed target, not just a walking one; raise further
  per mission for a faster target class). A 30 m/s / 5 rad/s sanity bound catches an obvious typo reaching
  the flight controller, but is not meant to be the real limit for any mission — pick the ceiling that
  actually fits the target you're following, there's no one-size-fits-all default.
- The rescue dash requires: locked ≥2s, GUIDED, armed, link up, GPS fix, **EKF healthy**, a plausible
  destination, and a deliberate two-step arm+confirm (a real 3-position RC switch, or two presses from the
  keyboard/stdin/QGC) within range — any one of those failing refuses it, with a reason.
- The dash stops a configurable height **above** the target (`--rescue-hover-height`, default 3m), not on
  top of it — the target can be a car, a person, a drone, anything selectable, so landing on it is not
  acceptable.
- Losing the lock sends STATUSTEXT to the pilot and switches to LOITER so they regain the sticks
  immediately — except during an active dash, where losing the box keeps the last good destination rather
  than aborting (see the "known open items" in `tracker/docs/RESCUE.md` for a real edge case found in
  testing: losing the box right as a dash *finishes* also triggers this, which is correct with a pilot on
  the sticks but worth a second look for the unattended case).
- A frozen/stale RC reading (radio dropout) is never read as a live switch position.
- **Everything above the flight-controller line is advisory.** Geofence, failsafes (RC/GCS/battery) and
  the actual safety authority live in ArduPilot's own parameters — this program does not replace them.

## 3. Hardware this targets

| Part | Role |
|---|---|
| **SkyDroid G20 + GR01** | The officer's RC transmitter + long-range digital video/telemetry link |
| **Cube Orange** (ArduPilot Copter, GUIDED mode) | Flight controller; the only thing with real safety authority |
| **SkyDroid C12** | 3-axis gimbal camera: RTSP video (1280×720 visible, thermal available) + its own UDP "TOP" protocol for gimbal push/track commands |
| **Companion computer** | Jetson Nano today; Raspberry Pi 5 planned (no CSI/TensorRT dependency remains — the program only needs OpenCV + GStreamer + MAVLink headers) |
| **QGroundControl** | The officer's operator display/control app, talking to the companion as a MAVLink camera component (compid 100) and to the vehicle as usual |

## 4. Repository layout

```
drone-gazebo/
├── tracker/                  the companion program (what actually flies the mission)
│   ├── main.cpp              single-TU C++17 source — MAVLink link, tracker, control laws, rescue FSM, C12 client, self-test
│   ├── CMakeLists.txt
│   ├── scripts/               build.sh, fetch_mavlink.sh, run_*_laptop.sh, run_tracker_jetson.sh, relay_video_to_jetson.sh
│   └── docs/RESCUE.md         <- START HERE for exact commands
└── simulation/                Python tooling: nothing here runs on the aircraft, it's all for testing
    ├── sim_up.sh/sim_down.sh/sim_status.sh    one-command Gazebo+SITL+MAVProxy+simulated-C12 stack
    ├── takeoff.py              arm + climb + hold in GUIDED
    ├── move_target.py          move one target box (+ a lookalike) in Gazebo
    ├── spawn_traffic.py        spawn N cars + M people, each on its own path at its own speed (fast/slow mix)
    ├── sim_events.py           inject RC/mode/failsafe events for testing
    ├── sim_c12.py              a simulated SkyDroid C12 (speaks the real TOP protocol) for the Gazebo gimbal
    ├── fault.sh, mav_gate.py, scenario_runner.py, serial_bridge.py    fault injection / link-loss / scenario harness
    ├── c12/                    the C12 "TOP" protocol library + a protocol-only fake camera, for bench testing without hardware
    └── phase0/                 the tracker-selection benchmark (why CSRT, not KCF/MIL/TLD or YOLO) + ArduPilot parameter notes
```

## 5. Why CSRT, and why no YOLO/detector here

An earlier version of this program used a YOLO object detector. It was deliberately replaced with
**class-agnostic, operator-selected tracking (CSRT)**: the officer decides what the target is by
dragging/tapping it, rather than the software guessing "this is a person" or "this is a car" — the
target can be anything the officer points at. `simulation/phase0/` holds the benchmark that chose CSRT
over KCF/MIL/TLD (CSRT: mean IoU 0.70-0.77, never lost track, on OTB sequences; the others each failed in
some tested case) and the ArduPilot parameter research behind the MAVLink design choices in `main.cpp`
(e.g. `MAV_CMD_DO_SET_MODE` not the deprecated `SET_MODE`, companion heartbeats only while the main loop
is alive, `GUID_TIMEOUT`).

A separate, earlier "approach and strike" behaviour (closing on a target until it fills the frame) exists
in this project's history but is **not included in this repository** — it was put on hold pending a
decision about its actual purpose, and this repository only contains the simplified "go to the target,
that's it" rescue behaviour described above.

## 6. Quick start

```bash
# one-time build
cd tracker && scripts/fetch_mavlink.sh
mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j"$(nproc)"
./target_tracker --selftest     # expect: 290 checks, 0 failed

# simulation (needs Gazebo Harmonic + ardupilot_gazebo + ArduPilot built separately, see below)
cd ../../simulation && ./sim_up.sh --target moving
# take off: fly it yourself through QGroundControl (the program has no takeoff/altitude logic of its
# own, by design — see "Safety posture" above), or for quick dev iteration: python3 takeoff.py
cd ../tracker && ./build/target_tracker --profile sim --enable-rescue
```

**Full step-by-step (every command run live while writing it, QGC controls, RC switches, traffic
scenarios, troubleshooting, and the current known-open-items list): see
[`tracker/docs/RESCUE.md`](tracker/docs/RESCUE.md).**

### External dependencies not vendored in this repo

- **ArduPilot** (built at `~/ardupilot` by default) and **ardupilot_gazebo** (built at
  `~/ardupilot_gazebo`) — needed only for simulation; a real aircraft just needs the Cube Orange flashed
  with stock ArduPilot Copter.
- **MAVLink C headers** — fetched by `tracker/scripts/fetch_mavlink.sh` (pinned revision of
  `mavlink/c_library_v2`), not committed.
- **OpenCV** with the `tracking` contrib module (for `cv::TrackerCSRT`), **GStreamer 1.0** (capture +
  video out), a C++17 compiler, CMake ≥3.10.
- **QGroundControl**, for the operator-interface parts (optional — the program works fully from the
  keyboard/stdin/RC without it).

Tests: see [TESTING.md](TESTING.md) (`./run_all_tests.sh`).

### Options added for real use and moving targets

| Option | What it does |
|---|---|
| `--camera-mount body\|gimbal`, `--cam-pitch-deg`, `--cam-yaw-deg` | Camera geometry for the target-position estimate and follow yaw. `gimbal` = stabilized (vehicle pitch not added). With a C12 attached its live angles are used; if they go stale, no destination is estimated. Real profiles default to `gimbal`. |
| `--vz-mode center\|hold` | `hold` stops the follow loop chasing the target with altitude (real profiles default to `hold`). |
| `--height-source auto\|home\|rangefinder` | Height above the target's ground: a downward rangefinder (DISTANCE_SENSOR) when fresh, else height above home. Shown in the ARMED preview as `h:...`. Sim profiles use `home`. |
| `--rescue-min-battery PCT` | Refuse to arm a dash below this battery % (default 30). |
| `--rescue-decel M/S2` | Dash slows to a stop over the last metres (default 2.0; 0 = old constant speed). |
| `--follow-ff 0..1` | Feed-forward of the target's estimated ground velocity while following (adds sideways motion). Off by default (and unused while `--fpv` chases). |
| `--fpv`, `--no-fpv` | FPV chase: fly with the target at its own speed and hold `--fpv-standoff` metres (default 12) instead of only turning. Off by default (opt-in, experimental). See RESCUE.md "FPV chase". |
| `--fpv-speed`, `--fpv-approach`, `--fpv-accel`, `--fpv-ff`, `--fpv-yaw-ff`, `--fpv-gain`, `--fpv-lateral` | Chase limits: top speed (10 m/s), approach/back-off speed on top of the target's (4 m/s; scaled down to 1.5 for small targets), command ramp (2.5 m/s²), velocity and yaw feed-forward gains (0.7, 0.8), distance gain (0.4), also match sideways velocity (off). |
| `--box-refit` | Background GrabCut re-fit of the tracked box (5 Hz). Off by default: it can grow the box onto the surroundings and lose fresh locks. |
| `--video-latency S` | How far the video lags the telemetry; the target geometry uses the vehicle pose from then (sim profile: 0.2, measured). |
| `--mount-frame body\|earth` | Frame of the autopilot gimbal's reported angles. ArduPilot SERVO mounts report body-frame demanded angles (`body`, sim profile); self-stabilising gimbals report earth frame (default). |
| `--gimbal-settle S`, `--est-q Q` | Gimbal stillness required before its angle is trusted (default 0.8 s, sim 0.15); target-track estimator process noise. |
| `--rescue-standoff M` (alias `--goto-standoff`) | GO stop distance: how far before the target's CURRENT position GO ends (default 0 = at the target; never below `--alt-floor` above its ground). Per GO: stdin `go SPEED DIST`, or QGC GO with param1 = speed, param2 = distance. Your landing goes in `on_go_arrived()` (main.cpp), called once on arrival. |
| `--goto-accel A` | GO: braking/acceleration rate the speed command plans with (default 4 m/s²). GO flies by velocity commands straight at the end point, braking from the drone's actual distance. |
| `--abort-climb M` | ABORT leaves the target and climbs this many metres from where it is (default 10), then holds in GUIDED. |
| `--lost-loiter` | A lost target switches to LOITER (old). Default: hover in the same place in GUIDED. |
| `--goto-paced` | The old GO: one paced line to the point at confirm time, ending `--rescue-hover-height` above it, then holding. |
| `--rescue-lead`, `--rescue-max-lead M` | Aim the dash where a moving target will be. Off by default. |
| `--rescue-stay-over` | After arrival, keep the hold point over a moving target. Off by default. |
| `--lock-without-guided` | Track-only: lock without taking the sticks; following starts when the pilot selects GUIDED. |
| `--gcs-sysid N` | Only this system id may send tracking/rescue commands (default 255; 0 = anyone, bench only). |
| `--record DIR` | Record the raw camera video (Matroska) for after-action review. |
| `--rtsp-out rtsp://0.0.0.0:8554/live` | Serve the annotated feed as RTSP (any number of viewers; built when `libgstrtspserver-1.0-dev` is installed). |
| `--detector-port 5810` | Use the detector/segmenter sidecar (`tracker/detector_sidecar.py`): click snapping, segmenter lock refinement, track re-anchoring, relock candidates. See RESCUE.md "Lock refinement". |
| `--seg-max-latency S`, `--seg-min-score X`, `--no-refine-lock` | Segmenter upgrade limits; disable all lock refinement. |
| `--lateral-mode yaw\|strafe`, `--klat`, `--ki-yaw`, `--ki-limit` | Centre the target by turning (default) or sliding sideways; optional bearing integral with anti-windup. |
| `--no-map-marker` | Turn off the `TARGET`/`DEST` icons on QGC's map. |
| `--rtsp-out-encoder auto\|hw` | Use the Jetson hardware H.264 encoder (default stays `sw`: the previous iteration found `nvv4l2h264enc` crashing the process on some L4T/GStreamer combinations; bench-test it with `gst-launch-1.0` on your board first). |

QGroundControl buttons for LOCK / GO / ABORT / UNLOCK / BRAKE, plus live `TARGET`/`DEST` map icons: copy `tracker/qgc/RescueActions.json` into
QGC's `MavlinkActions` folder (Linux: `~/Documents/QGroundControl/MavlinkActions`), see `tracker/docs/RESCUE.md` §6.

## 7. Real-hardware deployment

`--profile real-c12` (image-mode CSRT on the C12's own RTSP feed) or `--profile real-c12-gimbal`
(following the C12's own onboard tracking/gimbal angles instead) over the Cube Orange's TELEM2 UART. The
profile's *default* connection string (`serial:/dev/serial0:921600`) assumes a **Raspberry Pi** — found
live that `/dev/serial0` is a Pi-only `raspi-config` symlink and does not exist on Jetson, which fails with
"cannot open serial port /dev/serial0: No such file or directory" even with the Cube correctly wired. On a
Jetson, pass `--connection serial:/dev/ttyTHS1:921600` explicitly (verify the exact device for your board:
`ls /dev/ttyTHS* /dev/ttyUSB*` and `dmesg` on the Jetson) — `tracker/scripts/run_tracker_jetson.sh` now does
this automatically via `MAV_SERIAL` (defaults to `/dev/ttyTHS1`, override for your board/a Pi). **Before any real
flight:** confirm the C12's gimbal-angle reference frame (body- vs earth-relative — currently unconfirmed;
it maps directly onto `--camera-mount body|gimbal`, the profiles assume `gimbal`), what it reports when it loses a target, and bench-test
the MAVLink link and RC failsafes with props off. None of that has been done yet — see
`tracker/docs/RESCUE.md`'s "known open items" for the full, current list of what is and isn't verified.
