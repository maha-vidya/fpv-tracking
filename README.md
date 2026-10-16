# FPV Tracking — click a target, the drone follows it and flies to it

Companion-computer software for an **FPV drone with a fixed camera (no gimbal)**:

1. **Click** anything in the video (a person, a car, a boat, a box) — the program **locks** onto it.
2. The drone **follows** it: it turns, climbs and descends by itself to keep the target in view.
3. Press **GO** once — the drone **flies fast, straight to the target** and stops where you choose
   (on it, some metres before it, or some metres past it).
4. **ABORT** makes it leave the target and climb to safety. A lost target makes it stop and hover.

The same program runs in a **simulator** on your laptop (Gazebo + ArduPilot) and on the **real drone**.

> New to this? Read [GUIDE.md](GUIDE.md) — the same instructions in plain language, no programming needed.

---

## Contents

1. [How it works](#1-how-it-works)
2. [What you need](#2-what-you-need)
3. [Install and build (one time)](#3-install-and-build-one-time)
4. [Run it in the simulator](#4-run-it-in-the-simulator)
5. [Controls](#5-controls)
6. [GO: change the speed and where it stops](#6-go-change-the-speed-and-where-it-stops)
7. [Run it on the real drone](#7-run-it-on-the-real-drone)
8. [Problems and fixes](#8-problems-and-fixes)
9. [Tests](#9-tests)
10. [What is in this repository](#10-what-is-in-this-repository)
11. [Safety](#11-safety)

---

## 1. How it works

```
 You (laptop / QGroundControl / RC switch)
        │  click a target, GO, ABORT
        ▼
 Companion computer (Raspberry Pi 5 / Jetson; the laptop in simulation)
   runs ONE program: tracker/main.cpp  ->  "target_tracker"
        │  MAVLink: velocity commands while following, the GO dash
        ▼
 Flight controller (ArduPilot Copter, GUIDED mode)  ->  the drone
```

- **Finding the target:** when you click, the program learns what the object looks like (the colours around your
  click are "ground", the object is what differs), then **CSRT** tracks it frame to frame. A **Kalman filter** predicts
  where it moves; optional **DINOv2** (`--reid`) stops it from swapping onto a lookalike.
- **Following with a fixed camera:** the camera is bolted to the frame, so the drone itself does the pointing — it yaws
  to centre the target sideways and climbs/descends to keep it where it was when you locked. The drone's own tilt is
  removed from the calculation, so leaning forward to speed up does not make it climb by mistake.
- **GO:** the target's real position on the ground is computed from the camera angle and the drone's height, and the
  drone flies straight at it at FPV speed (up to 25–30 m/s), re-aiming every frame so a moving target is chased.

Everything — tracking, control, GO, MAVLink — is in one file, [`tracker/main.cpp`](tracker/main.cpp), on purpose.
Search it for `[GO-` to find every part of GO; the top of the file explains how GO works.

---

## 2. What you need

**For the simulator (laptop):**

| | Tested with |
|---|---|
| Ubuntu | 22.04 (24.04 works too) |
| C++ compiler, CMake ≥ 3.10 | g++ 11, CMake 3.22 |
| OpenCV **with contrib** (`tracking` module) | 4.5.4 (`libopencv-contrib-dev`) |
| GStreamer 1.0 | 1.20 |
| Gazebo Harmonic | 8.x |
| ArduPilot (built for SITL) | in `~/ardupilot` |
| ardupilot_gazebo plugin | in `~/ardupilot_gazebo` |
| Python 3 + `pymavlink`, `MAVProxy` | |
| QGroundControl (optional) | for the buttons and the map |

**For the real drone:** an FPV quad with an ArduPilot flight controller, a camera fixed to the frame, a companion
computer (Raspberry Pi 5 recommended) connected to the flight controller's TELEM port, and a way to get the video into
the companion (USB capture, CSI, RTSP or UDP).

---

## 3. Install and build (one time)

**1. System packages**

```bash
sudo apt update && sudo apt install -y git cmake g++ pkg-config python3-pip \
  libopencv-dev libopencv-contrib-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav
```

```bash
pip3 install --user pymavlink MAVProxy numpy
```

Optional, for `--reid` (DINOv2 inside the tracker): `pip3 install --user openvino` before building.

**2. Simulator (skip if you only fly the real drone):** install
[Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu),
[ArduPilot SITL](https://ardupilot.org/dev/docs/building-setup-linux.html) into `~/ardupilot`, and
[ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) into `~/ardupilot_gazebo` (follow each project's
own instructions). Other locations: set `ARDUPILOT_DIR` / `ARDUPILOT_GAZEBO_DIR`.

**3. Get this code and build it**

```bash
git clone https://github.com/maha-vidya/fpv-tracking.git ~/fpv-tracking
```

```bash
cd ~/fpv-tracking/tracker && scripts/build.sh
```

`build.sh` downloads the MAVLink headers (pinned version) and builds `tracker/build/target_tracker`.

**4. Check it works**

```bash
~/fpv-tracking/tracker/build/target_tracker --selftest
```

The last line must say `0 failed`. If not, do not fly — see [Problems and fixes](#8-problems-and-fixes).

---

## 4. Run it in the simulator

Use **one terminal per step** and keep each one open.

**Step 1 — start the simulated world and drone** (wait for `DONE`, 1–2 minutes; a Gazebo window opens):

```bash
cd ~/fpv-tracking/simulation && ./sim_up.sh --fpv-camera --target moving --video-to 127.0.0.1:5603 --mav-to 127.0.0.1:14561
```

- `--fpv-camera` = the FPV drone: camera fixed to the frame (15° down), FPV speed limits (`simulation/fpv.parm`).
- `--target moving` = a red box (plus an orange lookalike) moving side to side. `static` = standing still.
- `--video-to` / `--mav-to` keep everything on the laptop (the defaults send a copy to a Jetson's IP).

**Step 2 — take off** (the drone climbs to 12 m and waits):

```bash
cd ~/fpv-tracking/simulation && python3 takeoff.py --alt 12
```

**Step 3 (optional) — add traffic** (moving cars and vehicles, to make it harder):

```bash
cd ~/fpv-tracking/simulation && python3 spawn_traffic.py --cars 0 --people 0 --vehicles 14 --no-roads
```

**Step 4 — start the tracker** (a window opens with the drone's camera):

```bash
cd ~/fpv-tracking/tracker && ./build/target_tracker --profile sim-fpv --enable-rescue --video-out udp://127.0.0.1:5602
```

Now **click the red box** in the tracker window — a green box appears and the drone follows it. Press **G** to GO.

**Step 5 (optional) — QGroundControl:** open it; it connects by itself. For the GO / ABORT / LOCK buttons see
[QGroundControl buttons](#qgroundcontrol-buttons).

**Turn everything off:** press **Q** in the tracker window, **Ctrl+C** in the traffic terminal, then:

```bash
cd ~/fpv-tracking/simulation && ./sim_down.sh
```

> Always run `sim_down.sh` before starting `sim_up.sh` again. Two simulators running at once make objects jump
> back and forth and send every message twice.

---

## 5. Controls

**Click inside the tracker window first** so it receives your keys.

| To do this | Do this |
|---|---|
| Lock onto something | **Click** it |
| Lock onto what is in the middle of the picture | **L** or **Space** |
| Lock very precisely | **Drag** a box around it, then **S** |
| Let go of the target | **U** |
| **GO** — fly to the target | **G** (one press) |
| ABORT — leave the target and climb to safety | **X** |
| Show / hide the yellow "where it is going" box | **K** |
| Quit | **Q** |

**Typed commands** (in the tracker's terminal, then Enter): `go`, `go 15 -10` (see [section 6](#6-go-change-the-speed-and-where-it-stops)),
`abort`, `unlock`, `quit`.

**On screen:** green box = locked and following · orange/red with `COASTING` / `LOST` = target hidden / lost ·
yellow box = predicted position · bottom line = drone speed, height, and the GO speed / stop point in force.

### QGroundControl buttons

Copy [`tracker/qgc/RescueActions.json`](tracker/qgc/RescueActions.json) into QGC's `MavlinkActions` folder
(Linux: `~/Documents/QGroundControl/MavlinkActions`) and restart QGC. You get **Track LOCK**, **GO** (several
variants), **ABORT**, **Track UNLOCK** and **BRAKE**. Edit the file to make your own GO buttons
(`param1` = speed, `param2` = stop offset).

### RC switch (optional)

`--rc-rescue-channel N`: a switch on your transmitter. Flick it **up** = GO; back to **low** during the dash = stop
(BRAKE).

---

## 6. GO: change the speed and where it stops

**Where it stops** is a signed offset, measured along the line GO flies:

| Offset | GO stops… |
|---|---|
| `10` | 10 m **before** the target |
| `0` | **at** the target (default) |
| `-10` | **past** the target: flies over it and stops 10 m beyond |

The stop point follows a moving target, and is never lower than `--alt-floor` above the ground. What happens when it
arrives (landing, a release…) is your code: put it in `on_go_arrived()` in `tracker/main.cpp`.

**Three ways to change speed and stop point** — pick whichever suits you:

1. **Just this one GO** — type in the tracker's terminal:
   ```
   go 15 -10
   ```
   = 15 m/s, stop 10 m past the target. `go 0 10` = normal speed, stop 10 m before. (In QGC: GO with
   `param1` = speed, `param2` = offset.)
2. **For this run** — add to the tracker command:
   ```
   --rescue-speed 15 --go-offset -10
   ```
3. **Permanently** — edit the **`GO SETTINGS -- EDIT HERE`** block at the top of `tracker/main.cpp`:
   ```cpp
   constexpr double kSpeed      = 25.0;   // GO speed, m/s
   constexpr double kStopOffset = 0.0;    // +10 before the target, 0 at it, -10 past it
   constexpr double kAccel      = 8.0;    // braking, m/s²
   ```
   then rebuild: `cd ~/fpv-tracking/tracker && cmake --build build -j`.

> ArduPilot also limits the speed (`WP_SPD`, 30 m/s in `simulation/fpv.parm`). To fly GO faster than that, raise it
> there too (and on the real drone, in its parameters).

**By default GO goes on one press.** Want a safety confirm? Start the tracker with `--go-confirm`: then the first press
shows the distance and ETA, and a second press within 3 s confirms.

**Before GO is accepted**, all of these must be true (otherwise it tells you which one failed): target locked for ≥ 2 s,
GUIDED mode, armed, link OK, GPS fix, EKF healthy, battery above the minimum, target within `--rescue-max-range`.

---

## 7. Run it on the real drone

```bash
./build/target_tracker --profile real-fpv --enable-rescue \
  --connection serial:/dev/ttyAMA0:921600 --source 0 \
  --cam-pitch-deg -15 --hfov-deg 100
```

Change these to match **your** drone — wrong numbers make it steer wrongly:

| Option | What to put |
|---|---|
| `--connection` | The serial port wired to the flight controller's TELEM port, and its baud rate. Raspberry Pi: `/dev/serial0` or `/dev/ttyAMA0`; Jetson: `/dev/ttyTHS1`; USB adapter: `/dev/ttyUSB0`. |
| `--source` | The video device number (`0` = `/dev/video0`). For other video links: `--camera-source csi`, `rtsp --rtsp-url URL`, or `gst --gst-udp-port PORT`. |
| `--cam-pitch-deg` | The camera's **measured** tilt on the frame: negative = looking down (e.g. `-15`), positive = up. |
| `--hfov-deg` | The lens's horizontal field of view in degrees. |

**On the flight controller (ArduPilot):** set the TELEM port to MAVLink 2 at the same baud rate, and choose the speed
limits for your airframe (`WP_SPD`, `WP_ACC`, `ATC_ANGLE_MAX`, `LOIT_SPEED_MS` — `simulation/fpv.parm` is the simulator's
version, not a recommendation for your drone). Set up the geofence and failsafes (RC, GCS, battery) — they are the real
safety net, not this program.

**Before the first real flight:** test with **props off** that the link works, that following moves the sticks/motors the
right way, and that switching out of GUIDED gives you control back immediately.

Companion computer speed: Raspberry Pi 5 tracks at about 8–11 frames/s (enough for people and vehicles). A Raspberry
Pi 4 manages about 4–5 frames/s: add `--track-scale 0.33` and leave out `--reid`.

---

## 8. Problems and fixes

| Problem | Fix |
|---|---|
| `--selftest` shows failures | Don't fly. Rebuild (`scripts/build.sh`); if it still fails, report the `FAIL` lines. |
| Build: `MAVLink C headers not found` | Run `tracker/scripts/fetch_mavlink.sh` (needs internet). |
| Build: `opencv2/tracking.hpp` not found | Install `libopencv-contrib-dev`. |
| `sim_up.sh`: "pids exists: run sim_down.sh first" | Run `./sim_down.sh`, then start again. |
| Objects in Gazebo jump back and forth | Two simulators are running. `./sim_down.sh` (and `RUN_DIR=<other run dir> ./sim_down.sh` for an old one). |
| The tracker window shows no picture | Only **one** tracker may run. Close the others; restart the simulator. |
| "lock refused" | Click directly on the object, not beside it. |
| The box jumped onto the wrong object | Press **U**, click your object again. Try `--reid`. |
| GO refused: "…" | The message says which check failed (not locked long enough, not GUIDED, no GPS…). |
| `'g' has no effect: rescue is disabled` | Start the tracker with `--enable-rescue`. |
| The simulated drone crashed / fell over | `cd ~/fpv-tracking/simulation && python3 reset_drone.py` |
| Real drone: "cannot open serial port" | Wrong device name — check `ls /dev/tty*` and pass `--connection serial:/dev/<port>:921600`. |

---

## 9. Tests

```bash
cd ~/fpv-tracking && ./run_all_tests.sh
```

Builds with warnings-as-errors, runs the 456 self-test checks (control, GO, geometry, tracking) and compiles every Python
script — no simulator needed. With the simulator running and the drone in the air: `./run_all_tests.sh --sim` adds the
flight scenarios. Details: [TESTING.md](TESTING.md).

---

## 10. What is in this repository

```
fpv-tracking/
├── README.md                this file
├── GUIDE.md                 plain-language guide
├── TESTING.md               how to test
├── run_all_tests.sh         every test level in order
├── tracker/                 the program that flies the mission
│   ├── main.cpp             everything: tracking, follow control, GO, MAVLink, self-test
│   │                        (top of file: how GO works + the GO SETTINGS block)
│   ├── native_ai.hpp        optional DINOv2 lookalike check (OpenVINO)
│   ├── detector_sidecar.py  the same check as a Python helper, for builds without OpenVINO
│   ├── scripts/             build.sh, fetch_mavlink.sh, run scripts for laptop / Jetson
│   ├── qgc/RescueActions.json   QGroundControl buttons
│   ├── tests/               offline benchmarks (lock and tracking on recorded clips)
│   └── docs/                RESCUE.md (detailed reference), VERIFICATION.md
└── simulation/              testing tools only — nothing here runs on the drone
    ├── sim_up.sh / sim_down.sh / sim_status.sh   start / stop / check the whole simulator
    ├── fpv.parm, fast.parm  ArduPilot speed limits for the simulator
    ├── takeoff.py, reset_drone.py               take off / recover the simulated drone
    ├── move_target.py, spawn_traffic.py         moving targets and traffic
    ├── verify_suite.py, scenario_runner.py      automatic flight tests
    └── c12/, phase0/        older gimbal-camera tools and the tracker benchmark
```

The program also still supports a gimbal camera (SkyDroid C12: `--profile real-c12`, `sim-c12cam`); the FPV profiles
don't use it. All options: `tracker/docs/RESCUE.md`.

---

## 11. Safety

- **The flight controller is in charge.** Geofence, failsafes and the pilot's mode switch always win. Switching out of
  GUIDED gives the pilot control immediately.
- **No takeoff, arming or release logic** in this program. It never arms the drone and never drops anything; what happens
  on arrival is your own code (`on_go_arrived()`).
- Following is gated: commands are sent only while GUIDED, armed, linked and actually locked; speeds are capped.
- A lost target → the drone stops and hovers. ABORT → it climbs away and holds.
- FPV speeds (25–30 m/s) are fast. Test in the simulator first, then on the real drone with low speeds
  (`--rescue-speed 5 --ceil-vx 5`) before raising them.
