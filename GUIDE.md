# FPV Tracking: A Plain-Language Guide

This guide explains, without jargon, what this project does, what each part is for, and exactly what to type to use it.
You do not need to know programming to follow it. Copy each command exactly as written.

---

## 1. What does this project do?

An FPV drone flies with a camera fixed to its frame. On a screen you see what the camera sees.

1. **You click on something** in the picture (a person, a car, a boat, a box: anything).
2. The program draws a **green box** around it. This is called **locking** onto the target.
3. The drone **keeps the target in view**: the camera cannot move on its own, so the drone itself turns, climbs and goes
   down to follow it.
4. When you press **GO** (once), the drone **flies quickly to the target** and stops there, or a few metres before or past
   it if you ask (see section 8). What happens next (landing, dropping a float, …) is your own code.
5. **ABORT** makes the drone leave the target and climb straight up to safety.
6. If the target is lost, the drone **stops and hovers** where it is. It never flies off on its own.

There is **no list of object types**: it works for anything you click, because it learns what *your* object looks like when you
click it.

---

## 2. The parts, in simple words

| Part | What it is | Where |
|---|---|---|
| **The tracker** | The main program. It watches the video, follows the target and sends flight commands. | `tracker/` (`target_tracker`) |
| **The flight controller** | The drone's own autopilot (ArduPilot on a Cube Orange). It actually flies the drone. | on the drone |
| **The simulator** | A pretend world on the laptop (Gazebo) with a pretend drone (ArduPilot SITL), for safe testing. | `simulation/` |
| **QGroundControl (QGC)** | The usual drone ground-station app. It shows the video and has LOCK / GO / ABORT buttons. | your laptop |

**How the target is found and followed** (all inside the tracker, nothing to set up):

- **Finding the box when you click:** the program looks at the colours *around* your click (the "ground") and finds the thing
  that is different from them. If that fails, it uses **GrabCut**, a well-known picture-cutting method. A moment later, GrabCut
  also stretches the box to cover the *whole* object (for example a truck's cab *and* its trailer).
- **Following it from frame to frame:** **CSRT**, a fast and reliable way of finding the same patch again in the next picture.
- **Predicting where it moves:** a **Kalman filter** (a maths tool that smooths and predicts motion). It runs two versions side
  by side (the **IMM**): one for steady movement, one for sudden turns, and uses whichever fits better.
- **Not confusing it with a lookalike:** **DINOv2** (only if you add `--reid`). It is a small AI model that recognises the
  object's *shape*, so a different object of the same colour is not mistaken for yours.
- **Flying to it with GO:** **Proportional Navigation (PN)**, the same principle interceptors use. The drone heads for where the
  target *will be*, not where it was, so it does not chase behind a moving target.

---

## 3. One-time setup (only the first time)

First install the programs it needs: see "Install and build" in [README.md](README.md) (one `apt` and one `pip`
command, plus the simulator if you want to practise on the laptop). Then open a terminal and type:

```bash
git clone https://github.com/maha-vidya/fpv-tracking.git ~/fpv-tracking
```

```bash
cd ~/fpv-tracking/tracker && scripts/build.sh
```

This downloads the code and builds the program. It takes a few minutes. Then check it works:

```bash
~/fpv-tracking/tracker/build/target_tracker --selftest
```

You should see `0 failed` at the end. If you see failures, stop and ask for help before flying.

---

## 4. Practising in the simulator (safe, on the laptop)

Do these steps in order. Each one uses its own terminal window (open a new tab for each).

**Step 1: start the pretend world.**

```bash
cd ~/fpv-tracking/simulation && ./sim_up.sh --fpv-camera --target moving --video-to 127.0.0.1:5603 --mav-to 127.0.0.1:14561
```

Wait until it says `DONE` (1–2 minutes). A window with a runway appears. `--fpv-camera` gives you the FPV drone: camera fixed
to the frame, FPV speed.

**Step 2: take off.**

```bash
cd ~/fpv-tracking/simulation && python3 takeoff.py --alt 12
```

The pretend drone rises to 12 metres and waits.

**Step 3 (optional): add traffic to make it harder.**

```bash
cd ~/fpv-tracking/simulation && python3 spawn_traffic.py --cars 0 --people 0 --vehicles 14 --no-roads
```

**Step 4: start the tracker.**

```bash
cd ~/fpv-tracking/tracker && ./build/target_tracker --profile sim-fpv --enable-rescue --video-out udp://127.0.0.1:5602
```

A window opens showing the drone's camera. **Click the red box** to lock onto it, then press **G** to GO. **Run only one tracker at a time**: two would fight over the drone.

**Step 5 (optional): open QGroundControl.** It connects by itself and shows the video and the action buttons.

---

### About the FPV drone

An FPV drone has its camera **bolted to the frame**: it cannot tilt or turn on its own. So the **drone itself** turns left/right
to keep the target in the middle, and climbs or goes down to keep it where it was in the picture when you locked. It also flies
at **FPV speed**: up to 30 m/s (108 km/h) while following, 25 m/s for GO, leaning up to 60°.

The camera looks 15° down: when the drone leans forward hard, the camera leans with it, and a steeper camera would lose a target
ahead out of the top of the picture. To try another angle, start the simulator with `FPV_CAM_PITCH=-20 ./sim_up.sh --fpv-camera ...`
and give the tracker the same number: `--cam-pitch-deg -20`. Do not use `--gimbal` together with `--fpv-camera`.
Slower: add `--ceil-vx 15 --rescue-speed 12` to the tracker command. The simulator's own limits are in `simulation/fpv.parm`.
(The older gimbal-camera drone is still available: `./sim_up.sh --gimbal --c12-camera --fast` with `--profile sim-c12cam`.)

On the **real** FPV drone use `--profile real-fpv`, and always give it the camera angle you **measured** on the frame
(`--cam-pitch-deg`, negative = looking down) and the lens's field of view (`--hfov-deg`). Wrong numbers make it steer wrongly.

---

## 5. Using it: the buttons and keys

**Click inside the tracker window first** so it hears your keys. Capital or small letters both work.

| To do this | Do this |
|---|---|
| Lock onto something | **Click it** once |
| Lock onto what is in the middle of the picture | Press **L** or the **Space bar** |
| Lock very precisely | **Drag** a box around it, then press **S** |
| Let go of the target | Press **U** |
| Fly to the target | Press **G** once |
| Leave the target and climb to safety | Press **X** |
| Show / hide the yellow "where it is going" box | Press **K** |
| Quit | Press **Q** |

In QGroundControl the same actions are buttons: **Track LOCK**, **Rescue GO** (press once), **Rescue ABORT**, **Track UNLOCK**,
**BRAKE**.

You can also type into the tracker's terminal, then Enter:
- `go 8 3`: fly to the target at 8 m/s and stop 3 metres **before** it
- `go 15 -10`: fly at 15 m/s, over the target, and stop 10 metres **past** it
- `go 0 -10`: the normal speed, stop 10 metres past it (0 = keep the normal value)
- `unlock`, `abort`

**What the colours on screen mean:**
- **Green box:** locked and following.
- **Orange / red box with a word:** the target is briefly hidden (`COASTING`) or lost (`LOST`).
- **Yellow box with an arrow:** where the target is expected to be in half a second.
- **Crosshair and "CENTRE off x,y":** how far the target is from the middle of the picture (green = good).

---

## 6. If something goes wrong in the simulator

| Problem | Fix |
|---|---|
| The pretend drone crashed or fell over | `cd ~/fpv-tracking/simulation && python3 reset_drone.py` |
| The tracker window shows no picture | Make sure only **one** tracker is running; restart the simulator (step 7) |
| "lock refused" | Click directly on the object, not beside it |
| The box went onto the wrong object | Press **U**, then click your object again |

---

## 7. Turning everything off

1. Press **Q** in the tracker window.
2. Press **Ctrl + C** in the traffic window (if you started it).
3. Stop the simulator:

```bash
cd ~/fpv-tracking/simulation && ./sim_down.sh
```

---

## 8. Useful settings (add them to the tracker command in step 4)

| Setting | What it changes |
|---|---|
| `--rescue-speed 15` | GO speed in metres per second (normal: 25) |
| `--go-offset 10` | Where GO stops: `10` = 10 m **before** the target, `-10` = 10 m **past** it, `0` = at it (normal: 0) |

To change the **normal** GO speed / stop point for good, edit the `GO SETTINGS -- EDIT HERE` block at the top of
`tracker/main.cpp` (`kSpeed`, `kStopOffset`, `kAccel`), then rebuild: `cmake --build ~/fpv-tracking/tracker/build -j8`.
| `--abort-climb 10` | How high ABORT climbs (metres) |
| `--ceil-vx 18` | Fastest the drone may fly while following (m/s) |
| `--reid` | Use DINOv2 so same-coloured lookalikes are not confused with your target |
| `--go-guidance pn` | How GO flies: `pn` (default, best for moving targets), `pursuit`, `lead`, `intercept` |
| `--no-speed-match` | Do not match the target's speed while following |
| `--zoom-max 2` | Allow digital zoom (off by default) |

---

## 9. On the real drone: Raspberry Pi 4 or 5?

- **Raspberry Pi 5: recommended.** Tracking runs at about 8–11 pictures per second (estimated), enough for people and vehicles.
- **Raspberry Pi 4: only for a target that stays still.** About 4–5 pictures per second. Add `--track-scale 0.33` and leave
  out `--reid`.
- These are estimates. Measure on the real board before relying on them.

---

## 10. Checking that everything still works (for whoever maintains it)

```bash
cd ~/fpv-tracking && ./run_all_tests.sh
```

This builds the program, runs its 429 self-checks and the recorded-video tests. With the simulator running and **no tracker
open**, `./run_all_tests.sh --sim` also flies the 16 simulator tests.

Logs of every flight are kept in `~/sim_run/tracker_logs/`: one line per video frame, plus a picture of every click.
