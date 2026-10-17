# Testing

Four levels, from fast and offline to the full simulator. `./run_all_tests.sh` runs them in order and stops at the first failure.

| Level | What | Needs | Time |
|---|---|---|---|
| 1. Build | `-Werror` build, exactly as CI | compiler, OpenCV | ~1 min |
| 2. Unit tests | `target_tracker --selftest` (466 checks) + all Python scripts compile | nothing | ~20 s |
| 3. Offline benchmarks | `tracker/tests/lock_bench.py` (click-to-lock) and `tracker/tests/track_bench.py` (tracking) on saved data | saved clicks / recordings | 1–5 min |
| 4. Simulator scenarios | `simulation/verify_suite.py` (16 scenarios + endurance) | `sim_up.sh` running, drone airborne, **no other tracker** | ~15 min (+10 endurance) |

```bash
./run_all_tests.sh                 # levels 1-3
./run_all_tests.sh --sim           # + the simulator scenarios
./run_all_tests.sh --sim --long    # + the 10-minute endurance run
```

## 2. Unit tests (`--selftest`)
Pure logic and the real CSRT on synthetic scenes, no vehicle or camera:
- Follow control and its limits, the state machine (LOCKED / COASTING / LOST / IDLE), frame gaps.
- Plausibility and appearance gates.
- Occlusion: a car passing over the target.
- Crossings of identical targets: 5 speed / front-behind combinations.
- A target turning back behind a car with a lookalike waiting.
- ReID: a same-colour person walks out from behind a car before the target does. Without ReID the lock takes it; with
  (stand-in) fingerprints the lock comes back on the target.
- Camera-motion estimation, including a large object filling 40% of the view.
- The Kalman filter, geometry and height references.
- GO (FPV Dash): direct 3D target coordinates (no safety floor), speed / braking law, moving target PN guidance, arrival rule.
- QGC STATUSTEXT, lat/lon.

## 3. Offline benchmarks
**Lock** (`lock_bench.py DIR`):
- Runs the live click-to-lock path on every `click_*_frame_X_Y.png` the tracker saved.
- Reports lock time, lock and refuse rates, large-object locks, and accuracy on the red box against its own pixels.
- Fails when the median IoU drops below 0.70 or the median lock time rises above 150 ms.
- `--legacy` runs the previous lock path, for A/B comparisons.

**Tracking** (`track_bench.py VIDEO`):
- Replays a recording through the exact tracking core (`target_tracker --replay`) from 5 start points.
- Scores every frame against the red box's pixels: tracked %, median IoU, frames on a WRONG object, LOST events, and the Kalman
  prediction error.
- Record new footage with `python3 simulation/record_camera.py OUT.avi SECONDS`. It is safe while a tracker runs, because it
  reads Gazebo directly.
- Experiments: environment variables reach the tracker: `CSRT_SCALE_LR`, `CSRT_PAD`, `CSRT_TF`, `CSRT_LR`, `CSRT_SCALES`,
  `CSRT_SCALE_STEP`, `CSRT_SEG`, `KF_R`, `KF_QP`, `KF_QV`.

## 4. FPV Flight Stability Test (`simulation/stability_test.py`)
Direct end-to-end telemetry and computer vision stability verification across all flight phases:
```bash
cd simulation && python3 stability_test.py
```
Measures:
- Airframe vibration / shake (deg/s rate jitter)
- Pitch and roll extremes (deg)
- Pitch reversals ("nods/min")
- Whole-picture frame-to-frame pixel jitter (px)
- Direct 3D target intercept with no artificial floor hold

Pass criteria: PASS on all four phases (`takeoff`, `hover`, `follow`, `go`).

## 5. Simulator scenarios (`verify_suite.py`)
Each scenario starts a fresh tracker, drives it the way an operator or QGC would, injects faults, and grades from the tracker's
own log and Gazebo's truth. The suite **refuses to start if another tracker is connected**, and marks any result **VOID** if one
joins during it: two trackers take the vehicle link and the video from each other.

| Scenario | Checks |
|---|---|
| `lock` | click → lock, box vs truth, 30 s on the moving box, centring, command limits, smoothness, zoom limits, unlock, logging, clean exit |
| `reject` | a click on empty sky is refused |
| `relock` | 8 lock/unlock cycles |
| `mode` | pilot mode change drops the lock; no relock on return to GUIDED |
| `linkloss` | 7 s MAVLink freeze → LINK_LOST |
| `vfreeze` | 5 s video freeze → no command from stale data |
| `dash` | FPV Dash: GO sent, direct 3D target intercept, ABORT during GO climbs away |
| `abortfollow` | ABORT while following climbs away, no relock |
| `losthover` | target lost → hovers in place (< 0.5 m) |
| `qgc` | QGC video tap, LOCK, UNLOCK, GO, ABORT, BRAKE (as QGC sends them, acknowledged) |
| `goto` | GO pursuit vs truth: closest distance, top speed, arrival |
| `gostop` | `go 5 0`: direct arrival on target without safety floor |
| `large` | a 4 m magenta truck: lock box and 30 s tracking vs its pixels |
| `occlusion` | the red box with the car driving over it: never on the car, re-acquired |
| `people` | how long a small walker stays locked |
| `--sustained 10` | 10 minutes: no crash, no slowdown |

Results go to `~/sim_run/verify/report.json`. The checklist status is kept in `tracker/docs/VERIFICATION.md`.

Scene traffic (`simulation/spawn_traffic.py`):
- `--shirts red,blue,...`: walkers in different sweater colours (ReID / identity tests).
- `--vehicles N`: cars, vans, buses, trucks and motorbikes in 12 colours, built from simple shapes (no download). They roam
  +/-`--vehicle-area` m (default 70) in any direction, at slow (1.5-4), medium (5-9) or fast (11-18 m/s) speeds, given in turn.
  A vehicles-only run (`--cars 0 --people 0 --vehicles 14`) can run beside a people/cars run.

Simulator-only helpers:
- `simulation/reset_drone.py`: stands a crashed drone up and flies it back to 12 m.
- `simulation/fast.parm` / `sim_up.sh --fast`: fast GO limits.

## Not testable in the simulator
Raspberry Pi 5 CPU, temperature and frame rate; the real C12 video latency and quality; the G20 RC override and backup;
flight-controller failsafes; real people's identity.
