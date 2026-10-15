# Feature verification (SITL) — 2026-10-13

**Setup:** Gazebo `iris_runway`, ArduPilot SITL, `sim_up.sh --gimbal --c12-camera` plus the fast limits (`simulation/fast.parm`),
tracker `--profile sim-c12cam` on the current build.

**How it was tested:**
- `simulation/verify_suite.py`: each test starts a fresh tracker, drives it through its command input / QGC's MAVLink commands,
  injects faults (`fault.sh`), and grades the result from logs and Gazebo truth.
- Offline: `target_tracker --selftest` (405 checks) and the lock benchmark on 320 saved operator clicks.

**Status values:**
- **PASS:** measured and met the criterion.
- **PASS\*:** measured once end-to-end today, on this code, outside the suite run.
- **FAIL:** measured and missed.
- **NOT RE-VERIFIED:** the suite run was voided by a second tracker connected at the same time (it takes the vehicle link *and*
  the video), and the behaviour changed since the last clean pass.
- **PENDING:** needs real hardware / QGC UI / RC.
- **N/A:** not part of this design.

Two suite runs today were voided part-way by a second tracker started during the run. The suite now detects this: the tracker
prints "the link keeps reconnecting -- is ANOTHER target_tracker running?".

## A. Target selection and lock
| Check | Status | Evidence |
|---|---|---|
| Detect target | N/A | By design: the operator clicks; no detector. |
| Manual target selection | PASS | Window click → `57x37 object (ground model) in 47 ms`. QGC video tap → locked in 46 ms (PASS\*). |
| Lock confirmation | PASS | `IDLE -> LOCKED` only after the tracker initialised; LOCKING… / refused + reason otherwise. |
| Tracking box | PASS | 30 s on the moving cube, box on it. |
| Moving-target lock | PASS (cube) | LOCKED 93% of frames, never LOST (previous run 90%: borderline). People: PARTIAL (held 20–45 s in earlier runs). |
| Target identity | PASS (synthetic) · PENDING (live identical people) | Self-tests: 5 crossings of identical targets + turn-back behind a car, no switch. Live identity switches need instance ground truth. |
| Tap latency compensation | PASS | Lock job 44–47 ms in the background, replayed through the 2.5 s frame history to the live frame. |
| Lock rejection | PASS (offline) · NOT RE-VERIFIED (live) | **Bug found and fixed today:** a click on empty sky snapped 88 px to the horizon line and locked a 94x29 strip. Snapped locks now require a real object. The same frame now gives `REFUSED: nothing stands out`. On 320 saved clicks: 277 lock (22 by snapping), 43 refused. |

## B. Centering and tracking quality
| Check | Status | Evidence |
|---|---|---|
| Horizontal centering | PASS | \|x error\| median 18–21 px, p90 25–26 px (1280 px wide). |
| Vertical centering | PASS | \|y error\| median 11–18 px, p90 20–21 px. |
| Centering stability | PASS | 2 yaw reversals/min. |
| Target remains in frame | PASS | Yaw + gimbal keep it within ~26 px. |
| Temporary occlusion | PASS (synthetic + earlier replays) | No switch to the occluder; re-acquired. |
| Target leaves frame → lost | PASS\* | 5 s video freeze → `Target lost: hovering here`, drift ≤ 6 cm over 8 s, GUIDED. Suite rerun voided. |
| Reacquisition | PASS (synthetic) | Only a unique match re-locks; ties are refused. |
| Unlock | PASS | → IDLE; all following frames zero command. |

## C. Camera and zoom
| Check | Status | Evidence |
|---|---|---|
| Zoom control (manual) | PENDING / not implemented | No pilot zoom command; the C12 optical zoom is not wired. |
| Zoom according to target size | PASS | 12 view decisions in 30 s. |
| Zoom limits | PASS | 1.00–1.25 (cap 2.0). |
| Tracker after zoom | PARTIAL | The box is carried with the predicted motion; zoom only while steady. Earlier, 1 rescue run in 3 lost the lock during approach + zoom. |
| Camera feed latency | MEASURED | Video ~0.2 s behind telemetry (SITL). C12 not measured. |
| Video quality | PENDING | Real camera. |

## D. Follow movement and FPV behavior
| Check | Status | Evidence |
|---|---|---|
| Forward/backward response | PASS | View control moves closer (vx > 0 toward the target). |
| Left/right response | N/A | Centring is by yaw (`--lateral-mode strafe` untested). |
| Turning response | PASS | See centering. |
| Vertical response | N/A (follow) · PASS (GO/ABORT) | Follow holds altitude (`--vz-mode hold`). GO descends to the floor (1.5 m); ABORT climbs +10 m (PASS\*). |
| Speed limits | PASS | Follow max \|vx\| 0.58 m/s, \|yaw\| 0.34 rad/s within 8 m/s / 1.2 rad/s. GO: ArduPilot `WP_SPD` 15 caps any `--rescue-speed`. |
| Smooth acceleration | PASS | Follow command step p99 0.08 per frame. |
| Target distance (GO standoff) | PASS\* | GO stops 1 m short: closest 1.07–1.36 m from the box centre. |
| Target speed changes | PARTIAL | GO re-aims every frame; the opt-in chase held a 4 m/s circling target in 2 of 3 runs. |
| FPV responsiveness | PASS (laptop) · PENDING (Pi 5) | 15.8 fps video, 24–29 ms frame→command, 20 Hz MAVLink. GO at 8 m/s: top 6.8–7.4 m/s, 15 m in 7.2 s (PASS\*). |
| Camera/drone coordination | PASS | Horizontal by yaw, vertical by gimbal tilt, distance by forward motion / GO. |

## E. Pilot controls and safety
| Check | Status | Evidence |
|---|---|---|
| Abort (= leave the target and climb) | PASS\* (during GO) · NOT RE-VERIFIED (during follow) | QGC ABORT 2.5 s into an 8 m/s GO at 6.2 m: sank 2.3 m while braking, then climbed to 16.4 m and held, GUIDED. |
| Abort during movement | PASS\* | As above. |
| Mode-change cancellation | PASS (logic unchanged since the last clean pass) · NOT RE-VERIFIED today | Last clean pass: pilot LOITER → `LOCKED -> IDLE (left GUIDED: lock dropped)`. |
| Pilot override | PASS (mode switch) · PENDING (RC) | |
| No automatic relock | PASS | No relock without a click after a mode change, and none after ABORT. |
| Link-loss handling | PASS | 7 s MAVLink freeze: `LINK_LOST`; ArduPilot `GUID_TIMEOUT` 3 s stops it. |
| Stale target protection | PASS | 0 stalled-video frames with a non-zero command. GO velocity commands expire after 0.3 s and become a hold (**fixed today:** a stale velocity once flew the sim drone into the ground). |
| RC backup | PENDING | |
| Arming and takeoff boundary | PASS (code) | The tracker never sends arm or takeoff. |
| Failsafe behavior | PENDING | Flight-controller configuration. |

## F. Go-to target (GO)
| Check | Status | Evidence |
|---|---|---|
| Destination selection | PASS\* | `GO-TO sent: 15–18 m straight at the target`. Suite rerun voided. |
| Destination validation | PASS | Refusals: `target not locked`, `no destination estimate yet` (now bridged by the 1 s track estimate). |
| Confirmation step | PASS | A single GO without a confirmation within 3 s → nothing sent (4/4). |
| Command acknowledgement | PASS | Every QGC command (LOCK, GO, ABORT, UNLOCK, BRAKE, video tap) answered with COMMAND_ACK 0. |
| Destination movement | PASS\* | Straight at the moving box at 8 m/s; closest 1.07–1.36 m. |
| Arrival detection | PASS\* | `GO-TO arrived: 1.0 m from the target`. |
| Abort during go-to | PASS\* | Climb-away, see E. |
| QGC go-to conflict | PENDING | Code cancels on a foreign position target; not tested. |
| No stale destination | PASS (design) | Re-aimed every frame; holds after 3 s without the target; a new lock cancels the old GO. |
| Rescue readiness checks | PASS | See Destination validation. |

## G. Reliability and performance
| Check | Status | Evidence |
|---|---|---|
| Sustained run (10 min) | PASS (2026-10-12) · NOT RE-VERIFIED today | Yesterday: 10 min locked, loop time flat at 5.2 ms. Today's run was voided after 0.5 min. |
| CPU and temperature | PENDING | Pi 5. |
| Tracker benchmark | PASS | Lock on 320 saved clicks: ~40 ms (warm). Red-box clicks: median IoU 0.72 (previous lock code 0.51). |
| Dropped-frame handling | PASS | 5 s video freeze: alive, safe. |
| Startup and shutdown | PASS | Clean exit in 0.3 s, return code 0. |
| Logging | PASS | Per-frame JSON + operator events + STATUSTEXT + click images. |
| Repeatability | PARTIAL | Lock/unlock 8/8; lock tests consistent across 2 runs; GO speed/arrival 3 runs; others voided. |

## Additional lock-correctness checks
| Check | Status | Evidence |
|---|---|---|
| Tap-coordinate accuracy | PASS (window, QGC tap normalised 0–1 → pixels) | QGC tap at (0.416, 0.608) locked the box under it. |
| Lock initialization validation | PASS (borderline) | First box vs truth IoU 0.51 (pass ≥ 0.5). The box includes the shadow (58x38 vs 33x34). The previous run failed at 0.41. |
| Multiple similar objects | PASS (synthetic) · PENDING (live) | |
| Overlapping objects | PASS (synthetic) | |
| Small and distant targets | MEASURED | Reliable from ~15 px; the box can't go below 20 px at half-resolution tracking. |
| Lock confirmation feedback | PASS | LOCKING… / LOCKED / TARGET HIDDEN / LOST / refused + reason; GO / ABORT banners; live telemetry line. |
| Arbitrary-object support | PASS | Boxes, cars, people (no class list). |
| Repeated lock/unlock | PASS | 8/8. |

## Bugs found and fixed by this verification
1. A sky click snapped to the horizon line and locked it → snapped locks now need a real object.
2. A stale GO velocity kept the drone moving (into the ground) → velocities expire after 0.3 s, then hold.
3. Every GO began with 1 s of competing zero commands → silenced at hand-over.
4. QGC had no LOCK/GO/ABORT buttons → actions file installed. QGC video taps used a fixed-size box → same lock as a click.

## Open
- Shadow included in the lock box (borderline IoU).
- Lock persistence through approach + zoom.
- People at 6–10 px.
- Re-run the voided tests with no other tracker connected: `python3 simulation/verify_suite.py mode linkloss vfreeze rescue abortfollow losthover qgc goto --sustained 10`
