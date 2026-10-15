# Cube Orange parameter checklist (Phase 0, item 6)

**Status: a checklist for review, not a parameter file to load blindly.** Values marked *site* depend on the field, the
battery and the airframe. Names come from the **ArduPilot 4.8-dev** metadata on the development laptop; the Cube Orange will
probably run a **stable release**, where several names differ (table 1). Check every name in QGC/Mission Planner's parameter
search before setting anything, and test every failsafe on the bench with propellers removed.

Companion in the **simulation phase** = Jetson Nano over UDP (MAVProxy output); in the **real drone** = Raspberry Pi over a UART.
The parameters below apply to both; the link parameters (section A) only to the UART.

## 1. Names that changed in 4.8-dev (old stable name → new name)

| Old (4.5/4.6 stable, to my knowledge) | 4.8-dev | Note |
|---|---|---|
| `SYSID_MYGCS` | `MAV_GCS_SYSID` (+ `MAV_GCS_SYSID_HI`) | ids accepted as "the ground station": GCS failsafe, RC overrides, manual control |
| `SYSID_THISMAV` | `MAV_SYSID` | vehicle system id |
| `ANGLE_MAX` (centi-deg) | `ATC_ANGLE_MAX` (deg) | maximum lean angle |
| `RTL_ALT` (cm), `RTL_SPEED` (cm/s) | `RTL_ALT_M` (m), `RTL_SPEED_MS` (m/s) | **units changed** |
| `PILOT_SPEED_UP/DN` (cm/s) | `PILOT_SPD_UP/DN` (m/s) | units changed |
| `ARMING_CHECK` (checks to **run**) | `ARMING_SKIPCHK` (checks to **skip**) | logic inverted; 0 = nothing skipped |

## 2. Verified in SITL (ArduPilot 4.8-dev, GPS enabled, 2026-10-05) – reproducible with `sitl_tests/`

| # | Finding | Evidence |
|---|---|---|
| 1 | Velocity commands in the body frame are followed (≈2 m/s after 5 s); zero-velocity commands stop in ≈2 s (≈1.8 m) | `guided_takeback_test.py` |
| 2 | If the companion dies at 2 m/s the vehicle keeps its speed for `GUID_TIMEOUT`, then stops and holds: **7.7 m with 3 s (default), 3.7 m with 1 s** | `guided_takeback_test.py`, `guided_timeout_and_gcs_failsafe_test.py` |
| 3 | After the companion switched to GUIDED over MAVLink, the mode is **not** reverted by the RC switch; moving the switch to another position regains control | `guided_takeback_test.py` |
| 4 | **A heartbeat from a system id inside the GCS range (default 255) resets the GCS-failsafe timer**: with the real GCS silent and the companion heartbeating as 255, no failsafe in 8 s | `guided_timeout_and_gcs_failsafe_test.py` |
| 5 | Companion heartbeating as a different id (190) or as **sysid 1 / compid 191**: GCS failsafe fires after 2.1–2.6 s (`FS_GCS_TIMEOUT=2`) and the mode becomes Land | same + `companion_identity_test.py` |
| 6 | A companion using sysid 1 / compid 191 can send `SET_POSITION_TARGET_LOCAL_NED` and `DO_SET_MODE` (accepted) | `companion_identity_test.py` |
| 7 | **`WP_SPD` does NOT cap Guided velocity setpoints**: commanded 4 m/s with `WP_SPD=2` → peak 4.04 m/s | `companion_identity_test.py` |
| 8 | RC overrides are only honoured from the GCS system id (earlier test mistake); an armed vehicle in GUIDED with no takeoff command auto-disarms after ≈10 s (`DISARM_DELAY`) | observed |

Consequences: **our own code must clamp speeds** (the FC will not); the companion must **never** use system id 255 / the GCS range;
use `GUID_TIMEOUT` ≈ 1 s with commands streamed at ≥ 10 Hz.

## 3. A. Companion link (Raspberry Pi on a Cube UART; ArduPilot Pi guide, MAVLink 2)

| Parameter | Value | Why |
|---|---|---|
| `SERIAL2_PROTOCOL` | 2 (MAVLink2) | TELEM2 to the Pi. Reboot required. |
| `SERIAL2_BAUD` | 921 (921600) | documented value for a Pi companion |
| `MAV2_RC_CHAN` | 10 (Hz) | `RC_CHANNELS` for the hit channel and stick-takeover detection |
| `MAV2_POSITION` | 10 | `LOCAL_POSITION_NED`, `GLOBAL_POSITION_INT` |
| `MAV2_EXTRA1` | 10 | `ATTITUDE` (needed for camera tilt/roll compensation) |
| `MAV2_EXT_STAT` | 2 | `SYS_STATUS` (battery), `GPS_RAW_INT` |
| `MAV2_EXTRA2`, others | 0 | not needed |
| `MAV2_OPTIONS` | 0 | alternative: bit 4 *Unicast* + the Pi requests messages with `MAV_CMD_SET_MESSAGE_INTERVAL` (documented); both work, pick one |
| `MAV_SYSID` | 1 | the vehicle |
| `MAV_GCS_SYSID` | 255 (default) | the pilot's QGC/G20. **Companion must not use ids in this range.** |
| Companion identity | **sysid 1, compid 191** (`MAV_COMP_ID_ONBOARD_COMPUTER`) | verified (rows 5–6) – failsafe stays intact, commands accepted |
| Companion **heartbeat** | send at 1 Hz | ArduPilot forwards only to channels it has heard from (needed for the camera component). The existing C++ app sends none. |

Wiring: Cube TELEM2 TX/RX are 3.3 V (Cube docs), the same level as the Pi's GPIO UART: TX→RX, RX→TX, common ground.

## 4. B. Guided control and limits

| Parameter | Value | Why |
|---|---|---|
| `GUID_TIMEOUT` | **1.0** s (range 0.1–5) | dead companion = 3.7 m runaway at 2 m/s instead of 7.7 m (rows 2). Needs commands at ≥ 10 Hz. |
| `ATC_ANGLE_MAX` | *site/airframe* (e.g. 20–25°) | caps lean angle, hence acceleration |
| `WP_SPD_UP/DN`, `WP_SPD`, `WP_ACC` | *do not rely on them* | row 7: `WP_SPD` does not cap guided velocity; **clamp in code** (not yet tested for `WP_SPD_UP/DN`) |
| `GUID_OPTIONS` | 0 | bits: 0 allow arming from transmitter, 2 ignore pilot yaw, … leave default |

## 5. C. Failsafes (GPS available) – ArduPilot Copter docs: GCS failsafe, radio failsafe, battery failsafe, geofence

| Parameter | Suggested value | Notes |
|---|---|---|
| `FS_THR_ENABLE` | 1 (RTL) or 4 (SmartRTL, else RTL) | radio loss |
| `FS_THR_VALUE` | *site*: ≈ 50 µs below the lowest throttle PWM your G20/GR01 outputs | check receiver failsafe setting on the GR01 |
| `FS_GCS_ENABLE` | 1 (RTL), or 3/4 (SmartRTL…) | triggers when no heartbeat from the GCS id for `FS_GCS_TIMEOUT`; stays inactive if no GCS was ever connected (docs) |
| `FS_GCS_TIMEOUT` | 5 (default) – 2 | measured: 2 s → fires after 2.1–2.6 s. Shorter = more false triggers on a noisy link. |
| `FS_OPTIONS` | 0 | **keep bit 2 clear** ("continue if in Guided on RC failsafe"); bit 4 only affects pilot-controlled modes |
| `BATT_MONITOR` | per power module (e.g. 4 analog V+I) | verify against the hardware |
| `BATT_LOW_VOLT`/`BATT_CRT_VOLT` or `BATT_LOW_MAH`/`BATT_CRT_MAH` | *battery* | |
| `BATT_FS_LOW_ACT` / `BATT_FS_CRT_ACT` | 2 (RTL) / 1 (Land) | |
| `FS_EKF_ACTION`, `FS_EKF_THRESH` | 1 (Land), 0.8 | defaults |
| `FENCE_ENABLE` | 1 | |
| `FENCE_TYPE` | 3 (max altitude + circle); +8 for a minimum altitude | bitmask: 0 max alt, 1 circle, 2 inclusion/exclusion, 3 min alt |
| `FENCE_ACTION` | 1 (RTL or Land) or 3 (SmartRTL…) | |
| `FENCE_ALT_MAX`, `FENCE_RADIUS`, `FENCE_MARGIN` | *site* | the hit approach must stay inside the fence |
| `RTL_ALT_M`, `RTL_ALT_FINAL_M` (0 = land), `RTL_SPEED_MS` | *site* | units are metres in 4.8-dev |
| `FS_CRASH_CHECK` | 1 | |

## 6. D. RC / SkyDroid G20 + GR01 (SBUS)

| Item | Setting |
|---|---|
| RC input | GR01 SBUS → Cube RC input; `RC_PROTOCOLS` default (all) or SBUS only; calibrate in QGC |
| Flight modes | `FLTMODE_CH` = the G20 mode-switch channel; `FLTMODE1..6` e.g. Loiter / AltHold / Guided / RTL / Land / Stabilize. PWM bands: ≤1230, ≤1360, ≤1490, ≤1620, ≤1749, ≥1750. The pilot **always** regains control by moving this switch (verified, row 3). |
| Hit channel | one spare G20 channel, **3-position**: low < 1300 (safe), mid (armed), high > 1700 (fire); the existing app already uses 1700/1300 thresholds. Set that channel's `RCx_OPTION = 0` so ArduPilot does nothing with it. |
| Sticks | The Pi reads `RC_CHANNELS` (≥ 10 Hz) to detect stick movement and release control. Verified in SITL with emulated sticks; confirm with a real transmitter on the bench. |

## 7. E. Arming, logging

| Parameter | Value | Why |
|---|---|---|
| `ARMING_SKIPCHK` | 0 | keep all checks |
| `ARMING_RUDDER`, `BRD_SAFETY_DEFLT` | per policy/hardware | |
| `DISARM_DELAY` | 10 (default) | an armed, landed vehicle with no takeoff auto-disarms |
| `LOG_BACKEND_TYPE` | default (File) | keep dataflash logs for every flight test |
| `LOG_DISARMED` | 0 | |

## 8. Bench verification (props removed) before any flight

1. Read back every parameter; screenshot the list.
2. RC: sweep each G20 switch; confirm mode changes and the hit channel PWM in QGC.
3. Pull the G20 link (or switch it off): GCS failsafe fires; with the Pi powered and heartbeating, it must still fire.
4. Stop the companion program mid-command: the vehicle must stop within ≈ `GUID_TIMEOUT` (SITL equivalent above).
5. Low-battery and EKF failsafes, geofence breach: simulate in SITL first, then bench.

## 9. Not verified yet

* Names/values on the **actual Cube firmware** (see section 1).
* `WP_SPD_UP/DN` effect on guided vertical speed; a real transmitter's stick values in `RC_CHANNELS`.
* Behaviour of the QGC click-to-track UI (separate test: `qgc_camera_test/`).

## 10. Which rows apply to the simulation (SITL + Jetson) and which only to the real drone

| Section | Simulation now (SITL, Jetson over UDP) | Real drone (Cube Orange + Pi over UART) |
|---|---|---|
| 1. Renamed parameters | applies (SITL is 4.8-dev) – check names on the Cube firmware before copying values | **check every name** |
| 2. Verified findings | these are the SITL results | same ArduPilot code, so expected to hold; re-check on the bench |
| A. Companion link | **no** UART/`SERIAL2_*`/`MAV2_*` (Jetson uses MAVProxy UDP). **Yes**: companion identity (sysid 1 / compid 191, never 255) and heartbeat | yes, all rows |
| B. Guided limits | yes (`GUID_TIMEOUT`, `ATC_ANGLE_MAX`, clamp speeds in code) | yes, tune for the airframe |
| C. Failsafes | can be tested in SITL (`FS_GCS_*`, geofence, battery via SIM params) | yes, on the bench with props removed; values are site/battery specific |
| D. G20 / GR01 / SBUS | **no** (simulated sticks via MAVProxy `rc` or RC override from sysid 255) | yes |
| E. Arming, logging | yes (SITL logs) | yes |
