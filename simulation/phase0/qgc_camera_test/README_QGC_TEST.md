# QGC camera-tracking test (Phase 0, item 3)

Question: with the QGroundControl version we will use, does the video view offer **click-to-track**, and what
exactly does QGC send? This test uses a *fake* companion camera, so no flight code is involved.

## Run
1. `./run_qgc_camera_test.sh`   (needs ArduPilot SITL built at ~/ardupilot, `mavproxy.py`, `gst-launch-1.0` with x264enc)
2. Start QGroundControl (it should connect to the simulated vehicle automatically on UDP 14550).
3. QGC video: *Application Settings → Video → Video Source = "UDP h.264 Video Stream", UDP port = 5600*.
   You should see a moving ball test pattern. (Because the fake camera also announces the stream, QGC may pick it up by itself.)

## What to look at and report back
| # | Question | Where to look |
|---|---|---|
| 1 | QGC version (Help → About / Application Settings → General) | write it down |
| 2 | Does a **camera control panel** appear in Fly view (camera icon / photo / video buttons)? | Fly view |
| 3 | Can you **click or drag a box on the video**? Does QGC show any tracking mode/menu? | try on the video |
| 4 | When you do, does the terminal running the script print `TRACK_POINT` or `TRACK_RECTANGLE`? | terminal output |
| 5 | Does QGC draw a **box around the target** afterwards (from our status messages)? | video |
| 6 | Anything that looks like it asks for "tracking" in QGC settings (Fly view / Video / Gimbal sections)? | settings |

Screenshots of 2, 3 and 5 are very helpful. Repeat the test on the **QGC build that runs on the SkyDroid G20**
(Android) because QGC versions differ.

## Already verified without QGC (protocol level, SITL)
Companion on a *separate serial port* of the flight controller → camera heartbeat reaches the ground station,
CAMERA_INFORMATION (with tracking flags) and VIDEO_STREAM_INFORMATION are served, TRACK_RECTANGLE / TRACK_POINT /
STOP_TRACKING are ACKed, CAMERA_TRACKING_IMAGE_STATUS streams back. Note: ArduPilot forwards only to channels it has
received a packet from, so a GCS must send heartbeats (QGC does) – and the Pi must send its own heartbeat.
