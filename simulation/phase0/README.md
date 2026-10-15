# Phase 0 – tracker & decode benchmark (nothing here is part of the flight code)

Purpose: choose the tracker (KCF / CSRT / MOSSE / Nano / ViT) and the board (Pi 5, Pi 4, RDK X5 …)
from **measurements on our own footage**, not from what happens to be installed.

## 1. Record the footage (do this first, any machine)

Record the camera stream **exactly as the Pi will receive it** (no re-encode), at the real resolution/fps/bitrate:

    ffmpeg -rtsp_transport tcp -i rtsp://<CAMERA_URL> -c copy -t 60 S3_lookalike.mp4

(USB/CSI camera on the Pi: record an H.264 file with the same settings you will fly with.)

One 30–60 s clip per scenario; keep one target and, where noted, one lookalike:

| Clip | What happens | Why it matters |
|---|---|---|
| S1_baseline | one target walks slowly across the frame, mid distance | speed / baseline accuracy |
| S2_approach | camera moves toward the target so it grows 3–5× | scale change = the "hit" approach |
| S3_lookalike | two similar targets (same clothes/colour) pass and cross | identity switches (our known bug) |
| S4_occlusion | target goes behind a pole/person/tree for 1–2 s | loss detection + recovery |
| S5_exit_reenter | target leaves the frame, comes back | lost-target logic |
| S6_shake | fast pan, wind, **vibration with props spinning** (tether/hold the drone) | motion blur, jitter |
| S7_lowcontrast | dusk / cloudy / background similar to target | appearance failure |
| S8_small_far | target only ~20–40 px tall | range limit |

Put real flight conditions in as many clips as you safely can. Props-on bench clips are fine for S6.

## 2. Label the clips (on the laptop, needs a screen)

    python3 annotate.py --video S3_lookalike.mp4 --out S3_lookalike_gt.csv                 # the target
    python3 annotate.py --video S3_lookalike.mp4 --out S3_lookalike_distractor_gt.csv     # the lookalike (S3 only)

Drag a box + ENTER every 10th frame; press `c` when the object is not visible.

## 3. Run on each board

    # what does this board's OpenCV offer?   (kcf/csrt need opencv-contrib; nano needs >= 4.7, vit >= 4.9)
    python3 -c "import cv2; print(cv2.__version__)"
    python3 bench_tracker.py --video S3_lookalike.mp4 --gt S3_lookalike_gt.csv \
        --distractor-gt S3_lookalike_distractor_gt.csv --trackers all \
        --max-width 640 --threads 2 --out results_<board>_S3

* `--threads N` = the number of cores we would really give the tracker (the flight program also needs cores).
* Neural trackers need model files: `--vit-model object_tracking_vittrack_2023sep.onnx`
  (OpenCV Zoo: `models/object_tracking_vittrack`, needs OpenCV > 4.8) and
  `--nano-backbone ... --nano-neckhead ...` (NanoTrack ONNX pair; source to be confirmed – see OpenCV TrackerNano docs).
* To get ALL trackers without compiling: in a venv, `pip install opencv-contrib-python-headless` (check the
  harness prints kcf/csrt/mosse/nano/vit as available; wheel version/architecture to be verified on the board).
* Sustained/thermal run on the clip you consider hardest: add `--repeat-seconds 600`.
* Decode cost of the real stream:  `python3 bench_decode.py --video S3_lookalike.mp4`

Send back the whole `results_*` folders (summary.json, *_frames.csv, *_thermal.csv) plus `lscpu` and, on a Pi,
`vcgencmd get_throttled`.

## 4. Proposed decision rules (to agree before we look at the numbers)

* Holds the camera frame rate with headroom (p95 update time < 1 frame period) using the agreed core budget.
* No thermal throttling in the 10-minute run (`throttled_end` clean, CPU MHz not dropping).
* Highest success at IoU 0.5 on S1/S2/S6; zero distractor switches on S3; reports "lost" on S4/S5 when the target is absent.
* Tracker wins only if it meets the above on the **slowest board we intend to support**.

`make_synthetic.py` creates a toy clip + labels just to prove the tools run; its numbers mean nothing.
