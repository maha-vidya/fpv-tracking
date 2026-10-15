#!/usr/bin/env python3
"""Offline TRACKING benchmark: replays a recorded camera video through the live tracking core (target_tracker --replay:
click-lock, CSRT + appearance + occlusion handling, camera-motion compensation, Kalman) from several start points, and scores every
frame against the simulator red box's own pixels.

  python3 tracker/tests/track_bench.py ~/sim_run/recordings/csrt_a.avi            # 5 starts
  CSRT_SCALE_LR=0.1 python3 tracker/tests/track_bench.py VIDEO                     # an experiment (env reaches the tracker)

Reports per start and overall: frames tracked (LOCKED) while the box is visible, median IoU, frames on a WRONG object, LOST events.
Record a video from the simulator's camera with: python3 simulation/record_camera.py OUT.avi SECONDS
"""
import argparse, os, subprocess, sys
import cv2, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.environ.get("BENCH_BIN", os.path.join(HERE, "..", "build", "target_tracker"))   # BENCH_BIN: benchmark another build

def red_box(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mk = (((hsv[..., 0] < 6) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 50)).astype(np.uint8)
    n, lab, st, cc = cv2.connectedComponentsWithStats(mk)
    if n < 2: return None
    j = max(range(1, n), key=lambda q: st[q, 4])
    return tuple(int(v) for v in st[j, :4]) if st[j, 4] >= 40 else None

def iou(a, b):
    ix = max(0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])); iy = max(0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))
    i = ix * iy; return i / (a[2] * a[3] + b[2] * b[3] - i + 1e-9)

def truth(video):
    cache = video + ".truth.npy"
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(video): return list(np.load(cache, allow_pickle=True))
    cap = cv2.VideoCapture(video); gt = []
    while True:
        ok, im = cap.read()
        if not ok: break
        gt.append(red_box(im))
    np.save(cache, np.array(gt, dtype=object)); return gt

def run(video, gt, f0, scale):
    g = gt[f0]
    o = subprocess.run([BIN, "--replay", video, str(g[0] + g[2] / 2), str(g[1] + g[3] / 2), str(f0), str(scale)], capture_output=True, text=True)
    rows = [l.split(",") for l in o.stdout.strip().split("\n")[1:] if l]
    vis = tracked = wrong = lost = 0; ious = []; prev = None; perr = []
    for r in rows:
        n, st = int(r[0]), int(r[1]); b = tuple(map(float, r[2:6]))
        pb = tuple(map(float, r[6:10])) if len(r) >= 10 else None
        if st == 3 and prev != 3: lost += 1
        prev = st
        gb = gt[n] if n < len(gt) else None
        if gb is None or gb[0] <= 2 or gb[1] <= 2 or gb[0] + gb[2] >= 1278 or gb[1] + gb[3] >= 718: continue
        vis += 1
        if st != 0: continue
        tracked += 1; v = iou(b, gb); ious.append(v); wrong += v < 0.05
        if pb and pb[2] > 0:   # prediction error: centre distance to the truth, in units of the target's size
            perr.append(np.hypot(pb[0] + pb[2] / 2 - (gb[0] + gb[2] / 2), pb[1] + pb[3] / 2 - (gb[1] + gb[3] / 2)) / max(gb[2], gb[3]))
    return dict(vis=vis, tracked=tracked / max(1, vis), iou=float(np.median(ious)) if ious else 0.0, wrong=wrong / max(1, vis), lost=lost,
                perr=float(np.median(perr)) if perr else 0.0, perr90=float(np.percentile(perr, 90)) if perr else 0.0)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video"); ap.add_argument("--starts", type=int, default=5); ap.add_argument("--scale", type=float, default=0.5)
    a = ap.parse_args()
    gt = truth(a.video); n = len(gt)
    cand = [i for i in range(30, n - 200) if gt[i] is not None and gt[i][2] >= 12]
    starts = [cand[int(k * (len(cand) - 1) / max(1, a.starts - 1))] for k in range(a.starts)] if cand else []
    res = [run(a.video, gt, f0, a.scale) for f0 in starts]
    for f0, r in zip(starts, res):
        print("start %5d: tracked %5.1f%%  IoU %.2f  wrong %4.1f%%  LOST %d  prediction error %.2f (p90 %.2f) x size" % (f0, 100 * r["tracked"], r["iou"], 100 * r["wrong"], r["lost"], r["perr"], r["perr90"]))
    V = sum(r["vis"] for r in res)
    print("OVERALL: tracked %.1f%%  median IoU %.2f  wrong %.2f%%  LOST %d  prediction error %.2f (p90 %.2f) x size" % (100 * sum(r["tracked"] * r["vis"] for r in res) / max(1, V),
          np.median([r["iou"] for r in res]), 100 * sum(r["wrong"] * r["vis"] for r in res) / max(1, V), sum(r["lost"] for r in res),
          np.median([r["perr"] for r in res]), np.median([r["perr90"] for r in res])))

if __name__ == "__main__":
    main()
