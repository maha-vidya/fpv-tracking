#!/usr/bin/env python3
"""Offline click-to-lock benchmark (no simulator, no vehicle): runs the LIVE lock path (target_tracker --lock-job-test, warm) on
every saved click image (the tracker writes click_*_frame_X_Y.png into its --log-dir), and scores it.

  python3 tracker/tests/lock_bench.py ~/sim_run            # all click images below that folder
  python3 tracker/tests/lock_bench.py DIR --min-iou 0.7 --max-ms 150

Truth: clicks on the simulator's RED box are scored against its red pixels (IoU); all clicks count for lock time and lock/refuse
rates. Exit code 1 when the red-box median IoU or the median lock time misses its limit (a regression).
"""
import argparse, glob, os, re, subprocess, sys
import cv2, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BIN = os.environ.get("BENCH_BIN", os.path.join(HERE, "..", "build", "target_tracker"))   # BENCH_BIN: benchmark another build

def red_boxes(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mk = (((hsv[..., 0] < 6) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 120) & (hsv[..., 2] > 50)).astype(np.uint8)
    n, lab, st, cc = cv2.connectedComponentsWithStats(mk)
    return [tuple(int(v) for v in st[i, :4]) for i in range(1, n) if st[i, 4] >= 60]

def iou(a, b):
    ix = max(0, min(a[0] + a[2], b[0] + b[2]) - max(a[0], b[0])); iy = max(0, min(a[1] + a[3], b[1] + b[3]) - max(a[1], b[1]))
    i = ix * iy; return i / (a[2] * a[3] + b[2] * b[3] - i + 1e-9)

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir"); ap.add_argument("--min-iou", type=float, default=0.70); ap.add_argument("--max-ms", type=float, default=150.0)
    ap.add_argument("--legacy", action="store_true", help="the previous lock path (LOCK_NO_GROUND=1), for A/B")
    a = ap.parse_args()
    fs = sorted(glob.glob(os.path.join(os.path.expanduser(a.dir), "**", "click_*_frame_*_*.png"), recursive=True))
    if not fs: sys.exit("no click_*_frame_X_Y.png images under " + a.dir)
    env = dict(os.environ, LOCK_WARM="1", **({"LOCK_NO_GROUND": "1"} if a.legacy else {}))
    ious, ms, locked, large = [], [], 0, 0
    for f in fs:
        x, y = map(int, re.search(r"frame_(\d+)_(\d+)\.png$", f).groups())
        o = subprocess.run([BIN, "--lock-job-test", f, f, str(x), str(y)], capture_output=True, text=True, env=env).stdout
        m = re.search(r"\((\d+) ms", o)
        if m: ms.append(int(m.group(1)))
        b = list(map(float, o.split()[1:5])) if o.startswith("OK") else None
        if b: locked += 1; large += b[2] * b[3] > 6000
        gts = [g for g in red_boxes(cv2.imread(f)) if g[0] - 15 <= x <= g[0] + g[2] + 15 and g[1] - 15 <= y <= g[1] + g[3] + 15]
        if gts: ious.append(iou(b, gts[0]) if b else 0.0)
    r = np.array(ious) if ious else np.array([0.0]); t = np.array(ms) if ms else np.array([0])
    print("clicks %d: locked %d, refused %d, large-object locks %d" % (len(fs), locked, len(fs) - locked, large))
    print("lock time (warm): median %.0f ms, p90 %.0f ms" % (np.median(t), np.percentile(t, 90)))
    print("red-box clicks %d: median IoU %.2f, >= 0.7: %.0f%%, bad (< 0.3): %.0f%%" % (len(ious), np.median(r), 100 * (r >= .7).mean(), 100 * (r < .3).mean()))
    ok = np.median(r) >= a.min_iou and np.median(t) <= a.max_ms
    print("RESULT:", "PASS" if ok else "FAIL (limits: median IoU >= %.2f, median lock time <= %.0f ms)" % (a.min_iou, a.max_ms))
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
