#!/usr/bin/env python3
"""Make a ground-truth CSV for a recorded clip by drawing boxes on keyframes.
For every keyframe (default every 10th frame): drag a box around the target and press ENTER/SPACE;
press 'c' (cancel) when the target is NOT visible (occluded / out of frame). ESC quits and saves.
Boxes between keyframes are interpolated by the benchmark. Run this on a desktop machine (laptop).
Run it once per object: target -> <clip>_gt.csv, lookalike -> <clip>_distractor_gt.csv
"""
import argparse, csv, cv2

ap = argparse.ArgumentParser()
ap.add_argument('--video', required=True); ap.add_argument('--out', required=True)
ap.add_argument('--every', type=int, default=10); ap.add_argument('--max-width', type=int, default=1280)
a = ap.parse_args()
cap = cv2.VideoCapture(a.video); rows = []; k = 0
W = cap.get(cv2.CAP_PROP_FRAME_WIDTH); sc = min(1.0, a.max_width / W) if W else 1.0
while True:
    ok, f = cap.read()
    if not ok: break
    if k % a.every == 0:
        show = cv2.resize(f, None, fx=sc, fy=sc) if sc != 1.0 else f
        r = cv2.selectROI(f'frame {k}  (drag box + ENTER, or c = not visible, ESC = finish)', show, False, False)
        cv2.destroyAllWindows()
        if r == (0, 0, 0, 0): rows.append((k, 0, 0, 0, 0, 0))
        else: rows.append((k, r[0] / sc, r[1] / sc, r[2] / sc, r[3] / sc, 1))
    k += 1
# make sure the last frame has a keyframe so interpolation covers the whole clip
if rows and rows[-1][0] != k - 1: rows.append((k - 1,) + rows[-1][1:])
with open(a.out, 'w', newline='') as fh:
    w = csv.writer(fh); w.writerow(['frame', 'x', 'y', 'w', 'h', 'visible']); w.writerows(rows)
print('saved', a.out, 'with', len(rows), 'keyframes (frames in clip:', k, ')')
