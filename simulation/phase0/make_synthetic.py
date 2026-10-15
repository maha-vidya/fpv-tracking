#!/usr/bin/env python3
"""Generate a synthetic test clip (+ ground truth) to verify the benchmark harness.
NOT a performance reference: real footage from the real camera is what decides the tracker."""
import argparse, csv, cv2, numpy as np

def patch(w, h, seed, hue):
    rng = np.random.RandomState(seed)
    p = np.zeros((h, w, 3), np.uint8); p[:] = (hue, 200, 220)
    p = cv2.cvtColor(p, cv2.COLOR_HSV2BGR)
    for _ in range(14):
        x, y = rng.randint(0, w - 6), rng.randint(0, h - 6)
        cv2.rectangle(p, (x, y), (x + rng.randint(4, w // 3), y + rng.randint(4, h // 3)), tuple(int(c) for c in rng.randint(0, 255, 3)), -1)
    cv2.rectangle(p, (0, 0), (w - 1, h - 1), (20, 20, 20), 2)
    return p

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='synthetic'); ap.add_argument('--seconds', type=int, default=20)
    ap.add_argument('--w', type=int, default=640); ap.add_argument('--h', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    a = ap.parse_args()
    W, H, FPS = a.w, a.h, a.fps; N = a.seconds * FPS
    rng = np.random.RandomState(1)
    bg = cv2.GaussianBlur(rng.randint(60, 200, (H, W, 3)).astype(np.uint8), (0, 0), 4)
    tgt = patch(70, 110, 5, 10); dis = patch(70, 110, 6, 14)        # lookalike: same shape, similar hue
    vw = cv2.VideoWriter(a.out + '.mp4', cv2.VideoWriter_fourcc(*'mp4v'), FPS, (W, H))
    gt = []; gt2 = []
    for k in range(N):
        t = k / float(N)
        f = bg.copy()
        # target path with scale growth (approach) and a bounce
        s = 1.0 + 0.8 * t
        tw, th = int(70 * s), int(110 * s)
        tx = int(80 + (W - 220) * t + 40 * np.sin(k / 12.0)); ty = int(150 + 60 * np.sin(k / 40.0))
        # distractor crosses near the target around 55-65% of the clip
        dx = int(W * 0.85 - (W * 0.7) * t + 30 * np.cos(k / 15.0)); dy = int(ty + 25 * np.sin(k / 25.0))
        occl = (0.40 <= t < 0.47)                 # target hidden behind a grey wall
        gone = (0.80 <= t < 0.85)                 # target leaves the frame
        if gone: tx = W + 50
        T = cv2.resize(tgt, (tw, th)); D = cv2.resize(dis, (int(70 * s), int(110 * s)))
        def paste(img, p, x, y):
            ph, pw = p.shape[:2]
            x0, y0, x1, y1 = max(x, 0), max(y, 0), min(x + pw, W), min(y + ph, H)
            if x1 > x0 and y1 > y0: img[y0:y1, x0:x1] = p[y0 - y:y1 - y, x0 - x:x1 - x]
        paste(f, D, dx, dy); paste(f, T, tx, ty)
        if occl: cv2.rectangle(f, (tx - 20, ty - 30), (tx + tw + 20, ty + th + 30), (110, 110, 110), -1)
        f = cv2.add(f, rng.randint(0, 6, f.shape, dtype=np.uint8))  # sensor noise
        vw.write(f)
        vis = 0 if (occl or gone or tx + tw // 2 > W) else 1
        gt.append((k, tx, ty, tw, th, vis)); gt2.append((k, dx, dy, int(70 * s), int(110 * s), 1))
    vw.release()
    for name, rows in ((a.out + '_gt.csv', gt), (a.out + '_distractor_gt.csv', gt2)):
        with open(name, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['frame', 'x', 'y', 'w', 'h', 'visible'])
            for r in rows[::5] + [rows[-1]]: w.writerow(r)       # sparse keyframes every 5 frames
    print('wrote', a.out + '.mp4', a.out + '_gt.csv', a.out + '_distractor_gt.csv', f'({N} frames @ {FPS} fps, {W}x{H})')

if __name__ == '__main__':
    main()
