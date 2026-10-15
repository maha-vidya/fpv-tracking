#!/usr/bin/env python3
"""Offline Kalman-filter evaluation on tracker boxes logged by tracker_timing (laptop, numpy only).

Reads a *_frames.csv with columns  frame,ok,iou,gt_visible,bx,by,bw,bh,reported_ok,gx,gy,gw,gh
(extra columns are ignored) and answers three questions about a constant-velocity Kalman filter
on the box (cx, cy, w, h):

  1. SMOOTHING   Does it reduce frame-to-frame jitter without hurting IoU against ground truth?
  2. GATING      Does a large-innovation gate flag the frames where the tracker has drifted
                 (IoU < 0.5) -- with what delay, and how many false alarms?
  3. COASTING    While the tracker reports lost, is the Kalman prediction closer to the target
                 than just holding the last box?

Ground truth is used only to SCORE; the filter and the gate never see it.
This is an offline study on mostly-static-camera clips: it says nothing about behaviour under
drone/gimbal motion (the filter would need ego-motion compensation there).

  python3 kalman_eval.py kf_david_kcf_frames.csv [more.csv ...] [--gate-nis 13.3] [--persist 3]
  python3 kalman_eval.py --selftest
"""
import argparse
import csv
import sys

import numpy as np


def iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    iw = max(0.0, min(ax + aw, bx + bw) - max(ax, bx))
    ih = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = iw * ih
    u = aw * ah + bw * bh - inter
    return inter / u if u > 0 else 0.0


class BoxKalman:
    """Constant-velocity filter. State [cx, cy, w, h, vx, vy]; measurement [cx, cy, w, h].
    Measurement noise scales with box size (a big box wobbles more pixels than a small one)."""

    def __init__(self, box, r_frac=0.08, q_pos=1.0, q_vel=0.5):
        x, y, w, h = box
        self.x = np.array([x + w / 2, y + h / 2, w, h, 0.0, 0.0])
        self.P = np.diag([25.0, 25.0, 25.0, 25.0, 100.0, 100.0])
        self.F = np.eye(6)
        self.F[0, 4] = self.F[1, 5] = 1.0
        self.H = np.zeros((4, 6))
        self.H[:4, :4] = np.eye(4)
        self.r_frac, self.q_pos, self.q_vel = r_frac, q_pos, q_vel

    def _R(self):
        s = max(self.x[2], self.x[3], 8.0) * self.r_frac
        return np.eye(4) * s * s

    def predict(self):
        Q = np.diag([self.q_pos] * 4 + [self.q_vel] * 2) ** 2
        self.x = self.F @ self.x
        self.x[2:4] = np.maximum(self.x[2:4], 4.0)
        self.P = self.F @ self.P @ self.F.T + Q

    def nis(self, box):
        """Normalised innovation squared of a measurement against the current prediction."""
        z = self._z(box)
        S = self.H @ self.P @ self.H.T + self._R()
        v = z - self.H @ self.x
        return float(v @ np.linalg.solve(S, v))

    def update(self, box):
        z = self._z(box)
        S = self.H @ self.P @ self.H.T + self._R()
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (z - self.H @ self.x)
        self.P = (np.eye(6) - K @ self.H) @ self.P

    @staticmethod
    def _z(box):
        x, y, w, h = box
        return np.array([x + w / 2, y + h / 2, w, h])

    def box(self):
        cx, cy, w, h = self.x[:4]
        return (cx - w / 2, cy - h / 2, w, h)


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            def g(k):
                v = r.get(k, '')
                return float(v) if v not in ('', None) else None
            rows.append({'frame': int(float(r['frame'])), 'vis': int(float(r['gt_visible'])) == 1,
                         'ok': int(float(r.get('reported_ok', r.get('ok', 1)))) == 1,
                         'b': tuple(g(k) for k in ('bx', 'by', 'bw', 'bh')),
                         'g': tuple(g(k) for k in ('gx', 'gy', 'gw', 'gh'))})
    return rows


def jitter(boxes):
    """Mean |second difference| of the box centre (pixels): a measure of frame-to-frame twitch."""
    c = np.array([(b[0] + b[2] / 2, b[1] + b[3] / 2) for b in boxes])
    if len(c) < 3:
        return 0.0
    return float(np.mean(np.linalg.norm(c[2:] - 2 * c[1:-1] + c[:-2], axis=1)))


def evaluate(rows, nis_thr=13.3, persist=3, bad_iou=0.5, label=''):
    # --- run the filter: tracker box in, filtered box out; gated frames are not fed to the filter
    start = next((i for i, r in enumerate(rows) if r['vis'] and r['b'][0] is not None), None)
    if start is None:
        return None
    kf = BoxKalman(rows[start]['b'])
    raw_iou, filt_iou, flags, nis_list, filt_boxes, raw_boxes, gt_boxes = [], [], [], [], [], [], []
    consec = 0
    for r in rows[start + 1:]:
        kf.predict()
        meas = r['b']
        flagged = False
        if r['ok'] and meas[0] is not None:
            n = kf.nis(meas)
            nis_list.append(n)
            consec = consec + 1 if n > nis_thr else 0
            flagged = consec >= persist
            if flagged:                           # persistent disagreement = the target "jumped":
                kf = BoxKalman(meas)              # restart on the new box (else the filter locks out)
                consec = 0
            elif n <= nis_thr:                    # otherwise accept only plausible measurements
                kf.update(meas)
        else:
            consec = 0                            # tracker said lost: coast on the prediction
        fb = kf.box()
        if r['vis'] and r['g'][0] is not None:
            raw_iou.append(iou(meas, r['g']) if (r['ok'] and meas[0] is not None) else 0.0)
            filt_iou.append(iou(fb, r['g']))
            gt_boxes.append(r['g'])
            raw_boxes.append(meas)
            filt_boxes.append(fb)
        flags.append((r, flagged, iou(meas, r['g']) if (r['vis'] and r['g'][0] is not None and meas[0] is not None) else None))

    out = {'label': label, 'frames': len(flags)}
    out['iou_raw'] = float(np.mean(raw_iou)) if raw_iou else float('nan')
    out['iou_filtered'] = float(np.mean(filt_iou)) if filt_iou else float('nan')
    out['jitter_raw_px'] = jitter(raw_boxes)
    out['jitter_filtered_px'] = jitter(filt_boxes)
    out['jitter_gt_px'] = jitter(gt_boxes)       # the TRUE motion's own roughness (floor for honest smoothing)

    # --- gating quality: bad runs (>= 3 consecutive visible frames with raw IoU < bad_iou)
    runs, cur = [], None
    for i, (r, fl, io) in enumerate(flags):
        bad = io is not None and io < bad_iou and r['ok']
        if bad:
            if cur is None:
                cur = [i, i]
            else:
                cur[1] = i
        else:
            if cur and cur[1] - cur[0] + 1 >= 3:
                runs.append(tuple(cur))
            cur = None
    if cur and cur[1] - cur[0] + 1 >= 3:
        runs.append(tuple(cur))
    detected, delays, first_bad = 0, [], []
    for a, b in runs:
        hit = next((j for j in range(a - 10, b + 1) if 0 <= j < len(flags) and flags[j][1]), None)
        if hit is not None:
            detected += 1
            delays.append(hit - a)
        first_bad.append(flags[a][0]['frame'])
    good_flag = sum(1 for r, fl, io in flags if fl and io is not None and io >= 0.6)
    flagged_total = sum(1 for r, fl, io in flags if fl)
    out.update({'bad_runs': len(runs), 'bad_runs_flagged': detected,
                'median_delay_frames': float(np.median(delays)) if delays else None,
                'flag_frames': flagged_total, 'flag_frames_with_good_iou': good_flag,
                'first_bad_frames': first_bad[:5]})

    # --- coasting: frames where the tracker reported lost but the target was visible
    cs_pred, cs_hold = [], []
    kf2 = BoxKalman(rows[start]['b'])
    last_ok = rows[start]['b']
    for r in rows[start + 1:]:
        kf2.predict()
        if r['ok'] and r['b'][0] is not None:
            kf2.update(r['b']) if kf2.nis(r['b']) <= nis_thr else None
            last_ok = r['b']
        elif r['vis'] and r['g'][0] is not None:
            cs_pred.append(iou(kf2.box(), r['g']))
            cs_hold.append(iou(last_ok, r['g']))
    out['lost_frames_visible'] = len(cs_pred)
    out['coast_iou_predicted'] = float(np.mean(cs_pred)) if cs_pred else None
    out['coast_iou_held'] = float(np.mean(cs_hold)) if cs_hold else None
    return out


def show(o):
    print('== %s  (%d frames after init)' % (o['label'], o['frames']))
    print('  IoU vs GT          raw %.3f   filtered %.3f' % (o['iou_raw'], o['iou_filtered']))
    print('  centre jitter (px) raw %.2f   filtered %.2f   (GT own roughness %.2f)' %
          (o['jitter_raw_px'], o['jitter_filtered_px'], o['jitter_gt_px']))
    print('  drift runs (IoU<0.5, >=3 frames): %d, caught by gate: %d, median delay %s frames; first at frames %s' %
          (o['bad_runs'], o['bad_runs_flagged'], o['median_delay_frames'], o['first_bad_frames']))
    print('  gate flags: %d frames, of which %d had IoU >= 0.6 (false alarms)' %
          (o['flag_frames'], o['flag_frames_with_good_iou']))
    if o['lost_frames_visible']:
        print('  coasting on %d lost-but-visible frames: Kalman prediction IoU %.3f vs hold-last-box %.3f' %
              (o['lost_frames_visible'], o['coast_iou_predicted'], o['coast_iou_held']))


def _synthetic(kind):
    rng = np.random.default_rng(1)
    rows = []
    for i in range(300):
        gx, gy = 100 + 1.5 * i, 200 + 0.5 * np.sin(i / 20.0) * 20
        g = (gx, gy, 60.0, 90.0)
        b = (gx + rng.normal(0, 2), gy + rng.normal(0, 2), 60 + rng.normal(0, 1), 90 + rng.normal(0, 1))
        if kind == 'slow_drift' and i >= 150:     # slides away 3 px/frame for 50 frames, then stays
            k = min(i - 149, 50)
            b = (b[0] + k * 3.0, b[1] + k * 1.0, b[2], b[3])
        if kind == 'jump' and i >= 150:           # snaps onto a lookalike 100 px away and stays
            b = (b[0] + 100.0, b[1] + 30.0, b[2], b[3])
        rows.append({'frame': i, 'vis': True, 'ok': True, 'b': b, 'g': g})
    return rows


def selftest():
    for kind in ('none', 'slow_drift', 'jump'):
        o = evaluate(_synthetic(kind), label='selftest: ' + kind)
        show(o)
        assert o['jitter_filtered_px'] < o['jitter_raw_px'], 'filter should reduce jitter'
        if kind == 'none':
            assert o['flag_frames'] == 0, 'no false alarms on a clean track'
        if kind == 'jump':
            assert o['bad_runs'] >= 1 and o['bad_runs_flagged'] >= 1, 'a sudden jump must be caught'
            assert o['median_delay_frames'] <= 4, o['median_delay_frames']
        if kind == 'slow_drift':
            print('  (expected: a slow drift is NOT caught by an innovation gate: caught=%d of %d)' %
                  (o['bad_runs_flagged'], o['bad_runs']))
    print('selftest OK')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='*')
    ap.add_argument('--gate-nis', type=float, default=13.3, help='NIS threshold (chi2, 4 dof, 99%% = 13.28)')
    ap.add_argument('--persist', type=int, default=3, help='consecutive over-threshold frames to raise a flag')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        selftest()
        sys.exit(0)
    for p in a.csv:
        res = evaluate(load(p), a.gate_nis, a.persist, label=p)
        if res:
            show(res)
