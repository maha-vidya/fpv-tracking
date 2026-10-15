#!/usr/bin/env python3
"""Phase 0 tracker benchmark: run KCF / CSRT / MOSSE / MIL / Nano / Vit on the SAME clip and report
speed, CPU, temperature, accuracy (needs a ground-truth CSV) and loss-reporting behaviour.

Only the tracker's update() call is timed for FPS/latency; video decode is excluded (see bench_decode.py).
Run it on every candidate board (Pi 5, Pi 4, RDK X5 ...) with the same clip and send back the results folder.
"""
import argparse, json, os, resource, sys, time
import cv2, numpy as np
from common import *

def make_tracker(name, args):
    if name == 'kcf':
        return (cv2.TrackerKCF_create if hasattr(cv2, 'TrackerKCF_create') else cv2.legacy.TrackerKCF_create)()
    if name == 'csrt':
        return (cv2.TrackerCSRT_create if hasattr(cv2, 'TrackerCSRT_create') else cv2.legacy.TrackerCSRT_create)()
    if name == 'mosse':
        return cv2.legacy.TrackerMOSSE_create()
    if name == 'mil':
        return cv2.TrackerMIL_create()
    if name == 'nano':
        if not (args.nano_backbone and args.nano_neckhead):
            raise RuntimeError('nano needs --nano-backbone and --nano-neckhead (ONNX files)')
        p = cv2.TrackerNano_Params(); p.backbone = args.nano_backbone; p.neckhead = args.nano_neckhead
        return cv2.TrackerNano_create(p)
    if name == 'vit':
        if not args.vit_model:
            raise RuntimeError('vit needs --vit-model (ONNX file)')
        p = cv2.TrackerVit_Params(); p.net = args.vit_model
        return cv2.TrackerVit_create(p)
    raise ValueError(name)

def read_frame(cap, scale):
    ok, f = cap.read()
    if not ok:
        return None
    if scale != 1.0:
        f = cv2.resize(f, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return f

def run_one(name, args, gt, gt2, scale):
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise RuntimeError('cannot open ' + args.video)
    frame0 = read_frame(cap, scale)
    # init box: explicit, or first visible GT box
    if args.init_box:
        box = tuple(float(v) for v in args.init_box.split(','))
        init_frame = 0
    else:
        init_frame = None
        for k in range(0, 100000):
            b = gt(k) if gt else None
            if b:
                init_frame = k; box = tuple(v * scale for v in b); break
        if init_frame is None:
            raise RuntimeError('no GT box found; pass --init-box x,y,w,h')
    # advance to the init frame
    frame = frame0; idx = 0
    while idx < init_frame:
        frame = read_frame(cap, scale); idx += 1
        if frame is None:
            raise RuntimeError('video shorter than init frame')
    tr = make_tracker(name, args)
    t0 = time.perf_counter(); ok_init = tr.init(frame, tuple(int(round(v)) for v in box)); init_ms = (time.perf_counter() - t0) * 1000
    # warm-up on the init frame (some trackers JIT-allocate on first update)
    rec = []; temps = []
    cpu0 = time.process_time(); wall0 = time.perf_counter(); ru0 = resource.getrusage(resource.RUSAGE_SELF)
    last_temp_t = 0.0; first_temp = read_temp_c(); max_temp = first_temp or 0.0; min_mhz = None
    deadline = wall0 + args.repeat_seconds if args.repeat_seconds else None
    n_frames = 0
    while True:
        frame = read_frame(cap, scale)
        if frame is None:
            if deadline and time.perf_counter() < deadline:      # loop the clip for sustained / thermal runs
                cap.release(); cap = cv2.VideoCapture(args.video)
                frame = read_frame(cap, scale); idx = 0
                for _ in range(init_frame):
                    frame = read_frame(cap, scale); idx += 1
                tr = make_tracker(name, args); tr.init(frame, tuple(int(round(v)) for v in box))
                continue
            break
        idx += 1; n_frames += 1
        t1 = time.perf_counter(); ok, bb = tr.update(frame); dt = (time.perf_counter() - t1) * 1000
        score = None
        if hasattr(tr, 'getTrackingScore'):
            try: score = float(tr.getTrackingScore())
            except Exception: pass
        rec.append((idx, dt, bool(ok), tuple(float(v) for v in bb) if ok else None, score))
        now = time.perf_counter()
        if now - last_temp_t >= 1.0:
            last_temp_t = now; t = read_temp_c(); mh = read_cpu_mhz()
            if t is not None: max_temp = max(max_temp, t)
            if mh is not None: min_mhz = mh if min_mhz is None else min(min_mhz, mh)
            temps.append((round(now - wall0, 1), t, mh))
        if args.max_frames and n_frames >= args.max_frames and not deadline:
            break
        if deadline and now >= deadline:
            break
    cap.release()
    wall = time.perf_counter() - wall0; cpu = time.process_time() - cpu0
    # --- metrics
    times = [r[1] for r in rec]
    res = {'tracker': name, 'frames': len(rec), 'init_ms': round(init_ms, 2), 'init_ok': bool(ok_init),
           'update_ms_mean': round(sum(times) / len(times), 2) if times else None,
           'update_ms_p50': round(percentile(times, 50), 2) if times else None,
           'update_ms_p95': round(percentile(times, 95), 2) if times else None,
           'update_ms_p99': round(percentile(times, 99), 2) if times else None,
           'update_ms_max': round(max(times), 2) if times else None,
           'fps_from_update_mean': round(1000.0 / (sum(times) / len(times)), 1) if times else None,
           'process_cpu_cores_used': round(cpu / wall, 2) if wall else None,   # includes video decode in this process
           'rss_mb_peak': round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0, 1),
           'temp_c_start': first_temp, 'temp_c_max': round(max_temp, 1) if max_temp else None,
           'cpu_mhz_min_seen': min_mhz, 'throttled_end': throttled_flags(), 'wall_s': round(wall, 1)}
    if gt:
        vis = []; absent = []; lost_flag_vis = 0; lost_flag_abs = 0; ious = []; sw = 0; sw_first = None
        for idx_, dt, ok, bb, score in rec:
            g = gt(idx_ % (args.gt_length or 10**9) if args.gt_length else idx_)
            g = tuple(v * scale for v in g) if g else None
            lost = (not ok) or (args.score_threshold is not None and score is not None and score < args.score_threshold)
            if g:
                i = iou(bb, g) if ok else 0.0; ious.append(i); vis.append(i)
                lost_flag_vis += 1 if lost else 0
                if gt2:
                    g2 = gt2(idx_); g2 = tuple(v * scale for v in g2) if g2 else None
                    if ok and g2 and iou(bb, g2) > max(0.3, i):
                        sw += 1; sw_first = sw_first or idx_
            else:
                absent.append(1); lost_flag_abs += 1 if lost else 0
        res.update({
            'gt_visible_frames': len(vis), 'iou_mean': round(sum(vis) / len(vis), 3) if vis else None,
            'success_iou_0.5': round(sum(1 for v in vis if v >= 0.5) / len(vis), 3) if vis else None,
            'success_iou_0.3': round(sum(1 for v in vis if v >= 0.3) / len(vis), 3) if vis else None,
            'reported_lost_while_visible': round(lost_flag_vis / len(vis), 3) if vis else None,
            'gt_absent_frames': len(absent),
            'reported_lost_while_absent': round(lost_flag_abs / len(absent), 3) if absent else None,
            'distractor_switch_frames': sw if gt2 else None, 'first_switch_frame': sw_first if gt2 else None})
    return res, rec, temps

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--video', required=True)
    ap.add_argument('--gt', help='ground-truth CSV (frame,x,y,w,h,visible) in ORIGINAL video pixels')
    ap.add_argument('--distractor-gt', help='optional GT CSV of a lookalike object, for identity-switch counting')
    ap.add_argument('--trackers', default='all', help='comma list of kcf,csrt,mosse,mil,nano,vit or "all" (= every available)')
    ap.add_argument('--scale', type=float, default=1.0, help='resize factor applied to frames (e.g. 0.5)')
    ap.add_argument('--max-width', type=int, help='alternative to --scale: resize so width <= this')
    ap.add_argument('--init-box', help='x,y,w,h in original pixels at frame 0 (default: first visible GT box)')
    ap.add_argument('--max-frames', type=int)
    ap.add_argument('--repeat-seconds', type=float, default=0, help='loop the clip for this long (sustained load / thermal test)')
    ap.add_argument('--score-threshold', type=float, help='treat tracker score below this as "lost" (nano/vit)')
    ap.add_argument('--nano-backbone'); ap.add_argument('--nano-neckhead'); ap.add_argument('--vit-model')
    ap.add_argument('--threads', type=int, help='cv2.setNumThreads(N) to mimic the final core budget')
    ap.add_argument('--out', default='results'); ap.add_argument('--label', default='')
    args = ap.parse_args()
    args.gt_length = None
    if args.threads is not None: cv2.setNumThreads(args.threads)
    os.makedirs(args.out, exist_ok=True)
    cap = cv2.VideoCapture(args.video); W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS); nfr = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.release()
    scale = args.scale if not args.max_width else min(1.0, args.max_width / float(W))
    info = sysinfo(cv2); info.update({'video': os.path.basename(args.video), 'video_size': [W, H], 'video_fps': fps,
                                      'video_frames': nfr, 'scale': round(scale, 3), 'label': args.label,
                                      'threads': cv2.getNumThreads()})
    print(json.dumps(info, indent=1))
    avail = info['trackers_available']
    toks = [t.strip() for t in args.trackers.split(',') if t.strip()]
    names = []
    for t in toks:
        for n in ([k for k, ok in avail.items() if ok] if t == 'all' else [t]):
            if n not in names: names.append(n)
    gt = load_gt(args.gt) if args.gt else None; gt2 = load_gt(args.distractor_gt) if args.distractor_gt else None
    results = []
    for n in names:
        if n not in avail or not avail[n]:
            print(f'[skip] {n}: not available in this OpenCV ({cv2.__version__}) - see README for contrib/newer build'); continue
        if n == 'nano' and not (args.nano_backbone and args.nano_neckhead): print('[skip] nano: model paths not given'); continue
        if n == 'vit' and not args.vit_model: print('[skip] vit: model path not given'); continue
        print(f'[run ] {n} ...', flush=True)
        try:
            res, rec, temps = run_one(n, args, gt, gt2, scale)
        except Exception as e:
            print(f'[fail] {n}: {e}'); results.append({'tracker': n, 'error': str(e)}); continue
        results.append(res)
        with open(os.path.join(args.out, f'{n}_frames.csv'), 'w') as f:
            f.write('frame,update_ms,ok,x,y,w,h,score\n')
            for idx, dt, ok, bb, sc in rec:
                f.write(f'{idx},{dt:.3f},{int(ok)},' + (','.join(f'{v:.1f}' for v in bb) if bb else ',,,') + f',{"" if sc is None else sc}\n')
        with open(os.path.join(args.out, f'{n}_thermal.csv'), 'w') as f:
            f.write('t_s,temp_c,cpu_mhz\n'); [f.write(f'{a},{b},{c}\n') for a, b, c in temps]
        print('       ' + json.dumps({k: v for k, v in res.items() if k in ('fps_from_update_mean', 'update_ms_p95', 'process_cpu_cores_used', 'iou_mean', 'success_iou_0.5', 'temp_c_max')}))
    info['temp_c_end'] = read_temp_c(); info['throttled_end'] = throttled_flags()
    out = {'info': info, 'results': results}
    json.dump(out, open(os.path.join(args.out, 'summary.json'), 'w'), indent=1)
    # human table
    cols = ['tracker', 'fps_from_update_mean', 'update_ms_p95', 'update_ms_max', 'process_cpu_cores_used', 'iou_mean',
            'success_iou_0.5', 'reported_lost_while_visible', 'reported_lost_while_absent', 'distractor_switch_frames', 'temp_c_max']
    print('\n' + ' | '.join(cols))
    for r in results:
        print(' | '.join(str(r.get(c, '-')) for c in cols))
    print(f'\nSaved: {args.out}/summary.json (+ per-tracker *_frames.csv, *_thermal.csv)')

if __name__ == '__main__':
    main()
