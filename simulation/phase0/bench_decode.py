#!/usr/bin/env python3
"""Phase 0 video DECODE benchmark: how much CPU does decoding the camera's stream cost on this board?
Tries (a) OpenCV/FFmpeg, (b) OpenCV with GStreamer pipelines, (c) gst-launch with software and hardware decoders.
Use the clip recorded straight from the camera with `ffmpeg -c copy` (see README) so the codec/bitrate is the real one.
"""
import argparse, json, os, resource, shutil, subprocess, time
import cv2
from common import *

def cpu_now():
    r = resource.getrusage(resource.RUSAGE_SELF); c = resource.getrusage(resource.RUSAGE_CHILDREN)
    return r.ru_utime + r.ru_stime + c.ru_utime + c.ru_stime

def bench_cv(label, cap_factory, seconds, resize_to):
    cap = cap_factory()
    if not cap or not cap.isOpened():
        return {'method': label, 'error': 'could not open'}
    n = 0; c0 = cpu_now(); t0 = time.perf_counter(); t_end = t0 + seconds; t_temp = read_temp_c()
    while time.perf_counter() < t_end:
        ok, f = cap.read()
        if not ok:
            cap.release(); cap = cap_factory()
            if not cap.isOpened(): break
            continue
        n += 1
    wall = time.perf_counter() - t0; cap.release()
    return {'method': label, 'frames': n, 'fps': round(n / wall, 1), 'cpu_cores_used': round((cpu_now() - c0) / wall, 2),
            'temp_c_start': t_temp, 'temp_c_end': read_temp_c()}

def bench_gst_launch(label, pipeline, frames, seconds):
    if not shutil.which('gst-launch-1.0'):
        return {'method': label, 'error': 'gst-launch-1.0 not installed'}
    c0 = cpu_now(); t0 = time.perf_counter()
    try:
        p = subprocess.run(['gst-launch-1.0', '-q'] + pipeline.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=seconds + 60)
    except Exception as e:
        return {'method': label, 'error': str(e)}
    wall = time.perf_counter() - t0
    if p.returncode != 0:
        return {'method': label, 'error': (p.stderr or p.stdout).strip().splitlines()[-1][:200] if (p.stderr or p.stdout) else 'failed'}
    return {'method': label, 'frames': frames, 'fps': round(frames / wall, 1), 'cpu_cores_used': round((cpu_now() - c0) / wall, 2),
            'note': 'whole-file decode, unthrottled (sync=false)'}

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--video', required=True, help='clip recorded with ffmpeg -c copy from the real camera stream')
    ap.add_argument('--seconds', type=float, default=15); ap.add_argument('--out', default='results_decode.json')
    args = ap.parse_args()
    cap = cv2.VideoCapture(args.video); frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)); cap.release()
    info = sysinfo(cv2); info['video'] = os.path.basename(args.video); info['frames'] = frames
    print(json.dumps(info, indent=1))
    res = []
    res.append(bench_cv('opencv_ffmpeg', lambda: cv2.VideoCapture(args.video, cv2.CAP_FFMPEG), args.seconds, None))
    v = os.path.abspath(args.video)
    base = f'filesrc location={v} ! decodebin ! videoconvert ! video/x-raw,format=BGR'
    if info.get('opencv_gstreamer'):
        res.append(bench_cv('opencv_gstreamer(decodebin)', lambda: cv2.VideoCapture(base + ' ! appsink', cv2.CAP_GSTREAMER), args.seconds, None))
    # decoders that may exist: software, Pi 4 style V4L2 hardware, others
    def has(el):
        return shutil.which('gst-inspect-1.0') and subprocess.run(['gst-inspect-1.0', el], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode == 0
    dec = [('avdec_h264', 'avdec_h264 ! videoconvert'), ('v4l2h264dec', 'v4l2h264dec ! v4l2convert'), ('v4l2slh264dec', 'v4l2slh264dec ! v4l2convert'),
           ('nvv4l2decoder', 'nvv4l2decoder ! nvvidconv')]
    for el, tail in dec:
        if has(el):
            pipe = f'filesrc location={v} ! qtdemux ! h264parse ! {tail} ! fakesink sync=false'
            res.append(bench_gst_launch('gst:' + el, pipe, frames, args.seconds))
    json.dump({'info': info, 'results': res}, open(args.out, 'w'), indent=1)
    for r in res: print(json.dumps(r))
    print('\nSaved', args.out)

if __name__ == '__main__':
    main()
