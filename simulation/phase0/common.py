"""Shared helpers for the Phase 0 benchmark tools (no project code is imported)."""
import csv, glob, os, platform, subprocess, time

def read_temp_c():
    """CPU temperature in Celsius. Prefers a zone whose type looks like the CPU/SoC
    (cpu-thermal, x86_pkg_temp, soc...), otherwise the hottest readable zone."""
    best = None; hottest = None
    for z in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        try:
            t = int(open(z + '/temp').read().strip()) / 1000.0
            typ = open(z + '/type').read().strip().lower()
        except Exception:
            continue
        hottest = t if hottest is None else max(hottest, t)
        if best is None and any(k in typ for k in ('cpu', 'x86_pkg', 'soc', 'bcm', 'a55', 'a76', 'a72')):
            best = t
    return best if best is not None else hottest

def read_cpu_mhz():
    """Current CPU frequency in MHz (cpu0 scaling_cur_freq), else None."""
    try:
        return int(open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq').read()) / 1000.0
    except Exception:
        return None

def throttled_flags():
    """Raspberry Pi 'vcgencmd get_throttled' value (hex string) or None."""
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=2).stdout
        return out.strip().split('=')[-1]
    except Exception:
        return None

def board_model():
    for p in ('/proc/device-tree/model', '/sys/firmware/devicetree/base/model'):
        try:
            return open(p, 'rb').read().decode(errors='ignore').strip('\x00').strip()
        except Exception:
            pass
    return platform.node()

def cpu_model():
    try:
        for line in open('/proc/cpuinfo'):
            if line.lower().startswith(('model name', 'hardware')):
                return line.split(':', 1)[1].strip()
    except Exception:
        pass
    return platform.processor() or platform.machine()

def mem_total_mb():
    try:
        for line in open('/proc/meminfo'):
            if line.startswith('MemTotal'):
                return int(line.split()[1]) // 1024
    except Exception:
        return None

def sysinfo(cv2=None):
    info = {
        'board': board_model(), 'cpu': cpu_model(), 'machine': platform.machine(),
        'os': platform.platform(), 'python': platform.python_version(),
        'cpus': os.cpu_count(), 'mem_mb': mem_total_mb(),
        'temp_c_start': read_temp_c(), 'cpu_mhz_start': read_cpu_mhz(),
        'throttled_start': throttled_flags(),
    }
    if cv2 is not None:
        info['opencv'] = cv2.__version__
        info['trackers_available'] = available_trackers(cv2)
        try:
            bi = cv2.getBuildInformation()
            info['opencv_gstreamer'] = any('GStreamer' in l and 'YES' in l for l in bi.splitlines())
            info['opencv_ffmpeg'] = any(l.strip().startswith('FFMPEG') and 'YES' in l for l in bi.splitlines())
        except Exception:
            pass
    return info

def available_trackers(cv2):
    names = {}
    def has(*path):
        o = cv2
        for p in path:
            o = getattr(o, p, None)
            if o is None:
                return False
        return True
    names['kcf'] = has('TrackerKCF_create') or has('legacy', 'TrackerKCF_create')
    names['csrt'] = has('TrackerCSRT_create') or has('legacy', 'TrackerCSRT_create')
    names['mosse'] = has('legacy', 'TrackerMOSSE_create')
    names['mil'] = has('TrackerMIL_create')
    names['nano'] = has('TrackerNano_create')
    names['vit'] = has('TrackerVit_create')
    return names

def load_gt(path):
    """CSV: frame,x,y,w,h,visible (keyframes). Returns function frame->(box|None).
    Boxes are linearly interpolated between consecutive visible keyframes; a keyframe with
    visible=0 starts an 'absent' span that lasts until the next visible keyframe."""
    rows = []
    with open(path, newline='') as f:
        for r in csv.DictReader(f):
            rows.append((int(r['frame']), float(r['x']), float(r['y']), float(r['w']), float(r['h']), int(r['visible'])))
    rows.sort()
    def at(frame):
        prev = None; nxt = None
        for r in rows:
            if r[0] <= frame:
                prev = r
            if r[0] >= frame:
                nxt = r; break
        if prev is None or not prev[5]:
            return None
        if nxt is None or nxt is prev or not nxt[5]:
            return (prev[1], prev[2], prev[3], prev[4]) if (nxt is None or nxt is prev) else None
        t = (frame - prev[0]) / max(1, nxt[0] - prev[0])
        return tuple(prev[i] + t * (nxt[i] - prev[i]) for i in (1, 2, 3, 4))
    return at

def iou(a, b):
    if a is None or b is None:
        return 0.0
    ax, ay, aw, ah = a; bx, by, bw, bh = b
    ix = max(0.0, min(ax + aw, bx + bw) - max(ax, bx)); iy = max(0.0, min(ay + ah, by + bh) - max(ay, by))
    inter = ix * iy; uni = aw * ah + bw * bh - inter
    return inter / uni if uni > 0 else 0.0

def percentile(vals, p):
    if not vals:
        return None
    s = sorted(vals); k = (len(s) - 1) * p / 100.0
    lo = int(k); hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)
