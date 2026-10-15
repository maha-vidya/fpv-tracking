#!/usr/bin/env python3
"""Skydroid C12 "TOP" gimbal-camera protocol (UDP port 5000), frame builder/parser + tiny client.

Source: "TOP Protocol EN.docx" v1.1.6 (Skydroid). Python 3.6+ (runs on the Jetson).

Frame (ASCII):  header(3) src(1) dst(1) len(1 hex char) ctrl(1) ident(3) data(len) crc(2)
  header  "#TP" fixed length / "#tp" variable length (G-class always "#TP")
  src/dst U=UART/UDP host, M=lens, D=system&image, E=sub-device, G=gimbal
  ctrl    'w' set/control, 'r' query
  crc     sum of all bytes before the crc, mod 256, as 2 upper-case hex chars

Doc inconsistencies handled here (all MUST be confirmed on real hardware, see README.md):
  * GAM/GAY/GAP angle unit: text says 0.5 deg, but the worked example (-50 deg -> 0xEC78 = -5000)
    and the attitude push use 0.01 deg. We use 0.01 deg.
  * GSM/GSY/GSP speed unit is 0.5 deg/s, but the worked example (-6 -> "-30 -> 0xE2") mixes
    numbers. We use value = round(dps / 0.5), two's complement byte.
  * FAI: the field list mentions a mode byte, the example frame has only yaw/pitch/roll (len 0xC).
    We send yaw/pitch/roll only, matching the example.
"""
import socket
import time

DEFAULT_PORT = 5000

PTZ = {
    "stop": "00", "up": "01", "down": "02", "left": "03", "right": "04", "center": "05",
    "follow": "06", "lock": "07", "toggle_follow_lock": "08",
}


def crc(body):
    """Checksum over the ASCII text before the crc field, as 2 upper-case hex chars."""
    return "%02X" % (sum(body.encode("ascii")) & 0xFF)


def build(src, dst, ctrl, ident, data="", header="#TP"):
    if len(data) > 0x0F:
        raise ValueError("data field too long (max 15 chars): %r" % data)
    if len(ident) != 3 or ctrl not in ("w", "r") or len(src) != 1 or len(dst) != 1:
        raise ValueError("bad frame fields")
    body = "%s%s%s%X%s%s%s" % (header, src, dst, len(data), ctrl, ident, data)
    return body + crc(body)


def parse(frame):
    """Return dict(header,src,dst,ctrl,ident,data) or raise ValueError (bad crc / shape)."""
    if isinstance(frame, bytes):
        frame = frame.decode("ascii", "replace")
    frame = frame.strip()
    if len(frame) < 12 or frame[:3] not in ("#TP", "#tp"):
        raise ValueError("not a TOP frame: %r" % frame)
    body, got = frame[:-2], frame[-2:]
    if crc(body) != got.upper():
        raise ValueError("bad checksum: got %s want %s in %r" % (got, crc(body), frame))
    try:
        n = int(frame[5], 16)
    except ValueError:
        raise ValueError("bad length char in %r" % frame)
    data = frame[10:-2]
    if len(data) != n:
        raise ValueError("length field %d != data length %d in %r" % (n, len(data), frame))
    return {"header": frame[:3], "src": frame[3], "dst": frame[4], "ctrl": frame[6],
            "ident": frame[7:10], "data": data}


def _s8(v):
    v = max(-127, min(127, int(v)))
    return "%02X" % (v & 0xFF)


def _s16(v):
    return "%04X" % (int(v) & 0xFFFF)


def from_s16(h):
    v = int(h, 16)
    return v - 0x10000 if v >= 0x8000 else v


# ---- G-class (gimbal) -------------------------------------------------------------------------
def ptz(action):
    return build("U", "G", "w", "PTZ", PTZ[action])


def gimbal_speed(yaw_dps, pitch_dps):
    """GSM: yaw right +, pitch up +. Unit 0.5 deg/s, clamped to +-127 (+-63.5 deg/s)."""
    return build("U", "G", "w", "GSM", _s8(round(yaw_dps / 0.5)) + _s8(round(pitch_dps / 0.5)))


def gimbal_angle(yaw_deg, pitch_deg, speed=20):
    """GAM: absolute yaw/pitch (0.01 deg units), speed 0..99 (0.5 deg/s)."""
    sp = "%02X" % max(0, min(99, int(speed)))
    return build("U", "G", "w", "GAM",
                 _s16(round(yaw_deg * 100)) + sp + _s16(round(pitch_deg * 100)) + sp)


def attitude_push(hz):
    """GAA: enable the gimbal attitude push at 1..100 Hz, 0 disables."""
    return build("U", "G", "w", "GAA", "%02X" % max(0, min(100, int(hz))))


def parse_attitude_push(frame):
    """Push frame '#TPUGCrGAC' + Y(4) P(4) R(4) -> (yaw, pitch, roll) in degrees."""
    f = parse(frame)
    if f["ident"] != "GAC" or len(f["data"]) != 12:
        raise ValueError("not an attitude push: %r" % (frame,))
    d = f["data"]
    return tuple(from_s16(d[i:i + 4]) / 100.0 for i in (0, 4, 8))


def track_point(x, y):
    """GOT: target position in the 1280x720 frame, origin top-left."""
    if not (0 <= x <= 1280 and 0 <= y <= 720):
        raise ValueError("point outside 1280x720: %r,%r" % (x, y))
    return build("U", "G", "w", "GOT", "%04X%04X" % (int(x), int(y)))


def track_confirm():
    return build("U", "G", "w", "SUM", "01")


def track_stop():
    return build("U", "G", "w", "SUM", "00")


def fc_attitude_enable(on=True):
    return build("U", "G", "w", "FAE", "01" if on else "00")


def fc_attitude(yaw_deg, pitch_deg, roll_deg):
    return build("U", "G", "w", "FAI",
                 _s16(round(yaw_deg * 100)) + _s16(round(pitch_deg * 100)) + _s16(round(roll_deg * 100)))


# ---- D-class (system) / queries -----------------------------------------------------------------
def record(action):   # "stop" | "start" | "toggle"
    return build("U", "D", "w", "REC", {"stop": "00", "start": "01", "toggle": "0A"}[action])


def query(ident, dst="D", data="00"):
    return build("U", dst, "r", ident, data)


class C12Client:
    """Minimal UDP client. Control commands are echoed by the camera; queries return data."""

    def __init__(self, host, port=DEFAULT_PORT, timeout=0.5):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(timeout)

    def send(self, frame):
        self.sock.sendto(frame.encode("ascii"), self.addr)

    def request(self, frame, retries=1):
        """Send and wait for the reply frame; returns the parsed dict or None on timeout."""
        for _ in range(retries + 1):
            self.send(frame)
            try:
                data, _ = self.sock.recvfrom(256)
                return parse(data)
            except socket.timeout:
                continue
        return None

    def recv_push(self, timeout=1.0):
        old = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        try:
            data, _ = self.sock.recvfrom(256)
            return data.decode("ascii", "replace")
        except socket.timeout:
            return None
        finally:
            self.sock.settimeout(old)

    def close(self):
        self.sock.close()


if __name__ == "__main__":
    # Read-only smoke test against a real/fake camera: version, model, video resolution queries.
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.144.108"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT
    c = C12Client(host, port, timeout=1.0)
    for name in ("VER", "MOD", "VID"):
        t0 = time.time()
        r = c.request(query(name))
        print("%s -> %s (%.0f ms)" % (name, r, (time.time() - t0) * 1000))
    c.close()
