#!/usr/bin/env python3
"""Fake Skydroid C12 for testing control code WITHOUT the real camera (Python 3.6+).

Speaks the TOP protocol on UDP (default port 5000 -- use another port if something owns it).
It models only what the doc describes: gimbal yaw/pitch (speed + angle commands, limits),
the GAA attitude push, GOT/SUM tracking flags, REC, and a few queries. It is NOT a model of
the real firmware: it never reports tracking status because the document has no such message.

  python3 fake_c12.py --port 15000 --verbose
"""
import argparse
import socket
import threading
import time

import c12_protocol as p

YAW_LIM = 90.0
PITCH_MIN, PITCH_MAX = -90.0, 10.0   # manual: pitch -90..+10 (TOP doc says +-90; we use the tighter one)


class FakeC12:
    def __init__(self, verbose=False):
        self.lock = threading.Lock()
        self.yaw = 0.0
        self.pitch = 0.0
        self.yaw_rate = 0.0       # deg/s, from GSM
        self.pitch_rate = 0.0
        self.target = None        # (x, y) from GOT
        self.tracking = False
        self.recording = False
        self.mode = "follow"
        self.push_hz = 0
        self.push_addr = None
        self.log = []             # every valid frame received: (ident, data)
        self.verbose = verbose
        self.fc_att = None
        self.fc_att_on = False
        self._stop = threading.Event()

    # -- state integration -----------------------------------------------------------------
    def _tick(self, dt):
        with self.lock:
            self.yaw = max(-YAW_LIM, min(YAW_LIM, self.yaw + self.yaw_rate * dt))
            self.pitch = max(PITCH_MIN, min(PITCH_MAX, self.pitch + self.pitch_rate * dt))

    def _loop(self, sock):
        last = time.time()
        next_push = 0.0
        while not self._stop.is_set():
            now = time.time()
            self._tick(now - last)
            last = now
            with self.lock:
                hz, addr = self.push_hz, self.push_addr
                y, pi = self.yaw, self.pitch
            if hz and addr and now >= next_push:
                body = "#TPUGCrGAC" + p._s16(round(y * 100)) + p._s16(round(pi * 100)) + p._s16(0)
                try:
                    sock.sendto((body + p.crc(body)).encode("ascii"), addr)
                except OSError:
                    pass
                next_push = now + 1.0 / hz
            time.sleep(0.005)

    # -- command handling ------------------------------------------------------------------
    def handle(self, raw, addr):
        """Return the reply string for one datagram (or None)."""
        try:
            f = p.parse(raw)
        except ValueError as e:
            if self.verbose:
                print("bad frame:", e)
            return self._error(raw)
        ident, data, ctrl = f["ident"], f["data"], f["ctrl"]
        if self.verbose:
            print("RX %s %s %s" % (ctrl, ident, data))
        with self.lock:
            self.log.append((ident, data))
            if ctrl == "w":
                ok = self._write(ident, data, addr)
                if not ok:
                    return self._error(raw)
                return p.build(f["dst"], f["src"], "w", ident, data, header=f["header"])
            return self._read(f)

    def _write(self, ident, data, addr):
        if ident == "PTZ":
            return self._ptz(data)
        if ident == "GSM" and len(data) == 4:
            self.yaw_rate = _s8(data[0:2]) * 0.5
            self.pitch_rate = _s8(data[2:4]) * 0.5
            return True
        if ident == "GAM" and len(data) == 12:
            self.yaw = max(-YAW_LIM, min(YAW_LIM, p.from_s16(data[0:4]) / 100.0))
            self.pitch = max(PITCH_MIN, min(PITCH_MAX, p.from_s16(data[6:10]) / 100.0))
            self.yaw_rate = self.pitch_rate = 0.0
            return True
        if ident == "GAA" and len(data) == 2:
            self.push_hz = int(data, 16)
            self.push_addr = addr if self.push_hz else None
            return True
        if ident == "GOT" and len(data) == 8:
            x, y = int(data[:4], 16), int(data[4:], 16)
            if x > 1280 or y > 720:
                return False
            self.target = (x, y)
            return True
        if ident == "SUM" and data in ("00", "01"):
            self.tracking = data == "01" and self.target is not None
            if data == "00":
                self.target = None
            return True
        if ident == "FAE" and data in ("00", "01"):
            self.fc_att_on = data == "01"
            return True
        if ident == "FAI" and len(data) == 12:
            self.fc_att = tuple(p.from_s16(data[i:i + 4]) / 100.0 for i in (0, 4, 8))
            return True
        if ident == "REC" and data in ("00", "01", "0A"):
            self.recording = {"00": False, "01": True, "0A": not self.recording}[data]
            return True
        return False

    def _ptz(self, data):
        step = 5.0
        if data == "00":
            self.yaw_rate = self.pitch_rate = 0.0
        elif data == "01":
            self.pitch = min(PITCH_MAX, self.pitch + step)
        elif data == "02":
            self.pitch = max(PITCH_MIN, self.pitch - step)
        elif data == "03":
            self.yaw = max(-YAW_LIM, self.yaw - step)
        elif data == "04":
            self.yaw = min(YAW_LIM, self.yaw + step)
        elif data == "05":
            self.yaw = self.pitch = 0.0
            self.yaw_rate = self.pitch_rate = 0.0
        elif data in ("06", "07"):
            self.mode = "follow" if data == "06" else "lock"
        elif data == "08":
            self.mode = "lock" if self.mode == "follow" else "follow"
        else:
            return False
        return True

    def _read(self, f):
        ident = f["ident"]
        if ident == "VER":
            val = "V1.0.0"
        elif ident == "MOD":
            val = "C12"
        elif ident == "VID":
            val = "01"   # 1080p placeholder
        elif ident == "REC":
            val = "01" if self.recording else "00"
        else:
            return self._error(None)
        return p.build(f["dst"], f["src"], "r", ident, val, header="#tp")

    @staticmethod
    def _error(raw):
        # Doc: invalid command -> "#TPdd2wERE!!RR" with src/dst swapped (example #TPMU2wERE!!30).
        body = "#TPMU2wERE!!"
        return body + p.crc(body)

    # -- server ------------------------------------------------------------------------------
    def serve(self, host="127.0.0.1", port=5000, ready=None):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.settimeout(0.1)
        threading.Thread(target=self._loop, args=(sock,), daemon=True).start()
        if ready:
            ready.set()
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            reply = self.handle(data, addr)
            if reply:
                sock.sendto(reply.encode("ascii"), addr)
        sock.close()

    def stop(self):
        self._stop.set()


def _s8(h):
    v = int(h, 16)
    return v - 256 if v >= 128 else v


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    print("fake C12 on udp://%s:%d (Ctrl+C to stop)" % (a.host, a.port))
    try:
        FakeC12(a.verbose).serve(a.host, a.port)
    except KeyboardInterrupt:
        pass
