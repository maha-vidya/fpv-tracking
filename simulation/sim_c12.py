#!/usr/bin/env python3
"""Simulated SkyDroid C12 for Gazebo: speaks the TOP protocol on UDP and steers a virtual gimbal toward a Gazebo model.

It answers the same frames as the real camera (GOT, SUM, GAA, PTZ, queries) using simulation/c12/c12_protocol.py, and it pushes the gimbal
attitude (GAA push, "#TPUGCrGAC...") at the requested rate. Tracking starts when it has seen GOT then SUM(01); the gimbal then follows
the Gazebo model `--target` (default `target`) with a rate limit. Without tracking the gimbal holds its angles.

  python3 sim_c12.py --port 5000 --target target --target-height 0.9

Gimbal angle convention (TOP document): yaw RIGHT is positive, pitch UP is positive (looking down is negative), degrees,
relative to the airframe nose (the real camera's reference frame is still to be confirmed on hardware).
It does NOT model the real firmware: it never reports tracking status (the document has none), and it cannot lose the target.
STATUS: simulation helper; the protocol side is covered by simulation/c12/test_c12.py.
"""
import argparse
import math
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "c12"))
import c12_protocol as p  # noqa: E402

WORLD = "iris_runway"


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class World:
    """Latest drone and target poses from Gazebo (gz-transport pose topic)."""
    def __init__(self, target):
        self.target = target
        self.drone = None      # (x, y, z, yaw_deg ENU)
        self.tgt = None        # (x, y, z)
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.node = Node()
        self.node.subscribe(Pose_V, "/world/%s/pose/info" % WORLD, self._cb)

    def _cb(self, msg):
        for q in msg.pose:
            if q.name == "iris_with_gimbal":
                o = q.orientation
                yaw = math.degrees(math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z)))
                self.drone = (q.position.x, q.position.y, q.position.z, yaw)
            elif q.name == self.target:
                self.tgt = (q.position.x, q.position.y, q.position.z)


class Gimbal:
    def __init__(self, a):
        self.a = a
        self.yaw = 0.0
        self.pitch = 0.0
        self.tracking = False
        self.have_point = False
        self.push_hz = 0
        self.push_addr = None
        self.lock = threading.Lock()

    def desired(self, w):
        if not (w.drone and w.tgt):
            return None
        dx, dy = w.tgt[0] - w.drone[0], w.tgt[1] - w.drone[1]
        dist = max(0.1, math.hypot(dx, dy))
        bearing = math.degrees(math.atan2(dy, dx))                       # ENU, counter-clockwise from east
        yaw_rel_ccw = wrap180(bearing - w.drone[3])
        yaw_right = -yaw_rel_ccw                                          # TOP: yaw right positive
        depression = math.degrees(math.atan2(w.drone[2] - (w.tgt[2] + self.a.target_height), dist))
        return max(-90.0, min(90.0, yaw_right)), max(-90.0, min(10.0, -depression))

    def step(self, w, dt):
        with self.lock:
            if not self.tracking:
                return
            d = self.desired(w)
            if d is None:
                return
            m = self.a.rate * dt
            self.yaw += max(-m, min(m, d[0] - self.yaw))
            self.pitch += max(-m, min(m, d[1] - self.pitch))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5000)
    ap.add_argument("--target", default="target", help="Gazebo model the gimbal tracks")
    ap.add_argument("--target-height", type=float, default=0.9, help="height of the aimed point above the model origin, m")
    ap.add_argument("--rate", type=float, default=60.0, help="gimbal slew limit, deg/s")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    w = World(a.target)
    g = Gimbal(a)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((a.host, a.port))
    sock.settimeout(0.02)
    print("sim_c12: udp://%s:%d  target=%s" % (a.host, a.port, a.target), flush=True)

    last = time.time()
    next_push = 0.0
    while True:
        now = time.time()
        g.step(w, now - last)
        last = now
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            data = None
        if data:
            try:
                f = p.parse(data)
            except ValueError:
                sock.sendto(b"#TPMU2wERE!!30", addr)
                continue
            with g.lock:
                ident, d = f["ident"], f["data"]
                if ident == "GAA" and len(d) == 2:
                    g.push_hz = int(d, 16)
                    g.push_addr = addr if g.push_hz else None
                elif ident == "GOT" and len(d) == 8:
                    g.have_point = True
                elif ident == "SUM":
                    g.tracking = (d == "01") and g.have_point
                    if d == "00":
                        g.have_point = False
                elif ident == "PTZ" and d == "05":
                    g.yaw = g.pitch = 0.0
                    g.tracking = False
                if a.verbose:
                    print("rx", f["ctrl"], ident, d, "| tracking:", g.tracking, flush=True)
            if f["ctrl"] == "w":                                          # control commands are echoed with swapped addresses
                sock.sendto(p.build(f["dst"], f["src"], "w", f["ident"], f["data"], header=f["header"]).encode(), addr)
            elif f["ident"] == "VER":
                sock.sendto(p.build(f["dst"], f["src"], "r", "VER", "V1.0.0", header="#tp").encode(), addr)
        if g.push_hz and g.push_addr and now >= next_push:
            next_push = now + 1.0 / g.push_hz
            with g.lock:
                body = "#TPUGCrGAC" + p._s16(round(g.yaw * 100)) + p._s16(round(g.pitch * 100)) + p._s16(0)
            sock.sendto((body + p.crc(body)).encode(), g.push_addr)


if __name__ == "__main__":
    main()
