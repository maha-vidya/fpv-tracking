#!/usr/bin/env python3
"""Move a red target box (and an orange lookalike) around in the running Gazebo world.

Spawns `target_red` / `lookalike_orange` if they are missing (a Gazebo restart removes them),
then moves them along slow side-to-side paths so that they CROSS in the camera image (the
lookalike is further away, so it is smaller). Ctrl+C stops the motion and leaves the boxes where
they are. Only touches those two models; never the drone.

Default world / camera layout (drone hovers at the origin facing north = +y, camera looks forward):
  centre (0, 15), red amplitude 5 m, period 60 s  ->  max speed ~0.5 m/s.

  python3 move_target.py                       # red + lookalike
  python3 move_target.py --no-lookalike --amp 3 --period 40
  python3 move_target.py --once 0 15           # just place the red box at x=0,y=15 and exit

STATUS: written but NOT yet run against a live Gazebo (the sim was off when this was written).
Tries gz-transport Python first, falls back to the `gz service` CLI.
"""
import argparse
import math
import subprocess
import sys
import time

WORLD = "iris_runway"
RED = ("target_red", "1 0 0 1")
ORANGE = ("lookalike_orange", "1 0.4 0 1")


def _sdf(name, rgba, x, y):
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0.5 0 0 0</pose>'
            '<link name="l"><visual name="v"><geometry><box><size>1 1 1</size></box></geometry>'
            '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual></link></model></sdf>'
            % (name, x, y, rgba, rgba))


class Gz:
    def __init__(self, timeout_ms=1000):
        self.timeout = timeout_ms
        self.node = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            self.node, self.Pose, self.Boolean, self.EF = Node(), Pose, Boolean, EntityFactory
        except Exception as e:                                  # noqa: BLE001
            print("[info] gz python bindings unavailable (%s); using the gz CLI (slower)" % e)

    def models(self):
        out = subprocess.run(["gz", "model", "--list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                             universal_newlines=True, timeout=20).stdout
        return [l.strip()[2:] for l in out.splitlines() if l.strip().startswith("- ")]

    def spawn(self, name, rgba, x, y):
        sdf = _sdf(name, rgba, x, y)
        if self.node:
            req = self.EF()
            req.sdf = sdf
            ok, rep = self.node.request("/world/%s/create" % WORLD, req, self.EF, self.Boolean, self.timeout)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/create" % WORLD, "--reqtype", "gz.msgs.EntityFactory",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "sdf: '%s'" % sdf],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout

    def set_pose(self, name, x, y, z=0.5):
        if self.node:
            req = self.Pose()
            req.name = name
            req.position.x, req.position.y, req.position.z = x, y, z
            req.orientation.w = 1.0
            ok, rep = self.node.request("/world/%s/set_pose" % WORLD, req, self.Pose, self.Boolean, self.timeout)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/set_pose" % WORLD, "--reqtype", "gz.msgs.Pose",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req",
                            'name: "%s" position {x: %f y: %f z: %f} orientation {w: 1}' % (name, x, y, z)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cx", type=float, default=0.0, help="path centre x (east), m")
    ap.add_argument("--cy", type=float, default=15.0, help="path centre y (north, ahead of the drone), m")
    ap.add_argument("--amp", type=float, default=5.0, help="side-to-side amplitude, m")
    ap.add_argument("--depth-amp", type=float, default=2.0, help="forward/back wobble amplitude, m")
    ap.add_argument("--period", type=float, default=60.0, help="seconds per side-to-side cycle")
    ap.add_argument("--no-lookalike", action="store_true")
    ap.add_argument("--lookalike-depth", type=float, default=3.0, help="lookalike is this much further away, m")
    ap.add_argument("--rate", type=float, default=5.0, help="pose updates per second (video is ~5 fps)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--ellipse", nargs=2, type=float, metavar=("A", "B"),
                    help="FPV chase test: drive the red box round an ellipse with semi-axes A (east) x B (north) m around the centre, "
                         "at --speed m/s (the lookalike stays parked)")
    ap.add_argument("--speed", type=float, default=4.0, help="m/s along the --ellipse")
    ap.add_argument("--once", nargs=2, type=float, metavar=("X", "Y"), help="place the red box and exit")
    a = ap.parse_args()

    gz = Gz()
    have = gz.models()
    want = [RED] + ([] if a.no_lookalike or a.once else [ORANGE])
    for name, rgba in want:
        if name not in have:
            ok = False
            for _ in range(30):
                ok = gz.spawn(name, rgba, a.cx, a.cy)
                if ok:
                    break
                time.sleep(1.0)
            print("[spawn] %s: %s" % (name, "ok" if ok else "FAILED"), flush=True)
            if not ok:
                sys.exit("could not spawn %s: is Gazebo running with world %s?" % (name, WORLD))
    if a.once:
        print("placed:", gz.set_pose(RED[0], a.once[0], a.once[1]))
        return

    print("moving %s%s  centre=(%.1f,%.1f) amp=%.1f period=%.0fs  (Ctrl+C to stop)" %
          (RED[0], "" if a.no_lookalike else " + " + ORANGE[0], a.cx, a.cy, a.amp, a.period))
    t0 = time.time()
    dt = 1.0 / a.rate
    ell_th = 0.0
    fails = 0
    try:
        while True:
            t = time.time() - t0
            if a.duration and t > a.duration:
                break
            if a.ellipse:                                   # constant SPEED along the ellipse (arc length, not angle)
                ea, eb = a.ellipse
                th = ell_th
                ok = gz.set_pose(RED[0], a.cx + ea * math.sin(th), a.cy + eb * math.cos(th))
                ell_th += a.speed * dt / max(0.1, math.hypot(ea * math.cos(th), eb * math.sin(th)))
                fails = 0 if ok else fails + 1
                time.sleep(max(0.0, dt - ((time.time() - t0) - t)))
                continue
            w = 2 * math.pi * t / a.period
            ok = gz.set_pose(RED[0], a.cx + a.amp * math.sin(w), a.cy + a.depth_amp * math.sin(w / 1.7))
            if not a.no_lookalike:
                ok = gz.set_pose(ORANGE[0], a.cx + a.amp * math.sin(w + math.pi),
                                 a.cy + a.lookalike_depth + a.depth_amp * math.sin(w / 1.7 + 1.0)) and ok
            fails = 0 if ok else fails + 1
            if fails == 20:
                print("[warn] 20 failed pose updates in a row: is Gazebo still running?")
            time.sleep(max(0.0, dt - ((time.time() - t0) - t)))
    except KeyboardInterrupt:
        pass
    print("stopped (boxes left where they are)")


if __name__ == "__main__":
    main()
