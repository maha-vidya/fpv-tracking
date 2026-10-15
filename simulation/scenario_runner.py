#!/usr/bin/env python3
"""Scripted Gazebo scenarios for the drone-with-camera follow/rescue project.

Spawns the actors a scenario needs (a person-like target in a red jacket, an orange-jacket lookalike, a wall
that occludes) and moves them on a timeline, while firing pilot/environment events (mode change, RC switch,
wind, link loss, video freeze). The drone, its gimbal camera and the follower are NOT touched by this script.

  python3 scenario_runner.py --list
  python3 scenario_runner.py S3_lookalike                 # run one scenario (Ctrl+C stops; actors stay)
  python3 scenario_runner.py S4_occlusion --kind box      # boxes instead of people
  python3 scenario_runner.py S1_static --prepare          # only spawn + place the actors, then exit
  python3 scenario_runner.py S8_dash --start-delay 30     # wait 30 s (time to drag/lock) before the timeline starts

Frame: Gazebo ENU, the drone hovers at the origin facing north (+y) after simulation/takeoff.py; camera 640x480,
horizontal FOV 2.0 rad (about 115 deg; ardupilot_gazebo gimbal_small_3d model.sdf), so at 15 m ahead the view is
about +-23 m wide. Run simulation/sim_up.sh first. Drone truth is logged from Gazebo's pose topic.
Truth CSV: $RUN_DIR/scenario_<name>_<time>.csv  (t, actor poses, drone pose).

STATUS: written; see tracker/docs/RESCUE.md for what has and has not been exercised live.
"""
import argparse
import csv
import math
import os
import subprocess
import sys
import threading
import time

import move_target as mt
import sim_events as ev

WORLD = mt.WORLD
HERE = os.path.dirname(os.path.abspath(__file__))
RUN_DIR = os.environ.get("RUN_DIR", os.path.expanduser("~/sim_run"))
ALL_ACTORS = ("target", "lookalike", "occluder", "target_red", "lookalike_orange")

RED, ORANGE = (0.85, 0.05, 0.05), (1.0, 0.45, 0.0)


# ---- actor SDF -------------------------------------------------------------------------------
def _mat(c):
    return "<ambient>%f %f %f 1</ambient><diffuse>%f %f %f 1</diffuse>" % (c + c)


def _vis(name, geom, z, c):
    return ('<visual name="%s"><pose>0 0 %f 0 0 0</pose><geometry>%s</geometry><material>%s</material></visual>'
            % (name, z, geom, _mat(c)))


def sdf_person(name, jacket, x, y):
    """About 1.75 m tall: dark legs, coloured torso (the jacket), skin-tone head. Feet on the ground."""
    body = (_vis("legs", "<cylinder><radius>0.12</radius><length>0.85</length></cylinder>", 0.425, (0.1, 0.1, 0.15)) +
            _vis("torso", "<cylinder><radius>0.22</radius><length>0.65</length></cylinder>", 1.175, jacket) +
            _vis("head", "<sphere><radius>0.12</radius></sphere>", 1.60, (0.85, 0.65, 0.5)))
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0 0 0 0</pose>'
            '<link name="l">%s</link></model></sdf>' % (name, x, y, body))


def sdf_box(name, color, x, y, size=1.0):
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f %f 0 0 0</pose><link name="l">%s</link>'
            '</model></sdf>' % (name, x, y, size / 2, _vis("v", "<box><size>%f %f %f</size></box>" % (size, size, size), 0, color)))


def sdf_wall(name, x, y, w=4.0, h=3.0, d=0.3):
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f %f 0 0 0</pose><link name="l">%s</link>'
            '</model></sdf>' % (name, x, y, h / 2, _vis("v", "<box><size>%f %f %f</size></box>" % (w, d, h), 0, (0.45, 0.45, 0.5))))


def make_sdf(kind, actor, x, y):
    if actor == "target":
        return sdf_box(actor, RED, x, y) if kind == "box" else sdf_person(actor, RED, x, y)
    if actor == "lookalike":
        return sdf_box(actor, ORANGE, x, y) if kind == "box" else sdf_person(actor, ORANGE, x, y)
    return sdf_wall(actor, x, y)


def base_z(kind, actor):
    """z used for set_pose: the model origin (people stand on the ground, boxes/walls are centred)."""
    if actor == "occluder":
        return 1.5
    return 0.5 if kind == "box" else 0.0


# ---- Gazebo helper (spawn any SDF, remove models) ---------------------------------------------
class SimGz(mt.Gz):
    def spawn_sdf(self, sdf):
        if self.node:
            req = self.EF()
            req.sdf = sdf
            ok, rep = self.node.request("/world/%s/create" % WORLD, req, self.EF, self.Boolean, self.timeout)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/create" % WORLD, "--reqtype", "gz.msgs.EntityFactory",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "sdf: '%s'" % sdf],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout

    def remove(self, name):
        r = subprocess.run(["gz", "service", "-s", "/world/%s/remove" % WORLD, "--reqtype", "gz.msgs.Entity",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", 'name: "%s" type: 2' % name],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout


# ---- scenario library: actor path functions f(t) -> (x, y), plus timed events ------------------
def lateral(cx, cy, amp, period, phase=0.0, depth_amp=0.0):
    return lambda t: (cx + amp * math.sin(2 * math.pi * t / period + phase),
                      cy + depth_amp * math.sin(2 * math.pi * t / (period * 1.7) + phase))


def static(x, y):
    return lambda t: (x, y)


def piecewise(points):
    """points: [(t, x, y), ...] linear in between, constant outside."""
    def f(t):
        if t <= points[0][0]:
            return points[0][1:]
        for (t0, x0, y0), (t1, x1, y1) in zip(points, points[1:]):
            if t <= t1:
                k = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
                return (x0 + k * (x1 - x0), y0 + k * (y1 - y0))
        return points[-1][1:]
    return f


SCENARIOS = {
    "S1_static": dict(
        desc="Static target 15 m ahead: basic lock and follow.", duration=60,
        actors={"target": static(0, 15)}, events=[],
        expect="Lock holds; yaw/forward/vertical commands settle; no commands outside the ceilings."),
    "S2_walk": dict(
        desc="Person walks side to side (about 0.5 m/s) with a slight forward/back drift.", duration=90,
        actors={"target": lateral(0, 15, 5, 60, depth_amp=2)}, events=[],
        expect="Lock holds; drone yaws/translates to keep the target near the image centre."),
    "S3_lookalike": dict(
        desc="Orange-jacket lookalike crosses the red target (further away, opposite phase).", duration=90,
        actors={"target": lateral(0, 15, 5, 60), "lookalike": lateral(0, 18, 5, 60, phase=math.pi)}, events=[],
        expect="Count identity switches (lock on the wrong person). Phase 1 CANNOT detect a wrong lock; this measures the risk."),
    "S4_occlusion": dict(
        desc="A wall crosses between the drone and the target for about 4 s.", duration=60,
        actors={"target": static(0, 18), "occluder": piecewise([(0, -12, 9), (12, -12, 9), (20, 12, 9), (60, 12, 9)])},
        events=[],
        expect="LOCKED -> COASTING -> LOST as designed (zero velocity); no steering on a stale box; relock needs the operator."),
    "S5_leaves_fov": dict(
        desc="Target walks out of the camera's view (about 30 m to the side) and comes back.", duration=100,
        actors={"target": piecewise([(0, 0, 15), (5, 0, 15), (30, 30, 15), (45, 30, 15), (70, 0, 15), (100, 0, 15)])},
        events=[],
        expect="Lost-target handling: hold/zero velocity, LOCK LOST banner, auto-unlock after 5 s, new operator lock needed."),
    "S6_fast": dict(
        desc="Fast target (about 4 m/s peak lateral).", duration=60,
        actors={"target": lateral(0, 15, 8, 12)}, events=[],
        expect="Shows tracker/control limits at 5 fps; expect lock loss at some speed. Record where it breaks."),
    "S7_scale": dict(
        desc="Target approaches and recedes (10-30 m ahead) to test scale change.", duration=80,
        actors={"target": (lambda t: (0.0, 20 + 10 * math.sin(2 * math.pi * t / 40)))},
        events=[],
        expect="Forward command follows apparent size; box scale handled by CSRT."),
    "S8_dash": dict(
        desc="Rescue dash: static target 25 m away; the pilot switch (RC ch9) is pressed twice at t=20 s.", duration=60,
        actors={"target": static(0, 25)},
        events=[(20.0, "rc", dict(channel=9, pwm=1900, seconds=1.0)),
                (21.5, "rc", dict(channel=9, pwm=1100, seconds=0.6)),
                (22.5, "rc", dict(channel=9, pwm=1900, seconds=1.0))],
        expect="Interlocks: first press arms, second within 3 s fires; fast approach; stop; measure stop distance and overshoot."),
    "S9_link_loss": dict(
        desc="MAVLink link to the companion frozen for 6 s while following.", duration=70,
        actors={"target": lateral(0, 15, 4, 50)},
        events=[(20.0, "fault", dict(kind="link-loss", seconds=6))],
        expect="LINK_LOST within 2 s, zero velocity, no automatic relock; operator must lock again."),
    "S10_video_freeze": dict(
        desc="Video frozen for 5 s while following.", duration=70,
        actors={"target": lateral(0, 15, 4, 50)},
        events=[(20.0, "fault", dict(kind="video-freeze", seconds=5))],
        expect="Box goes stale -> COASTING/LOST, zero velocity; resumes only via a new lock."),
    "S11_pilot_override": dict(
        desc="Pilot switches to LOITER at t=20 s and back to GUIDED at t=35 s.", duration=60,
        actors={"target": lateral(0, 15, 4, 50)},
        events=[(20.0, "mode", dict(name="LOITER")), (35.0, "mode", dict(name="GUIDED"))],
        expect="Follower stops commanding on leaving GUIDED and does NOT resume velocity when GUIDED returns."),
    "S12_wind": dict(
        desc="5 m/s wind from the east with turbulence while following a walking target.", duration=90,
        actors={"target": lateral(0, 15, 5, 60)},
        events=[(0.0, "wind", dict(speed=5, direction=90)), (0.5, "param", dict(name="SIM_WIND_TURB", value=2)),
                (85.0, "wind", dict(speed=0))],
        expect="Control stays inside the ceilings and the lock holds despite disturbance."),
}


def fire(event, sim_cfg):
    kind, a = event
    try:
        if kind == "rc":
            m = ev.connect(sim_cfg["mav"]); ev.hold_rc(m, a["channel"], a["pwm"], a["seconds"])
        elif kind == "mode":
            m = ev.connect(sim_cfg["mav"]); print("   mode", a["name"], "ok" if ev.set_mode(m, a["name"]) else "NOT confirmed")
        elif kind == "wind":
            m = ev.connect(sim_cfg["mav"]); ev.wind(m, a["speed"], a.get("direction"))
        elif kind == "param":
            m = ev.connect(sim_cfg["mav"]); ev.set_param(m, a["name"], a["value"])
        elif kind == "fault":
            subprocess.call([os.path.join(HERE, "fault.sh"), a["kind"], str(a["seconds"])])
    except SystemExit as e:
        print("   [event %s failed] %s" % (kind, e))
    except Exception as e:                                    # noqa: BLE001
        print("   [event %s failed] %r" % (kind, e))


class DroneTruth:
    """Latest iris pose from Gazebo's pose topic (best effort; None if the bindings fail)."""
    def __init__(self):
        self.pose = None
        self.sim_t = None                                     # Gazebo simulation time (s) of the latest pose message
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_v_pb2 import Pose_V
            self.node = Node()
            self.node.subscribe(Pose_V, "/world/%s/pose/info" % WORLD, self._cb)
        except Exception as e:                                # noqa: BLE001
            print("[info] drone truth unavailable: %r" % e)

    def _cb(self, msg):
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == "iris_with_gimbal":
                self.pose = (p.position.x, p.position.y, p.position.z)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--kind", choices=("person", "box"), default="person")
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--prepare", action="store_true", help="spawn and place the actors at t=0, then exit")
    ap.add_argument("--start-delay", type=float, default=0.0, help="seconds to wait before the timeline starts")
    ap.add_argument("--duration", type=float, default=0.0, help="override the scenario duration")
    ap.add_argument("--mav", default="tcp:127.0.0.1:5762")
    ap.add_argument("--range", type=float, default=10.0,
                    help="distance of the target ahead of the drone, m (the scenarios are written for 15; people are tiny "
                         "at 15 m with this wide lens, so the default is 10)")
    a = ap.parse_args()
    if a.list or not a.scenario:
        for k, s in SCENARIOS.items():
            print("%-18s %3ds  %s" % (k, s["duration"], s["desc"]))
        return
    if a.scenario not in SCENARIOS:
        sys.exit("unknown scenario %r (use --list)" % a.scenario)
    sc = SCENARIOS[a.scenario]
    gz = SimGz(timeout_ms=300)
    dy = a.range - 15.0
    present = gz.models()
    for n in ALL_ACTORS:                                       # clear anything left from earlier runs
        if n in present:
            gz.remove(n)
    time.sleep(0.5)
    for name, f in sc["actors"].items():
        x, y = f(0.0)
        y += dy
        ok = gz.spawn_sdf(make_sdf(a.kind, name, x, y))
        print("[spawn] %-9s at (%.1f, %.1f): %s" % (name, x, y, "ok" if ok else "FAILED"))
        if not ok:
            sys.exit("could not spawn %s (is Gazebo running?)" % name)
    print("[scenario] %s: %s\n[expect]   %s" % (a.scenario, sc["desc"], sc["expect"]))
    if a.prepare:
        return
    if a.start_delay > 0:
        print("[scenario] starting in %.0f s (lock the target now)" % a.start_delay)
        time.sleep(a.start_delay)

    truth = DroneTruth()
    for _ in range(40):                                       # wait (up to 4 s) for the first pose message = sim clock
        if truth.sim_t is not None:
            break
        time.sleep(0.1)
    os.makedirs(RUN_DIR, exist_ok=True)
    path = os.path.join(RUN_DIR, "scenario_%s_%s.csv" % (a.scenario, time.strftime("%H%M%S")))
    fh = open(path, "w", newline="")
    cw = csv.writer(fh)
    names = list(sc["actors"])
    cw.writerow(["t"] + ["%s_%s" % (n, k) for n in names for k in "xy"] + ["drone_x", "drone_y", "drone_z"])
    pending = sorted(sc["events"], key=lambda e: e[0])
    cfg = {"mav": a.mav}
    dur = a.duration or sc["duration"]
    dt = 1.0 / a.rate
    sent = late = 0
    last_status = 0.0
    t0 = time.time()
    sim0 = truth.sim_t
    print("[scenario] running %.0f s of %s time, truth log %s (Ctrl+C stops)" %
          (dur, "SIMULATION" if sim0 is not None else "wall-clock", path))
    try:
        while True:
            wall = time.time() - t0
            t = (truth.sim_t - sim0) if (sim0 is not None and truth.sim_t is not None) else wall
            if t > dur:
                break
            if wall > 4 * dur + 30:
                print("[scenario] ABORT: %.0f s of wall time but only %.1f s of simulation time (real-time factor %.2f): "
                      "the simulator is stalled or far too slow" % (wall, t, t / wall if wall else 0))
                break
            if wall - last_status >= 10:
                last_status = wall
                print("[%5.1fs] sim time, %.0f s wall, real-time factor about %.2f" % (t, wall, t / wall if wall else 0))
            row = [round(t, 2)]
            for n in names:
                x, y = sc["actors"][n](t)
                y += dy
                sent += 1
                if not gz.set_pose(n, x, y, base_z(a.kind, n)):
                    late += 1                                 # the reply timed out; the pose is normally still applied
                row += [round(x, 2), round(y, 2)]
            while pending and pending[0][0] <= t:
                _, kind, args = pending.pop(0)
                print("[%5.1fs] event %s %s" % (t, kind, args))
                threading.Thread(target=fire, args=((kind, args), cfg), daemon=True).start()
            dp = truth.pose or (float("nan"),) * 3
            cw.writerow(row + [round(v, 2) for v in dp])
            time.sleep(max(0.0, dt - ((time.time() - t0) - wall)))
    except KeyboardInterrupt:
        print("\n[scenario] stopped by user")
    fh.close()
    print("[scenario] finished; truth log: %s" % path)
    if late:
        print("[note] %d of %d pose updates got no reply within 300 ms (Gazebo busy; a check on one run showed the poses "
              "were still applied). If the simulation is slow, close the Gazebo GUI or other heavy programs." % (late, sent))


if __name__ == "__main__":
    main()
