#!/usr/bin/env python3
"""Spawn and maintain static and slow-moving cars in different sizes in the Gazebo simulation.

Sizes represented:
  - Mini / Micro Car (3.0 m x 1.5 m, small city car)
  - Compact Hatchback (3.9 m x 1.6 m, OpenRobotics Fuel mesh: Hatchback red / blue)
  - Mid-size SUV (4.8 m x 1.9 m, OpenRobotics Fuel mesh: SUV)
  - Large Pickup Truck (5.8 m x 2.1 m, OpenRobotics Fuel mesh: Pickup)
  - Extra Large Box Truck / Delivery Van (7.2 m x 2.4 m)

Static cars (parked in view across different depths):
  - static_minicar_yellow:  x=-4.0, y=13.0 (small, 3.0 m)
  - static_hatchback_red:   x=5.0,  y=15.0 (compact, 3.9 m)
  - static_suv_silver:      x=-7.0, y=20.0 (mid-size, 4.8 m)
  - static_pickup_white:    x=7.5,  y=25.0 (large, 5.8 m)
  - static_van_blue:        x=-10.0,y=30.0 (extra large, 6.5 m)

Slow-moving cars (crawling/cruising at 1.2 - 2.2 m/s, facing direction of travel):
  - slow_hatchback_blue:    compact, 1.4 m/s along y=17.0 m, x in [-14, +14] m
  - slow_suv_black:         mid-size, 1.8 m/s along y=23.0 m, x in [-18, +18] m
  - slow_pickup_red:        large, 2.2 m/s along x=10.5 m, y in [12, 36] m
  - slow_truck_box:         extra large, 1.2 m/s along x=-13.0 m, y in [14, 34] m

Usage:
  python3 simulation/sim_cars.py               # run both static and slow-moving cars
  python3 simulation/sim_cars.py --static-only # place static cars only and exit
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import time

WORLD = "iris_runway"
LOCK_PATH = "/tmp/sim_cars.lock"
FUEL_BASE = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/%s"


def _smootherstep(x):
    """C2-continuous ease for smooth acceleration/deceleration at turnaround."""
    x = max(0.0, min(1.0, x))
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


class Gz:
    def __init__(self, timeout_ms=1000):
        self.timeout = timeout_ms
        self.node = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            from gz.msgs10.entity_pb2 import Entity
            self.node, self.Pose, self.Boolean, self.EF, self.Entity = Node(), Pose, Boolean, EntityFactory, Entity
        except Exception as e:
            print("[sim_cars] gz transport python bindings unavailable (%s); using gz CLI fallback" % e)

    def models(self):
        try:
            out = subprocess.run(["gz", "model", "--list"], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                 universal_newlines=True, timeout=15).stdout
            return [line.strip()[2:] for line in out.splitlines() if line.strip().startswith("- ")]
        except Exception:
            return []

    def spawn_sdf(self, sdf_str, timeout_ms=3000):
        if self.node:
            req = self.EF()
            req.sdf = sdf_str
            ok, rep = self.node.request("/world/%s/create" % WORLD, req, self.EF, self.Boolean, timeout_ms)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/create" % WORLD, "--reqtype", "gz.msgs.EntityFactory",
                            "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", "sdf: '%s'" % sdf_str],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=10)
        return "true" in r.stdout

    def spawn_fuel(self, name, fuel_model, x, y, z=0.0, yaw=0.0):
        uri = FUEL_BASE % fuel_model
        sdf = ('<sdf version="1.9"><include><uri>%s</uri><name>%s</name><pose>%f %f %f 0 0 %f</pose>'
               '<static>true</static></include></sdf>' % (uri, name, x, y, z, yaw))
        return self.spawn_sdf(sdf, timeout_ms=5000)

    def set_pose(self, name, x, y, z=0.0, yaw=0.0):
        qw, qz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        if self.node:
            req = self.Pose()
            req.name = name
            req.position.x, req.position.y, req.position.z = x, y, z
            req.orientation.w, req.orientation.z = qw, qz
            ok, rep = self.node.request("/world/%s/set_pose" % WORLD, req, self.Pose, self.Boolean, self.timeout)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/set_pose" % WORLD, "--reqtype", "gz.msgs.Pose",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req",
                            'name: "%s" position {x: %f y: %f z: %f} orientation {w: %f z: %f}' % (name, x, y, z, qw, qz)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout

    def remove_model(self, name):
        if self.node:
            req = self.Entity()
            req.name = name
            req.type = self.Entity.MODEL
            ok, rep = self.node.request("/world/%s/remove" % WORLD, req, self.Entity, self.Boolean, 1000)
            return ok and rep.data
        subprocess.run(["gz", "service", "-s", "/world/%s/remove" % WORLD, "--reqtype", "gz.msgs.Entity",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req", 'name: "%s" type: 2' % name],
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def _vis(name, size, xyz, rgb, rpy=(0, 0, 0), shape="box"):
    geo = ("<box><size>%f %f %f</size></box>" % size) if shape == "box" else \
          ("<cylinder><radius>%f</radius><length>%f</length></cylinder>" % size)
    return ('<visual name="%s"><pose>%f %f %f %f %f %f</pose><geometry>%s</geometry>'
            '<material><ambient>%s 1</ambient><diffuse>%s 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual>'
            % ((name,) + tuple(xyz) + tuple(rpy) + (geo, rgb, rgb)))


def _wheels(length, width, r):
    out = []
    xs = [length * 0.32, -length * 0.32]
    for i, x in enumerate(xs):
        for j, y in enumerate((width / 2, -width / 2)):
            out.append(_vis("w%d%d" % (i, j), (r, 0.24), (x, y, r), "0.05 0.05 0.05", (math.pi / 2, 0, 0), "cyl"))
    return out


def make_minicar_sdf(name, rgb, x, y, yaw=0.0):
    """Mini city car: 3.0 m x 1.5 m x 1.3 m."""
    v = [_vis("body", (3.0, 1.5, 0.65), (0, 0, 0.55), rgb),
         _vis("cabin", (1.6, 1.35, 0.55), (-0.1, 0, 1.1), rgb),
         _vis("glass", (1.65, 1.37, 0.35), (-0.1, 0, 1.08), "0.08 0.1 0.14")] + _wheels(3.0, 1.5, 0.28)
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose>'
            '<link name="l">%s</link></model></sdf>' % (name, x, y, yaw, "".join(v)))


def make_van_sdf(name, rgb, x, y, yaw=0.0):
    """Delivery van: 5.2 m x 2.0 m x 2.0 m."""
    v = [_vis("body", (5.2, 2.0, 1.9), (0, 0, 1.3), rgb),
         _vis("windshield", (0.06, 1.8, 0.6), (2.6, 0, 1.75), "0.08 0.1 0.14"),
         _vis("side_glass", (2.6, 2.02, 0.45), (1.0, 0, 1.75), "0.08 0.1 0.14")] + _wheels(5.2, 2.0, 0.36)
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose>'
            '<link name="l">%s</link></model></sdf>' % (name, x, y, yaw, "".join(v)))


def make_truck_sdf(name, rgb, cargo_rgb, x, y, yaw=0.0):
    """Large box truck: 7.2 m x 2.4 m x 2.8 m."""
    v = [_vis("cab", (2.0, 2.3, 2.2), (2.6, 0, 1.5), rgb),
         _vis("glass", (0.06, 2.1, 0.65), (3.61, 0, 2.0), "0.08 0.1 0.14"),
         _vis("cargo", (5.2, 2.4, 2.6), (-0.8, 0, 1.8), cargo_rgb),
         _vis("chassis", (7.0, 1.9, 0.3), (0.1, 0, 0.65), "0.15 0.15 0.15")] + _wheels(7.0, 2.2, 0.45)
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose>'
            '<link name="l">%s</link></model></sdf>' % (name, x, y, yaw, "".join(v)))


class MovingCar:
    """A car moving back and forth along a path with smooth turnaround at ends."""
    def __init__(self, name, model_type, fuel_name, x0, y0, x1, y1, speed, custom_sdf_fn=None):
        self.name = name
        self.model_type = model_type
        self.fuel_name = fuel_name
        self.x0, self.y0 = x0, y0
        self.x1, self.y1 = x1, y1
        self.speed = max(0.1, speed)
        self.custom_sdf_fn = custom_sdf_fn
        self.dist = math.hypot(x1 - x0, y1 - y0)
        self.heading = math.atan2(y1 - y0, x1 - x0)
        self.leg_time = self.dist / self.speed
        self.turn_time = 1.0
        self.cycle_time = 2.0 * (self.leg_time + self.turn_time)

    def pose(self, t):
        t_mod = t % self.cycle_time
        # Phase 1: outbound leg
        if t_mod < self.leg_time:
            frac = _smootherstep(t_mod / self.leg_time)
            x = self.x0 + frac * (self.x1 - self.x0)
            y = self.y0 + frac * (self.y1 - self.y0)
            return x, y, 0.0, self.heading
        # Phase 2: turnaround at (x1, y1)
        elif t_mod < self.leg_time + self.turn_time:
            turn_frac = _smootherstep((t_mod - self.leg_time) / self.turn_time)
            return self.x1, self.y1, 0.0, self.heading + turn_frac * math.pi
        # Phase 3: return leg
        elif t_mod < 2.0 * self.leg_time + self.turn_time:
            ret_t = t_mod - (self.leg_time + self.turn_time)
            frac = _smootherstep(ret_t / self.leg_time)
            x = self.x1 + frac * (self.x0 - self.x1)
            y = self.y1 + frac * (self.y0 - self.y1)
            return x, y, 0.0, self.heading + math.pi
        # Phase 4: turnaround at (x0, y0)
        else:
            turn_frac = _smootherstep((t_mod - (2.0 * self.leg_time + self.turn_time)) / self.turn_time)
            return self.x0, self.y0, 0.0, self.heading + math.pi + turn_frac * math.pi


def acquire_lock():
    try:
        fd = os.open(LOCK_PATH, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.write(fd, str(os.getpid()).encode())
        os.close(fd)
    except FileExistsError:
        try:
            with open(LOCK_PATH) as f:
                other_pid = int(f.read().strip())
            os.kill(other_pid, 0)
            print("[sim_cars] another sim_cars instance (pid %d) is active; stopping it to take over" % other_pid)
            os.kill(other_pid, signal.SIGTERM)
            time.sleep(1.0)
        except Exception:
            pass
        try:
            os.remove(LOCK_PATH)
        except Exception:
            pass
        fd = os.open(LOCK_PATH, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.write(fd, str(os.getpid()).encode())
        os.close(fd)


def release_lock():
    try:
        os.remove(LOCK_PATH)
    except Exception:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-only", action="store_true", help="spawn static cars only and exit")
    parser.add_argument("--speed-factor", type=float, default=1.0, help="speed multiplier for slow moving cars")
    parser.add_argument("--rate", type=float, default=10.0, help="pose update rate in Hz (default 10 Hz)")
    args = parser.parse_args()

    acquire_lock()
    gz = Gz()

    # Define Static Cars of different sizes
    static_cars = [
        # 1. Mini Car (Small, 3.0 m x 1.5 m)
        {"name": "static_minicar_yellow", "type": "custom", "sdf": make_minicar_sdf("static_minicar_yellow", "0.95 0.85 0.05", -4.0, 13.0, 0.15), "x": -4.0, "y": 13.0, "yaw": 0.15, "size": "3.0m Mini"},
        # 2. Compact Hatchback (Small/Compact, 3.9 m x 1.6 m)
        {"name": "static_hatchback_red", "type": "fuel", "model": "Hatchback red", "x": 5.0, "y": 15.0, "yaw": 0.25, "size": "3.9m Compact Hatchback"},
        # 3. Mid-size SUV (Medium, 4.8 m x 1.9 m)
        {"name": "static_suv_silver", "type": "fuel", "model": "SUV", "x": -7.0, "y": 20.0, "yaw": -0.35, "size": "4.8m Mid-size SUV"},
        # 4. Large Pickup Truck (Large, 5.8 m x 2.1 m)
        {"name": "static_pickup_white", "type": "fuel", "model": "Pickup", "x": 7.5, "y": 25.0, "yaw": 1.57, "size": "5.8m Large Pickup"},
        # 5. Delivery Van (Extra Large, 5.2 m x 2.0 m)
        {"name": "static_van_blue", "type": "custom", "sdf": make_van_sdf("static_van_blue", "0.1 0.35 0.8", -10.0, 30.0, 0.8), "x": -10.0, "y": 30.0, "yaw": 0.8, "size": "5.2m Delivery Van"}
    ]

    # Define Slow Moving Cars of different sizes
    sf = args.speed_factor
    moving_cars = [
        # 1. Compact Hatchback (3.9 m, slow crawling at 1.4 m/s across runway)
        MovingCar("slow_hatchback_blue", "fuel", "Hatchback blue", -14.0, 17.0, 14.0, 17.0, 1.4 * sf),
        # 2. Mid-size SUV (4.8 m, slow cruising at 1.8 m/s across deeper lane)
        MovingCar("slow_suv_black", "fuel", "SUV", -18.0, 23.0, 18.0, 23.0, 1.8 * sf),
        # 3. Large Pickup (5.8 m, slow driving at 2.2 m/s along right lane)
        MovingCar("slow_pickup_red", "fuel", "Pickup", 10.5, 12.0, 10.5, 36.0, 2.2 * sf),
        # 4. Large Box Truck (7.2 m, crawling at 1.2 m/s along left lane)
        MovingCar("slow_truck_box", "custom", "truck", -13.0, 14.0, -13.0, 34.0, 1.2 * sf,
                  custom_sdf_fn=lambda name, x, y, yaw: make_truck_sdf(name, "0.85 0.3 0.1", "0.9 0.9 0.92", x, y, yaw))
    ]

    have = set(gz.models())
    print("[sim_cars] Connected to Gazebo iris_runway world.")

    # Spawn static cars
    for c in static_cars:
        if c["name"] not in have:
            if c["type"] == "fuel":
                ok = gz.spawn_fuel(c["name"], c["model"], c["x"], c["y"], 0.0, c["yaw"])
            else:
                ok = gz.spawn_sdf(c["sdf"])
            print("[sim_cars] Spawning static car: %-22s (%s) -> %s" % (c["name"], c["size"], "OK" if ok else "FAIL"))
        else:
            gz.set_pose(c["name"], c["x"], c["y"], 0.0, c["yaw"])
            print("[sim_cars] Static car already present: %-22s (pose refreshed)" % c["name"])

    # Spawn slow moving cars
    for m in moving_cars:
        x, y, z, yaw = m.pose(0.0)
        if m.name not in have:
            if m.model_type == "fuel":
                ok = gz.spawn_fuel(m.name, m.fuel_name, x, y, z, yaw)
            else:
                sdf = m.custom_sdf_fn(m.name, x, y, yaw)
                ok = gz.spawn_sdf(sdf)
            print("[sim_cars] Spawning slow moving car: %-22s (speed=%.1f m/s) -> %s" % (m.name, m.speed, "OK" if ok else "FAIL"))
        else:
            gz.set_pose(m.name, x, y, z, yaw)
            print("[sim_cars] Moving car already present: %-22s (pose refreshed)" % m.name)

    if args.static_only:
        print("[sim_cars] Static cars placed. Exiting (--static-only).")
        release_lock()
        return

    running = True

    def handle_signal(sig, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    dt = 1.0 / max(1.0, args.rate)
    t0 = time.time()
    print("[sim_cars] Running %d static cars + %d slow-moving cars. Press Ctrl+C or kill to stop." % (len(static_cars), len(moving_cars)))

    try:
        while running:
            t = time.time() - t0
            for m in moving_cars:
                x, y, z, yaw = m.pose(t)
                gz.set_pose(m.name, x, y, z, yaw)
            elapsed = (time.time() - t0) - t
            time.sleep(max(0.0, dt - elapsed))
    except Exception as e:
        print("[sim_cars] Loop exception: %s" % e)
    finally:
        release_lock()
        print("[sim_cars] Stopped cleanly.")


if __name__ == "__main__":
    main()
