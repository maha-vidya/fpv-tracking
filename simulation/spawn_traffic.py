#!/usr/bin/env python3
"""Spawn real OpenRobotics car/pedestrian mesh models (fetched from Gazebo Fuel, not flat colored boxes)
in the running Gazebo world, on a simple cross-shaped road layout, for testing lock/follow/rescue accuracy
against several simultaneous targets moving in realistic, varied ways (not the single slow target
move_target.py drives).

People are real animated pedestrians: Gazebo skinned <actor>s (walk or run skeletal animation, legs in step with
their speed) following a looping scripted <trajectory> along their path -- limbs and silhouette change every frame,
a harder and more realistic test for CSRT and the colour check than a rigid mesh sliding along. Cars stay kinematic
mesh models moved here every tick (car meshes have no skeleton to animate). --people-mesh restores the old rigid people.

Each actor gets its own path: a straight line (horizontal, vertical, or diagonal heading, chosen at
random), optionally with a sinusoidal side-to-side wobble layered on top so the path curves rather than
staying razor-straight -- a mix of "drives straight down the road", "weaves a bit", and "wanders at an
angle", all running back and forth at the actor's own constant speed, FACING its direction of travel (not
just translating with a fixed orientation). Cars are biased towards the road axes (like traffic actually
would be); people go in any direction (like pedestrians crossing or walking along a sidewalk). Two asphalt
strips with dashed lane markings are spawned as a static visual cross-junction so the scene reads as "a
road", not just models scattered on a field.

  python3 spawn_traffic.py                                    # 4 cars + 6 people (10 total), default speeds
  python3 spawn_traffic.py --cars 5 --people 5 --seed 1        # repeatable layout
  python3 spawn_traffic.py --car-speed 3 14 --person-speed 0.8 2.2
  python3 spawn_traffic.py --area 40                          # scatter within +/-40 m of the centre
  python3 spawn_traffic.py --curvy-frac 0.7                   # more actors get the S-curve wobble
  python3 spawn_traffic.py --boxes                            # fall back to plain colored boxes (no network/Fuel needed)
  python3 spawn_traffic.py --cars 0 --people 0 --vehicles 14  # many-colour cars/vans/buses/trucks/motorbikes roaming
                                                              # +/-70 m at random, slow / medium / fast in turn

The real models are fetched from https://fuel.gazebosim.org on first use and cached under ~/.gz/fuel/ --
needs network access once; after that, spawns are instant from the local cache. Pre-fetch everything in
one go (useful before a demo, so there's no first-spawn download pause):
  gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback red"
  (repeat for: "Hatchback blue", SUV, Pickup, "Walking person", "Male visitor", Scrubs)

Ctrl+C stops the motion and leaves every actor where it is. Only touches the actors (and road strips) it
spawns -- never the drone, and never target_red/lookalike_orange (use move_target.py for those).

STATUS: live-tested against a running Gazebo (spawn, genuine motion, and the real-model fetch/cache path
all confirmed working). gz-transport Python bindings are tried first, falling back to the slower `gz` CLI.
"""
import argparse
import math
import os
import random
import re
import subprocess
import sys
import time

FUEL_BASE = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/%s"
CAR_MODELS = ["Hatchback red", "Hatchback blue", "SUV", "Pickup"]
PERSON_MODELS = ["Walking person", "Male visitor", "Scrubs"]          # --people-mesh only (rigid, slides)
# Animated pedestrians: Gazebo's skinned actor (Fuel "Mingfei/actor"), with real walk/run skeletal animations. Gazebo plays the
# animation and moves the actor along its scripted <trajectory> itself (sim time) -- no per-tick set_pose for these.
ACTOR_MESH = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/%s.dae"
RUN_ABOVE_MS = 2.2                                                      # faster than this: run animation instead of walk

WORLD = "iris_runway"
LOCK_PATH = "/tmp/spawn_traffic.%s.lock" % WORLD


def acquire_lock():
    """Refuse to start a second pose-driving loop against the same world: two instances independently
    set_pose()-ing the same actor names (different clocks) teleport each actor between two different
    positions every tick, which is exactly what shows up as "flickering" -- this is what was actually
    happening (confirmed live: two overlapping spawn_traffic.py processes running at once), not just the
    static/physics issue fixed alongside this. A stale lock from a killed-without-cleanup process is
    detected and replaced (its PID no longer exists)."""
    while True:
        try:
            fd = os.open(LOCK_PATH, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.write(fd, str(os.getpid()).encode())
            os.close(fd)
            return
        except FileExistsError:
            try:
                with open(LOCK_PATH) as f:
                    other_pid = int(f.read().strip())
                os.kill(other_pid, 0)                 # raises if that PID is gone
                sys.exit("another spawn_traffic.py (pid %d) is already driving world %s -- stop it first "
                          "(or it will fight this one over the same actor poses and look like flickering)"
                          % (other_pid, WORLD))
            except (ValueError, OSError):
                try:
                    os.remove(LOCK_PATH)              # stale lock (process gone, or unreadable) -- retry
                except OSError:
                    pass


def release_lock():
    try:
        os.remove(LOCK_PATH)
    except OSError:
        pass


def _sdf_box(name, rgba, sx, sy, sz, x, y, z, yaw=0.0, static=True):
    """Fallback visual (--boxes / no network): a single flat-colored box, same as before this model switch."""
    return ('<sdf version="1.9"><model name="%s"><static>%s</static><pose>%f %f %f 0 0 %f</pose>'
            '<link name="l"><visual name="v"><geometry><box><size>%f %f %f</size></box></geometry>'
            '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual></link></model></sdf>'
            % (name, "true" if static else "false", x, y, z, yaw, sx, sy, sz, rgba, rgba))


def _sdf_include(name, fuel_model, x, y, z, yaw=0.0, static=True):
    """A real mesh model fetched from Gazebo Fuel (cached locally after the first fetch), renamed to this
    actor's own entity name and placed/oriented like any other spawn. <static> is forced here regardless of
    the included model's own default: a person model carries real mass/inertia (meant for physics use
    elsewhere), which would fight our set_pose() teleportation the same way un-static boxes did earlier."""
    uri = FUEL_BASE % fuel_model
    return ('<sdf version="1.9"><include><uri>%s</uri><name>%s</name><pose>%f %f %f 0 0 %f</pose>'
            '<static>%s</static></include></sdf>'
            % (uri, name, x, y, z, yaw, "true" if static else "false"))


SHIRTS = {"green": None, "red": (0.70, 0.05, 0.05), "blue": (0.05, 0.15, 0.70), "yellow": (0.85, 0.75, 0.05), "white": (0.85, 0.85, 0.85),
          "orange": (0.90, 0.40, 0.03), "purple": (0.40, 0.08, 0.55), "black": (0.03, 0.03, 0.03)}
WALK_CACHE = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/1/meshes/walk.dae")

def shirt_skin(colour):
    """the walker's mesh with its sweater recoloured (the actor's colours are plain material colours in the .dae, no textures):
    distinct-looking people for ReID tests. Written once into $RUN_DIR/models/walker_shirts/. None = the original (green)."""
    rgb = SHIRTS[colour]
    if rgb is None or not os.path.exists(WALK_CACHE):
        return None
    out_dir = os.path.join(os.environ.get("RUN_DIR", os.path.expanduser("~/sim_run")), "models", "walker_shirts"); os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "walk_%s.dae" % colour)
    if not os.path.exists(out):
        s = open(WALK_CACHE).read(); i = s.find('id="sweater-green-effect"'); j = s.find("</effect>", i)
        c = "%f %f %f 1" % rgb
        eff = re.sub(r'(<color sid="(?:ambient|diffuse)">)[^<]*(</color>)', lambda m: m.group(1) + c + m.group(2), s[i:j])
        open(out, "w").write(s[:i] + eff + s[j:])
    return out

def _sdf_walker(name, waypoints, anim, shirt=None):
    """A skinned, animated pedestrian (<actor>) following a looping scripted trajectory. waypoints: [(t, x, y, yaw)].
    interpolate_x keeps the leg animation in step with the actual distance covered, so a slow walker and a fast
    walker both look right instead of moonwalking. The actor faces +x at yaw 0, so yaw = direction of travel."""
    skin = (shirt_skin(shirt) if shirt else None) or ACTOR_MESH % "walk"
    wps = "".join("<waypoint><time>%.3f</time><pose>%.3f %.3f 0 0 0 %.4f</pose></waypoint>" % w for w in waypoints)
    return ('<sdf version="1.9"><actor name="%s"><skin><filename>%s</filename><scale>1.0</scale></skin>'
            '<animation name="%s"><filename>%s</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>'
            '<script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>'
            '<trajectory id="0" type="%s">%s</trajectory></script></actor></sdf>'
            % (name, skin, anim, ACTOR_MESH % anim, anim, wps))


def walker_waypoints(act, step_m=0.5, turn_s=0.8):
    """Turn an Actor's back-and-forth (optionally curvy) path into actor waypoints: constant walking speed along the
    path (people do not ease in/out like the cars), facing the direction of travel, turning on the spot at each end."""
    pts = []                                                          # outbound leg: d from -half_len to +half_len
    n = max(2, int(2 * act.half_len / step_m))
    for i in range(n + 1):
        d = -act.half_len + 2 * act.half_len * i / n
        x, y = act.cx + act.dx * d, act.cy + act.dy * d
        if act.curve_amp > 0:
            lat = act.curve_amp * math.sin(2 * math.pi * act.curve_cycles * (d / (2 * act.half_len) + 0.5))
            x += act.px * lat; y += act.py * lat
        pts.append((x, y))
    wps, t = [], 0.0
    for leg in (pts, pts[::-1]):
        for i, (x, y) in enumerate(leg):
            nx, ny = leg[min(i + 1, len(leg) - 1)]
            px, py = leg[max(i - 1, 0)]
            yaw = math.atan2(ny - py, nx - px)                        # tangent (central difference)
            if i > 0:
                t += math.hypot(x - leg[i - 1][0], y - leg[i - 1][1]) / act.speed
            wps.append((t, x, y, yaw))
        t += turn_s                                                   # turn round on the spot at the end of the leg
        wps.append((t, leg[-1][0], leg[-1][1], wps[-1][3] + math.pi))
    return wps


class Gz:
    """Same spawn/set_pose helper as move_target.py (gz-transport Python first, `gz` CLI fallback)."""
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

    def spawn_box(self, name, rgba, sx, sy, sz, x, y, z, yaw=0.0, static=True):
        return self._create(_sdf_box(name, rgba, sx, sy, sz, x, y, z, yaw, static))

    def spawn_include(self, name, fuel_model, x, y, z, yaw=0.0, static=True):
        # A first-time fetch of a ~25 MB mesh (the person models) over the network is far slower than a
        # local spawn -- found live (the Walking person model alone took several seconds to download and
        # cache under ~/.gz/fuel/ the first time). 60s covers a cold cache on a slow link; every spawn
        # after the first for the same model is served from that cache and returns almost immediately.
        return self._create(_sdf_include(name, fuel_model, x, y, z, yaw, static), timeout_ms=60000)

    def _create(self, sdf, timeout_ms=None):
        timeout_ms = timeout_ms or self.timeout
        if self.node:
            req = self.EF(); req.sdf = sdf
            ok, rep = self.node.request("/world/%s/create" % WORLD, req, self.EF, self.Boolean, timeout_ms)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/create" % WORLD, "--reqtype", "gz.msgs.EntityFactory",
                            "--reptype", "gz.msgs.Boolean", "--timeout", str(timeout_ms), "--req", "sdf: '%s'" % sdf],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True,
                           timeout=timeout_ms / 1000.0 + 5)
        return "true" in r.stdout

    def spawn_walker(self, name, waypoints, anim, shirt=None):
        # first use downloads the actor meshes from Fuel (cached under ~/.gz/fuel afterwards)
        return self._create(_sdf_walker(name, waypoints, anim, shirt), timeout_ms=60000)

    def set_pose(self, name, x, y, z, yaw=0.0):
        qw, qz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)   # rotation about Z only: these are ground actors, never tilted
        if self.node:
            req = self.Pose(); req.name = name
            req.position.x, req.position.y, req.position.z = x, y, z
            req.orientation.w, req.orientation.z = qw, qz
            ok, rep = self.node.request("/world/%s/set_pose" % WORLD, req, self.Pose, self.Boolean, self.timeout)
            return ok and rep.data
        r = subprocess.run(["gz", "service", "-s", "/world/%s/set_pose" % WORLD, "--reqtype", "gz.msgs.Pose",
                            "--reptype", "gz.msgs.Boolean", "--timeout", "1000", "--req",
                            'name: "%s" position {x: %f y: %f z: %f} orientation {w: %f z: %f}' % (name, x, y, z, qw, qz)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        return "true" in r.stdout


class Actor:
    """A multi-box model (see car_parts/person_parts) moving back and forth along a line at constant
    speed, with an optional sinusoidal side-to-side wobble layered on top so the path curves instead of
    staying razor-straight. Faces its direction of travel (forward on the outbound leg, turned around on
    the return leg) -- with a real mesh model instead of a symmetric box, driving backwards-facing would be
    immediately, obviously wrong rather than just a crude approximation."""
    def __init__(self, name, model, cx, cy, z, heading_rad, half_len, speed, curve_amp=0.0, curve_cycles=2.0):
        self.name, self.model = name, model
        self.cx, self.cy, self.z = cx, cy, z
        self.heading = heading_rad
        self.dx, self.dy = math.cos(heading_rad), math.sin(heading_rad)     # along the line
        self.px, self.py = -self.dy, self.dx                                # perpendicular (for the wobble)
        self.half_len, self.speed = half_len, speed
        self.curve_amp, self.curve_cycles = curve_amp, curve_cycles
        self.period = 4.0 * half_len / speed if speed > 0 else 1e9   # full back-and-forth cycle

    @staticmethod
    def _smootherstep(x):
        # C2-continuous ease: zero velocity AND zero acceleration at x=0 and x=1, matching how a real car
        # or person actually starts/stops (eases in, cruises, eases out) instead of an instant reversal.
        return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)

    def pose(self, t):
        # Continuous back-and-forth (-half_len -> +half_len -> -half_len), eased at both ends of each leg
        # instead of a raw triangle wave. The previous version held the actor completely motionless for
        # HALF of every cycle (phase in [1,2) and [3,4)) while still using period = 4*half_len/speed --
        # meaning it covered the full back-and-forth distance in only half the period, so the real speed
        # while moving was 2x whatever --car-speed/--person-speed actually said, with a dead freeze in
        # between. Fixed: no hold phases, real average speed over a leg now matches the configured speed.
        frac = (t % self.period) / self.period                # 0..1 across one full cycle
        if frac < 0.5:
            s = -1.0 + 2.0 * self._smootherstep(frac / 0.5)    # outbound leg: -1 -> 1
        else:
            s = 1.0 - 2.0 * self._smootherstep((frac - 0.5) / 0.5)   # return leg: 1 -> -1
        d = s * self.half_len
        x = self.cx + self.dx * d
        y = self.cy + self.dy * d
        if self.curve_amp > 0:
            # wobble keyed to distance travelled along the line (not time), so it looks the same shape on
            # the way out as on the way back, like a real winding path rather than a flapping offset
            lateral = self.curve_amp * math.sin(2 * math.pi * self.curve_cycles * (d / (2 * self.half_len) + 0.5))
            x += self.px * lateral
            y += self.py * lateral
        # frac < 0.5: outbound leg, facing forward. frac >= 0.5: return leg, turned around. Ignores the
        # wobble's own small tangential contribution to heading: a reasonable approximation given curve_amp
        # is modest relative to half_len, and ground truth for "which way is it basically driving" is what
        # matters for a legible scene, not a geometrically exact tangent every instant.
        yaw = self.heading if frac < 0.5 else self.heading + math.pi
        return x, y, self.z, yaw


# ---- --vehicles: primitive-built cars, buses, trucks, vans and motorbikes in any colour (no download), roaming the area at random --
VEHICLE_COLOURS = {"red": "0.75 0.06 0.06", "blue": "0.06 0.18 0.75", "white": "0.88 0.88 0.88", "black": "0.06 0.06 0.06",
                   "silver": "0.6 0.62 0.65", "yellow": "0.9 0.75 0.05", "green": "0.1 0.5 0.15", "orange": "0.92 0.42 0.05",
                   "purple": "0.42 0.1 0.55", "cyan": "0.05 0.6 0.7", "brown": "0.4 0.24 0.1", "pink": "0.9 0.45 0.6"}
# type: (weight in the random mix, typical speed scale)
VEHICLE_TYPES = {"car": (5, 1.0), "van": (2, 0.9), "bus": (2, 0.7), "truck": (2, 0.75), "motorbike": (2, 1.15)}
SPEED_TIERS = {"slow": (1.5, 4.0), "medium": (5.0, 9.0), "fast": (11.0, 18.0)}


def _vis(name, size, xyz, rgb, rpy=(0, 0, 0), shape="box"):
    geo = ("<box><size>%f %f %f</size></box>" % size) if shape == "box" else \
          ("<cylinder><radius>%f</radius><length>%f</length></cylinder>" % size)
    return ('<visual name="%s"><pose>%f %f %f %f %f %f</pose><geometry>%s</geometry>'
            '<material><ambient>%s 1</ambient><diffuse>%s 1</diffuse><specular>0.3 0.3 0.3 1</specular></material></visual>'
            % ((name,) + tuple(xyz) + tuple(rpy) + (geo, rgb, rgb)))


def _wheels(length, width, r, n_axles=2):
    out = []
    xs = [length * 0.33, -length * 0.33] if n_axles == 2 else [length * 0.36, -length * 0.15, -length * 0.36]
    for i, x in enumerate(xs):
        for j, y in enumerate((width / 2, -width / 2)):
            out.append(_vis("w%d%d" % (i, j), (r, 0.28), (x, y, r), "0.05 0.05 0.05", (math.pi / 2, 0, 0), "cyl"))
    return out


def vehicle_sdf(name, vtype, rgb, x, y, yaw, rng):
    """One static model, several visuals: body, glass, wheels (and a cargo box in another colour for trucks). x forward."""
    glass, v = "0.08 0.1 0.14", []
    if vtype == "car":
        v += [_vis("body", (4.3, 1.8, 0.75), (0, 0, 0.7), rgb), _vis("cabin", (2.2, 1.6, 0.6), (-0.2, 0, 1.35), rgb),
              _vis("glass", (2.25, 1.62, 0.36), (-0.2, 0, 1.33), glass)] + _wheels(4.3, 1.8, 0.34)
    elif vtype == "van":
        v += [_vis("body", (5.0, 2.0, 1.9), (0, 0, 1.3), rgb), _vis("glass", (0.06, 1.8, 0.6), (2.5, 0, 1.75), glass),
              _vis("side", (2.6, 2.02, 0.45), (0.9, 0, 1.75), glass)] + _wheels(5.0, 2.0, 0.36)
    elif vtype == "bus":
        v += [_vis("body", (11.0, 2.5, 2.7), (0, 0, 1.85), rgb), _vis("windows", (10.0, 2.52, 0.8), (0.2, 0, 2.45), glass),
              _vis("roof", (10.6, 2.3, 0.12), (0, 0, 3.24), "0.85 0.85 0.85")] + _wheels(11.0, 2.5, 0.5)
    elif vtype == "truck":
        cargo = rng.choice([c for c in VEHICLE_COLOURS.values() if c != rgb])
        v += [_vis("cab", (2.2, 2.4, 2.3), (3.3, 0, 1.6), rgb), _vis("glass", (0.06, 2.2, 0.7), (4.41, 0, 2.15), glass),
              _vis("cargo", (6.4, 2.5, 2.9), (-1.2, 0, 2.0), cargo), _vis("chassis", (8.6, 2.0, 0.3), (0.1, 0, 0.75), "0.15 0.15 0.15")] \
             + _wheels(8.6, 2.3, 0.5, 3)
    else:                                                               # motorbike + rider
        v += [_vis("frame", (1.9, 0.35, 0.5), (0, 0, 0.7), rgb), _vis("wf", (0.33, 0.12), (0.75, 0, 0.33), "0.05 0.05 0.05", (math.pi / 2, 0, 0), "cyl"),
              _vis("wr", (0.33, 0.12), (-0.75, 0, 0.33), "0.05 0.05 0.05", (math.pi / 2, 0, 0), "cyl"),
              _vis("rider", (0.45, 0.5, 0.75), (-0.15, 0, 1.3), "0.12 0.12 0.15"), _vis("helmet", (0.3, 0.3, 0.3), (-0.05, 0, 1.8), rgb)]
    return ('<sdf version="1.9"><model name="%s"><static>true</static><pose>%f %f 0 0 0 %f</pose><link name="l">%s</link></model></sdf>'
            % (name, x, y, yaw, "".join(v)))


class Roamer:
    """A vehicle driving around the area at random: heads for a random point, turning like a vehicle (limited turn rate,
    slower in tight turns), picks a new point on arrival; its cruising speed changes now and then within its tier. Any
    direction, never back-and-forth. Stays inside the area (a new point is always inside it)."""
    def __init__(self, name, model, cx, cy, area, speed_rng, rng, turn_rate):
        self.name, self.model, self.rng = name, model, rng
        self.cx, self.cy, self.area, self.speed_rng, self.turn_rate = cx, cy, area, speed_rng, turn_rate
        self.x, self.y = cx + rng.uniform(-area, area), cy + rng.uniform(-area, area)
        self.yaw = rng.uniform(-math.pi, math.pi)
        self.speed = self.cruise = rng.uniform(*speed_rng)
        self.curve_amp = 0.0
        self.t = 0.0
        self._new_goal()

    def _new_goal(self):
        for _ in range(20):                                             # far enough to be a real drive
            gx, gy = self.cx + self.rng.uniform(-self.area, self.area), self.cy + self.rng.uniform(-self.area, self.area)
            if math.hypot(gx - self.x, gy - self.y) > 0.5 * self.area:
                break
        self.gx, self.gy = gx, gy
        if self.rng.random() < 0.4:
            self.cruise = self.rng.uniform(*self.speed_rng)

    def pose(self, t):
        dt = min(0.5, max(0.0, t - self.t)); self.t = t
        want = math.atan2(self.gy - self.y, self.gx - self.x)
        err = math.remainder(want - self.yaw, 2 * math.pi)
        self.yaw += max(-self.turn_rate * dt, min(self.turn_rate * dt, err))
        target_v = self.cruise * max(0.35, math.cos(min(abs(err), math.pi / 2)))   # slow down for a sharp turn
        self.speed += max(-4.0 * dt, min(3.0 * dt, target_v - self.speed))
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        if math.hypot(self.gx - self.x, self.gy - self.y) < max(3.0, self.speed):
            self._new_goal()
        return self.x, self.y, 0.0, self.yaw


def make_vehicles(a, rng):
    tiers = [t.strip() for t in a.vehicle_tiers.split(",") if t.strip()]
    for t in tiers:
        if t not in SPEED_TIERS: sys.exit("unknown speed tier %r (choose from %s)" % (t, ",".join(SPEED_TIERS)))
    types = [t.strip() for t in a.vehicle_types.split(",") if t.strip()]
    for t in types:
        if t not in VEHICLE_TYPES: sys.exit("unknown vehicle type %r (choose from %s)" % (t, ",".join(VEHICLE_TYPES)))
    weights = [VEHICLE_TYPES[t][0] for t in types]
    colours = list(VEHICLE_COLOURS)
    out = []
    for i in range(a.vehicles):
        vtype = rng.choices(types, weights)[0]
        tier = tiers[i % len(tiers)]                                    # every tier represented, in turn
        lo, hi = SPEED_TIERS[tier]; sc = VEHICLE_TYPES[vtype][1]
        colour = colours[i % len(colours)] if i < len(colours) else rng.choice(colours)
        turn = {"bus": 0.35, "truck": 0.4}.get(vtype, 0.7 if vtype != "motorbike" else 1.0)
        r = Roamer("traffic_veh_%d" % i, ("vehicle", vtype, VEHICLE_COLOURS[colour]), a.cx, a.cy, a.vehicle_area,
                   (lo * sc, hi * sc), rng, turn)
        r.label = "%s %s %s" % (colour, vtype, tier)
        out.append(r)
    return out


HEADINGS = {"horizontal": 0.0, "vertical": math.pi / 2}


def pick_heading(rng, road_biased):
    """road_biased: mostly along a road axis (+/- a few degrees of jitter), occasionally diagonal --
    like a car staying in its lane but not spawned perfectly axis-aligned. Pedestrians (road_biased=False)
    go any direction at all."""
    if not road_biased or rng.random() < 0.2:
        return rng.uniform(0, 2 * math.pi)
    base = rng.choice(list(HEADINGS.values()))
    if rng.random() < 0.5:
        base += math.pi                                       # travel the other way along the same axis
    return base + rng.uniform(-0.12, 0.12)                     # +/- ~7 deg jitter, not laser-straight


CAR_COLORS = ["0.75 0.08 0.08 1", "0.08 0.08 0.75 1", "0.8 0.8 0.82 1", "0.1 0.1 0.1 1", "0.85 0.75 0.1 1"]
PERSON_COLORS = ["0.1 0.3 0.8 1", "0.8 0.25 0.1 1", "0.2 0.55 0.2 1", "0.6 0.1 0.55 1"]


def make_actors(a, rng):
    actors = []
    for i in range(a.cars):
        heading = pick_heading(rng, road_biased=True)
        half_len = rng.uniform(a.area * 0.3, a.area * 0.9)
        speed = rng.uniform(*a.car_speed)
        cx = a.cx + rng.uniform(-a.area, a.area)
        cy = a.cy + rng.uniform(-a.area, a.area)
        curvy = rng.random() < a.curvy_frac
        amp = rng.uniform(1.5, 4.0) if curvy else 0.0
        cycles = rng.uniform(1.0, 2.5)
        model = ("box", rng.choice(CAR_COLORS), (4.2, 1.9, 1.1)) if a.boxes else ("mesh", rng.choice(CAR_MODELS))
        actors.append(Actor("traffic_car_%d" % i, model, cx, cy, 0.0, heading, half_len, speed, amp, cycles))
    for i in range(a.people):
        heading = pick_heading(rng, road_biased=False)
        half_len = rng.uniform(a.area * 0.15, a.area * 0.5)
        speed = rng.uniform(*a.person_speed)
        cx = a.cx + rng.uniform(-a.area, a.area)
        cy = a.cy + rng.uniform(-a.area, a.area)
        curvy = rng.random() < a.curvy_frac
        amp = rng.uniform(0.8, 2.5) if curvy else 0.0
        cycles = rng.uniform(1.0, 3.0)
        if a.boxes:
            model = ("box", rng.choice(PERSON_COLORS), (0.5, 0.5, 1.7))
        elif a.people_mesh:
            model = ("mesh", rng.choice(PERSON_MODELS))
        else:
            model = ("walker", "run" if speed > RUN_ABOVE_MS else "walk")
        actors.append(Actor("traffic_person_%d" % i, model, cx, cy, 0.0, heading, half_len, speed, amp, cycles))
    return actors


def spawn_actor(gz, act, x, y, z, yaw):
    kind = act.model[0]
    if kind == "vehicle":
        return gz._create(vehicle_sdf(act.name, act.model[1], act.model[2], x, y, yaw, random.Random(act.name)))
    if kind == "walker":
        return gz.spawn_walker(act.name, walker_waypoints(act), act.model[1], getattr(act, "shirt", None))
    if kind == "box":
        _, rgba, size = act.model
        return gz.spawn_box(act.name, rgba, *size, x, y, z + size[2] / 2, yaw, static=True)
    _, fuel_model = act.model
    return gz.spawn_include(act.name, fuel_model, x, y, z, yaw, static=True)


def spawn_roads(gz, a, have):
    """Two asphalt strips crossing at (cx, cy) with dashed white lane markings down the middle of each --
    a visual cue that this is a road junction, not just models scattered on a field. Purely cosmetic
    (static), no physics."""
    span = a.area * 2.2
    roads = [
        ("traffic_road_h", (span, 6.0, 0.04), a.cx, a.cy),
        ("traffic_road_v", (6.0, span, 0.04), a.cx, a.cy),
    ]
    for name, size, x, y in roads:
        if name in have:
            continue
        ok = gz.spawn_box(name, "0.12 0.12 0.13 1", *size, x, y, 0.02, static=True)
        print("[spawn] %-18s %s" % (name, "ok" if ok else "FAILED (non-fatal: cosmetic only)"))

    dash_len, gap, width = 1.6, 1.2, 0.18
    step = dash_len + gap
    n = int(span / step)
    for i in range(n):
        off = -span / 2 + i * step + dash_len / 2
        for axis, (dx, dy, dsz) in (("h", (off, 0, (dash_len, width, 0.02))),
                                     ("v", (0, off, (width, dash_len, 0.02)))):
            name = "traffic_lane_%s_%d" % (axis, i)
            if name in have or abs(off) < 3.5:    # skip dashes through the middle of the junction
                continue
            gz.spawn_box(name, "0.95 0.95 0.9 1", *dsz, a.cx + dx, a.cy + dy, 0.045, static=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cars", type=int, default=4)
    ap.add_argument("--people", type=int, default=6)
    ap.add_argument("--car-speed", type=float, nargs=2, default=(3.0, 12.0), metavar=("MIN", "MAX"), help="m/s, fast-to-slow mix")
    ap.add_argument("--person-speed", type=float, nargs=2, default=(0.8, 2.2), metavar=("MIN", "MAX"), help="m/s")
    ap.add_argument("--curvy-frac", type=float, default=0.5, help="fraction of actors that get an S-curve wobble instead of a dead-straight line")
    ap.add_argument("--no-roads", action="store_true", help="skip spawning the cosmetic road-strip visuals")
    ap.add_argument("--shirts", default="", help="comma-separated sweater colours given to the walkers in turn (%s) -- distinct-looking "
                    "people for ReID / identity tests; empty = all the original green" % ",".join(SHIRTS))
    ap.add_argument("--people-mesh", action="store_true", help="old behaviour: rigid person meshes slid along the path (no walking animation)")
    ap.add_argument("--boxes", action="store_true", help="fall back to plain colored boxes instead of real OpenRobotics meshes (no network/Fuel cache needed)")
    ap.add_argument("--cx", type=float, default=0.0, help="scatter area centre x (east), m")
    ap.add_argument("--cy", type=float, default=15.0, help="scatter area centre y (north, ahead of the drone), m")
    ap.add_argument("--area", type=float, default=25.0, help="scatter half-width, m (also scales path length)")
    ap.add_argument("--vehicles", type=int, default=0, help="primitive-built vehicles (cars, vans, buses, trucks, motorbikes) in many "
                    "colours, roaming the area at random in any direction (no download)")
    ap.add_argument("--vehicle-types", default=",".join(VEHICLE_TYPES), help="comma-separated mix (%(default)s)")
    ap.add_argument("--vehicle-tiers", default="slow,medium,fast", help="speed tiers given in turn: slow %.1f-%.1f, medium %.1f-%.1f, fast %.1f-%.1f m/s"
                    % (SPEED_TIERS["slow"] + SPEED_TIERS["medium"] + SPEED_TIERS["fast"]))
    ap.add_argument("--vehicle-area", type=float, default=70.0, help="vehicles roam within +/- this many metres of the centre")
    ap.add_argument("--rate", type=float, default=10.0, help="pose updates per second (higher than move_target.py: fast cars need it)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--seed", type=int, default=None, help="repeatable layout")
    a = ap.parse_args()

    global LOCK_PATH
    if a.cars == 0 and a.people == 0 and a.vehicles > 0:
        # only the traffic_veh_* actors: their own lock, so they can run beside a people/cars instance (different names, no fight)
        LOCK_PATH = "/tmp/spawn_traffic.%s.vehicles.lock" % WORLD
    acquire_lock()
    try:
        run(a)
    finally:
        release_lock()


def run(a):
    rng = random.Random(a.seed)
    gz = Gz()
    have = gz.models()

    if not a.no_roads:
        spawn_roads(gz, a, have)

    actors = make_actors(a, rng) + make_vehicles(a, rng)
    shirts = [c.strip() for c in a.shirts.split(",") if c.strip()]
    for c in shirts:
        if c not in SHIRTS: sys.exit("unknown shirt colour %r (choose from %s)" % (c, ",".join(SHIRTS)))
    if shirts:
        k = 0
        for act in actors:
            if act.model[0] == "walker":
                act.shirt = shirts[k % len(shirts)]; k += 1
    print("fetching/spawning real models from Gazebo Fuel (first use of each model downloads + caches it "
          "under ~/.gz/fuel/ -- can take a few seconds per model the first time)..." if not a.boxes else
          "spawning plain colored boxes (--boxes)...")
    for act in actors:
        if act.name in have:
            continue
        x, y, z, yaw = act.pose(0.0)
        # static=True: these are teleported every tick via set_pose(), not physics-driven. A real mesh model
        # (unlike a plain box) carries actual mass/inertia meant for physics use elsewhere -- spawning it
        # dynamic would let Gazebo's physics apply gravity/collision between our pose updates, fighting the
        # teleport exactly the way un-static boxes once did (that was the original cause of the reported
        # "flickering", fixed earlier this way for boxes and carried over here for the real models too).
        ok = spawn_actor(gz, act, x, y, z, yaw)
        shape = "curve" if act.curve_amp > 0 else "line"
        kind = {"mesh": act.model[1], "walker": "actor/" + act.model[1], "vehicle": getattr(act, "label", "")}.get(act.model[0], "box")
        print("[spawn] %-18s %-5s %-14s speed=%.1fm/s path=%-5s start=(%.1f,%.1f)" %
              (act.name, "ok" if ok else "FAILED", kind, act.speed, shape, x, y))
        if not ok:
            sys.exit("could not spawn %s: is Gazebo running with world %s, and is there network access for "
                      "the Fuel model fetch (try --boxes if not)?" % (act.name, WORLD))

    kinematic = [act for act in actors if act.model[0] != "walker"]
    print("moving %d cars + %d people + %d vehicles = %d total (Ctrl+C to stop; driven actors stay where they are, animated walkers keep "
          "walking -- remove them with: gz service -s /world/%s/remove ... or restart the sim)" % (a.cars, a.people, a.vehicles, len(actors), WORLD))
    t0 = time.time()
    dt = 1.0 / a.rate
    try:
        while True:
            t = time.time() - t0
            if a.duration and t > a.duration:
                break
            for act in kinematic:                                  # animated walkers are moved by Gazebo itself
                x, y, z, yaw = act.pose(t)
                gz.set_pose(act.name, x, y, z, yaw)
            time.sleep(max(0.0, dt - ((time.time() - t0) - t)))
    except KeyboardInterrupt:
        pass
    print("stopped")


if __name__ == "__main__":
    main()
