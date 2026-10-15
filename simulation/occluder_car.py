#!/usr/bin/env python3
"""A car that drives back and forth IN FRONT of the target, so it hides it from the drone's camera every pass -- the
occlusion test for the tracker (does it stay on the target, or switch to the car?).

The car is driven by Gazebo's own VelocityControl system (smooth, at simulation rate). Driving it with set_pose requests from
Python froze and then teleported it under CPU load; steering a fast physics car with angular velocity rolled it over -- so it
only drives straight, reversing at the ends.

  python3 occluder_car.py                     # y = 12 m (between the drone at 0 and the target at 15), +-12 m, 4 m/s
  python3 occluder_car.py --y 11 --half 15 --speed 6
Ctrl+C stops it (the car stays where it is).
"""
import argparse
import math
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

WORLD = "iris_runway"
NAME = "occluder_car"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--y", type=float, default=12.0, help="the lane (north of the drone), m")
    ap.add_argument("--half", type=float, default=12.0, help="drives between x = -half and +half, m")
    ap.add_argument("--speed", type=float, default=4.0, help="m/s")
    ap.add_argument("--model", default="Hatchback blue", help="OpenRobotics Fuel car model")
    a = ap.parse_args()
    n = Node()
    seen = {}
    def cb(msg):
        for q in msg.pose:
            if q.name == NAME:
                seen["x"] = q.position.x
    n.subscribe(Pose_V, "/world/%s/pose/info" % WORLD, cb)
    time.sleep(1.0)
    if "x" in seen:
        # already in the world (a previous run): put it on THIS run's lane -- otherwise it keeps its old lane and only drives in x
        from gz.msgs10.pose_pb2 import Pose
        req = Pose(); req.name = NAME; req.position.x, req.position.y, req.position.z = -a.half, a.y, 0.05; req.orientation.w = 1.0
        ok, rep = n.request("/world/%s/set_pose" % WORLD, req, Pose, Boolean, 3000)
        print("moved %s to its lane y=%.1f: %s" % (NAME, a.y, ok and rep.data))
    else:
        req = EntityFactory()
        req.sdf = ('<sdf version="1.9"><include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/%s</uri><name>%s</name>'
                   '<pose>%f %f 0.05 0 0 0</pose><static>false</static>'
                   '<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/></include></sdf>'
                   % (a.model, NAME, -a.half, a.y))
        ok, rep = n.request("/world/%s/create" % WORLD, req, EntityFactory, Boolean, 60000)
        print("spawned %s: %s" % (NAME, ok and rep.data))
    pub = n.advertise("/model/%s/cmd_vel" % NAME, Twist)
    sign = 1.0
    print("driving %s at %.1f m/s along y=%.1f between x=+-%.1f (Ctrl+C to stop)" % (NAME, a.speed, a.y, a.half))
    try:
        while True:
            x = seen.get("x")
            if x is not None:
                if x > a.half: sign = -1.0
                if x < -a.half: sign = 1.0
            tw = Twist(); tw.linear.x = a.speed * sign
            pub.publish(tw)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pub.publish(Twist())
        print("stopped")


if __name__ == "__main__":
    main()
