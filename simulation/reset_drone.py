#!/usr/bin/env python3
"""SIMULATION ONLY: put a crashed / tipped-over drone back in the air -- stand it upright where it is (Gazebo set_pose),
disarm, take off with takeoff.py, climb to --alt in GUIDED with the gimbal 30 deg down. For repeated fast-GO tests.

  python3 reset_drone.py            # 12 m
"""
import argparse, subprocess, sys, time, os
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
ap = argparse.ArgumentParser(); ap.add_argument("--alt", type=float, default=12.0); a = ap.parse_args()
n = Node(); cur = {}
n.subscribe(Pose_V, "/world/iris_runway/pose/info", lambda m: cur.update({q.name: q for q in m.pose if q.name == "iris_with_gimbal"}))
time.sleep(1.5); q = cur["iris_with_gimbal"]
m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=249); m.wait_heartbeat(timeout=15)
m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0); time.sleep(2)
req = Pose(); req.name = "iris_with_gimbal"; req.position.x, req.position.y, req.position.z = q.position.x, q.position.y, 0.2; req.orientation.w = 1.0
ok, rep = n.request("/world/iris_runway/set_pose", req, Pose, Boolean, 3000); print("upright:", ok and rep.data); time.sleep(8)
here = os.path.dirname(os.path.abspath(__file__))
for i in range(3):
    if subprocess.run([sys.executable, os.path.join(here, "takeoff.py")]).returncode == 0: break
    time.sleep(8)
for _ in range(26):
    m.mav.set_position_target_local_ned_send(0, 1, 1, 1, 0b100111111000, 0, 0, -a.alt, 0, 0, 0, 0, 0, 0, 0.0, 0)
    m.mav.command_int_send(1, 1, 1, mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0, 0, -30.0, 0.0, float("nan"), float("nan"), 0, 0, 0)
    time.sleep(0.5)
t = time.time()
while time.time() - t < 1: m.recv_match(blocking=False)
p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3); print("altitude %.1f m" % (p.relative_alt / 1000))
