#!/usr/bin/env python3
"""Simulated pilot with hands on the sticks: keeps RC roll/pitch/throttle/yaw at centre (1500 us) via RC_CHANNELS_OVERRIDE.

Why: when the tracker loses the target it hands control back by switching to LOITER. LOITER flies on the RC sticks; in SITL
nobody holds them, the throttle reads low, and the drone sinks, lands and disarms between tests. With the sticks held at
centre, LOITER holds position and altitude like it would with a real pilot. GUIDED ignores the sticks, so following is
unaffected. Simulation only.

  python3 sticks_centered.py                                  # MAVProxy's second output (sim_up.sh --mav-to 127.0.0.1:14561)
  python3 sticks_centered.py --connect udpin:0.0.0.0:14560     # whatever free MAVProxy output you have (not QGC's 14550)
"""
import argparse
import time

from pymavlink import mavutil

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--connect", default="udpin:0.0.0.0:14561")
a = ap.parse_args()
m = mavutil.mavlink_connection(a.connect, source_system=255)
print("waiting for the vehicle on %s ..." % a.connect)
m.wait_heartbeat(timeout=30)
print("holding sticks at centre (ch1-4 = 1500; throttle low while disarmed, as a pilot arms) -- Ctrl+C to release")
armed = False
try:
    while True:
        hb = m.recv_match(type="HEARTBEAT", blocking=False)
        while hb is not None:
            if hb.get_srcComponent() == 1 and hb.type != mavutil.mavlink.MAV_TYPE_GCS:
                armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            hb = m.recv_match(type="HEARTBEAT", blocking=False)
        thr = 1500 if armed else 1000                                 # centred throttle blocks arming ("throttle too high")
        # 0 = "no override" for every other channel, so switches on ch5+ stay with the (simulated) transmitter
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 1500, 1500, thr, 1500, 0, 0, 0, 0)
        time.sleep(0.2)
except KeyboardInterrupt:
    m.mav.rc_channels_override_send(m.target_system, m.target_component, 0, 0, 0, 0, 0, 0, 0, 0)
    print("released")
