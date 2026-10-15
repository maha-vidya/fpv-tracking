#!/usr/bin/env python3
"""Scripted arm + take off + face a compass heading, for the Gazebo/SITL tests.

Why: SITL auto-disarms ~10 s after arming if the drone has not taken off, so typing
`arm throttle` and `takeoff` by hand is error-prone. This does GUIDED -> arm -> takeoff -> wait for
altitude -> turn to the heading, then exits and leaves the drone hovering in GUIDED.

It connects to SITL's second TCP port (5762) so it does not fight MAVProxy (which uses 5760).
It sends NOTHING after it finishes (no kill, no mode change, no landing).

  python3 takeoff.py                       # 3 m, face north (heading 0)
  python3 takeoff.py --alt 5 --heading 90  # 5 m, face east
  python3 takeoff.py --connect udpin:0.0.0.0:14552

STATUS: written but NOT yet run against a live SITL (the sim was off when this was written).
Needs pymavlink (already installed on this laptop).
"""
import argparse
import sys
import time

from pymavlink import mavutil

GUIDED = 4


def wait(m, types, timeout, pred=None):
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = m.recv_match(type=types, blocking=True, timeout=1)
        if msg is not None and (pred is None or pred(msg)):
            return msg
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--alt", type=float, default=3.0, help="take-off altitude, metres")
    ap.add_argument("--heading", type=float, default=0.0, help="compass heading to face afterwards (0 = north)")
    ap.add_argument("--arm-timeout", type=float, default=90.0, help="seconds to wait for pre-arm checks to pass")
    a = ap.parse_args()

    m = mavutil.mavlink_connection(a.connect, source_system=255)
    print("waiting for heartbeat on %s ..." % a.connect)
    hb = m.wait_heartbeat(timeout=15)
    if hb is None:
        sys.exit("no heartbeat: is SITL running? (is MAVProxy using port 5760 and this 5762?)")
    print("vehicle %d:%d" % (m.target_system, m.target_component))
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)

    m.set_mode(GUIDED)
    if wait(m, "HEARTBEAT", 5, lambda h: h.custom_mode == GUIDED) is None:
        sys.exit("could not enter GUIDED")
    print("mode GUIDED")

    # Arm: retry until pre-arm checks pass (GPS fix, EKF) -- the first tries often fail right after boot.
    t0 = time.time()
    armed = False
    while time.time() - t0 < a.arm_timeout:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = wait(m, "COMMAND_ACK", 2, lambda k: k.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if ack is not None and ack.result == 0:
            armed = True
            break
        time.sleep(2)
    if not armed:
        sys.exit("could not arm within %.0f s (pre-arm checks failing; look at the MAVProxy console)" % a.arm_timeout)
    print("armed; taking off to %.1f m" % a.alt)

    # Take off immediately: SITL disarms again if nothing happens within ~10 s.
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, a.alt)
    ack = wait(m, "COMMAND_ACK", 3, lambda k: k.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
    if ack is None or ack.result != 0:
        sys.exit("take-off command refused: %s" % (ack,))

    t0 = time.time()
    while time.time() - t0 < 60:
        p = wait(m, "GLOBAL_POSITION_INT", 2)
        if p is not None:
            h = p.relative_alt / 1000.0
            print("\r  altitude %.1f m" % h, end="", flush=True)
            if h >= 0.92 * a.alt:
                break
    else:
        sys.exit("\ndid not reach altitude in 60 s")
    print("\nat altitude; turning to heading %.0f" % a.heading)

    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                            0, a.heading, 30, 0, 0, 0, 0, 0)   # absolute heading, 30 deg/s
    t0 = time.time()
    while time.time() - t0 < 20:
        at = wait(m, "ATTITUDE", 2)
        if at is not None:
            yaw = (at.yaw * 57.29578) % 360.0
            err = (yaw - a.heading + 180) % 360 - 180
            if abs(err) < 4:
                print("heading %.0f reached (error %.1f deg)" % (a.heading, err))
                break
    else:
        print("[warn] heading not reached within 20 s")
    print("done: hovering in GUIDED. (This script sends nothing more.)")


if __name__ == "__main__":
    main()
