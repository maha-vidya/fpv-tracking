#!/usr/bin/env python3
"""Pilot / environment events for the SITL simulation (pymavlink on SITL's second TCP port 5762).

  python3 sim_events.py mode LOITER            # pilot takes over (a mode change away from GUIDED)
  python3 sim_events.py mode GUIDED
  python3 sim_events.py wind 5 90              # 5 m/s wind from heading 90 deg (SIM_WIND_SPD / SIM_WIND_DIR)
  python3 sim_events.py wind 0                 # calm
  python3 sim_events.py rc 9 1900 --seconds 3  # hold RC channel 9 at 1900 us for 3 s (the GO switch), then release
  python3 sim_events.py rc 1 1700 --seconds 2  # a pilot stick deflection on channel 1
  python3 sim_events.py rcseq 9 1500:1.0 1900:1.5   # a 3-position switch: middle (arm) for 1 s, then high (go) for 1.5 s
  python3 sim_events.py param SIM_WIND_TURB 2

Uses source system 255 because ArduPilot only accepts RC overrides from the GCS system id range
(MAV_GCS_SYSID, default 255; ArduPilot parameter metadata). RC_CHANNELS_OVERRIDE: 0 releases a channel,
65535 (UINT16_MAX) means "ignore this channel" (MAVLink common.xml). The override times out after RC_OVERRIDE_TIME,
so it is re-sent at 5 Hz while held.

STATUS: written but not yet exercised against the live SITL when this header was written.
"""
import argparse
import sys
import time

from pymavlink import mavutil

COPTER_MODES = {"STABILIZE": 0, "ACRO": 1, "ALT_HOLD": 2, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6,
                "CIRCLE": 7, "LAND": 9, "BRAKE": 17, "POSHOLD": 16, "SMART_RTL": 21}


def connect(addr="tcp:127.0.0.1:5762"):
    m = mavutil.mavlink_connection(addr, source_system=255, source_component=190)
    if m.wait_heartbeat(timeout=8) is None:
        sys.exit("no heartbeat from SITL on %s" % addr)
    return m


def set_mode(m, name):
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, COPTER_MODES[name.upper()], 0, 0, 0, 0, 0)
    t0 = time.time()
    while time.time() - t0 < 4:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is not None and hb.custom_mode == COPTER_MODES[name.upper()]:
            return True
    return False


def set_param(m, name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    t0 = time.time()
    while time.time() - t0 < 4:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p is not None and p.param_id.rstrip("\x00") == name:
            return p.param_value
    return None


def hold_rc(m, chan, pwm, seconds):
    vals = [65535] * 18                       # UINT16_MAX = ignore every other channel
    vals[chan - 1] = int(pwm)
    t_end = time.time() + seconds
    while time.time() < t_end:
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)
        time.sleep(0.2)
    vals[chan - 1] = 0                        # 0 = release the channel
    for _ in range(3):
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)
        time.sleep(0.1)


def hold_rc_sequence(m, chan, steps):
    """steps: [(pwm, seconds), ...] on ONE connection, so the timing between steps is exact (a pilot's 3-position switch)."""
    for pwm, secs in steps:
        vals = [65535] * 18
        vals[chan - 1] = int(pwm)
        t_end = time.time() + secs
        while time.time() < t_end:
            m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)
            time.sleep(0.1)
    vals = [65535] * 18
    vals[chan - 1] = 0
    for _ in range(3):
        m.mav.rc_channels_override_send(m.target_system, m.target_component, *vals)
        time.sleep(0.1)


def wind(m, speed, direction=None):
    out = {"SIM_WIND_SPD": set_param(m, "SIM_WIND_SPD", speed)}
    if direction is not None:
        out["SIM_WIND_DIR"] = set_param(m, "SIM_WIND_DIR", direction)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("mode").add_argument("name")
    w = sub.add_parser("wind"); w.add_argument("speed", type=float); w.add_argument("direction", type=float, nargs="?")
    r = sub.add_parser("rc"); r.add_argument("channel", type=int); r.add_argument("pwm", type=int)
    r.add_argument("--seconds", type=float, default=2.0)
    q = sub.add_parser("rcseq"); q.add_argument("channel", type=int); q.add_argument("steps", nargs="+", help="PWM:SECONDS ...")
    p = sub.add_parser("param"); p.add_argument("name"); p.add_argument("value", type=float)
    a = ap.parse_args()
    m = connect(a.connect)
    if a.cmd == "mode":
        print("mode", a.name, "->", "ok" if set_mode(m, a.name) else "NOT confirmed")
    elif a.cmd == "wind":
        print(wind(m, a.speed, a.direction))
    elif a.cmd == "rc":
        hold_rc(m, a.channel, a.pwm, a.seconds); print("rc released")
    elif a.cmd == "rcseq":
        hold_rc_sequence(m, a.channel, [(int(x.split(":")[0]), float(x.split(":")[1])) for x in a.steps]); print("rc sequence done, released")
    elif a.cmd == "param":
        print(a.name, "=", set_param(m, a.name, a.value))


if __name__ == "__main__":
    main()
