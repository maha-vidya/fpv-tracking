#!/usr/bin/env python3
"""Acts as a GROUND STATION: checks that the fake camera component is visible through ArduPilot's routing and
that CAMERA_INFORMATION, tracking commands, ACKs and CAMERA_TRACKING_IMAGE_STATUS all make the round trip."""
import os, sys, time
os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil
mv = mavutil.mavlink
gcs = mavutil.mavlink_connection(sys.argv[1] if len(sys.argv) > 1 else 'tcp:127.0.0.1:5760', source_system=255, source_component=190)
gcs.wait_heartbeat(timeout=15)
def hb(): gcs.mav.heartbeat_send(mv.MAV_TYPE_GCS, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE)   # QGC sends this at 1 Hz; ArduPilot only forwards to channels it has heard from
hb()
cam = None; t0 = time.time()
while time.time() - t0 < 30 and not cam:
    hb()
    msg = gcs.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg and msg.type == mv.MAV_TYPE_CAMERA: cam = (msg.get_srcSystem(), msg.get_srcComponent())
print('1) camera heartbeat seen via the flight controller:', cam)
if not cam: sys.exit(1)
def cmd(c, *p, wait=3.0):
    p = list(p) + [0] * (7 - len(p)); ack = None
    gcs.mav.command_long_send(cam[0], cam[1], c, 0, *p)
    t = time.time()
    while time.time() - t < wait:
        hb()
        r = gcs.recv_match(type=['COMMAND_ACK', 'CAMERA_INFORMATION', 'VIDEO_STREAM_INFORMATION', 'CAMERA_TRACKING_IMAGE_STATUS'], blocking=True, timeout=0.5)
        if r is None: continue
        if r.get_type() == 'COMMAND_ACK' and r.command == c:
            ack = r; wait = min(wait, (time.time() - t) + 1.0)      # keep listening 1 s: info messages follow the ACK
            continue
        yield_list.append(r)
    return ack
yield_list = []
a = cmd(mv.MAV_CMD_REQUEST_MESSAGE, mv.MAVLINK_MSG_ID_CAMERA_INFORMATION)
info = [r for r in yield_list if r.get_type() == 'CAMERA_INFORMATION']
print('2) REQUEST_MESSAGE(CAMERA_INFORMATION): ack result =', a.result if a else None, '| CAMERA_INFORMATION received:', bool(info),
      '| tracking flags:', bool(info and info[0].flags & mv.CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE and info[0].flags & mv.CAMERA_CAP_FLAGS_HAS_TRACKING_POINT))
yield_list.clear()
a = cmd(mv.MAV_CMD_REQUEST_MESSAGE, mv.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION)
vs = [r for r in yield_list if r.get_type() == 'VIDEO_STREAM_INFORMATION']
print('3) VIDEO_STREAM_INFORMATION received:', bool(vs), '| uri:', vs[0].uri if vs else None)
yield_list.clear()
a = cmd(mv.MAV_CMD_CAMERA_TRACK_RECTANGLE, 0.40, 0.30, 0.60, 0.70)
print('4) TRACK_RECTANGLE ack result =', a.result if a else None, '(0 = ACCEPTED)')
a = cmd(mv.MAV_CMD_SET_MESSAGE_INTERVAL, mv.MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS, 200000)
t = time.time(); st = []
while time.time() - t < 2.0:
    hb()
    r = gcs.recv_match(type='CAMERA_TRACKING_IMAGE_STATUS', blocking=True, timeout=0.5)
    if r: st.append(r)
print('5) CAMERA_TRACKING_IMAGE_STATUS messages in 2 s:', len(st), '| first:', (st[0].tracking_status, st[0].tracking_mode, round(st[0].rec_top_x, 2), round(st[0].rec_top_y, 2), round(st[0].rec_bottom_x, 2), round(st[0].rec_bottom_y, 2)) if st else None)
a = cmd(mv.MAV_CMD_CAMERA_TRACK_POINT, 0.5, 0.5, 0.05)
print('6) TRACK_POINT ack result =', a.result if a else None)
a = cmd(mv.MAV_CMD_CAMERA_STOP_TRACKING)
print('7) STOP_TRACKING ack result =', a.result if a else None)
