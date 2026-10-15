#!/usr/bin/env python3
"""Fake MAVLink *camera component* that supports tracking (MAVLink Camera Protocol v2).
It stands in for the future Raspberry Pi so we can check, before writing the real code, that
  (1) ArduPilot routes our component's messages to the ground station (and commands back),
  (2) QGroundControl (the version we will fly with) offers click-to-track on the video, and
  (3) which commands/fields QGC actually sends (MAV_CMD_CAMERA_TRACK_POINT / _RECTANGLE).
Everything it receives is printed. It does not fly or control anything.
"""
import os, sys, time, math
os.environ.setdefault('MAVLINK20', '1')
import argparse
from pymavlink import mavutil
mv = mavutil.mavlink

ap = argparse.ArgumentParser()
ap.add_argument('--connect', default='tcp:127.0.0.1:5762', help='link to the flight controller (the companion port)')
ap.add_argument('--sysid', type=int, default=1, help='same system id as the vehicle (a camera is a COMPONENT of the vehicle)')
ap.add_argument('--compid', type=int, default=mv.MAV_COMP_ID_CAMERA)
ap.add_argument('--video-uri', default='udp://127.0.0.1:5600', help='where the video is (shown to QGC in VIDEO_STREAM_INFORMATION)')
ap.add_argument('--video-type', default='rtpudp', choices=['rtsp', 'rtpudp'])
ap.add_argument('--width', type=int, default=1280); ap.add_argument('--height', type=int, default=720)
ap.add_argument('--fps', type=float, default=30.0)
ap.add_argument('--seconds', type=float, default=0, help='exit after N seconds (0 = run until Ctrl+C)')
a = ap.parse_args()

m = mavutil.mavlink_connection(a.connect, source_system=a.sysid, source_component=a.compid)
t0 = time.time()
track = {'mode': 0, 'status': 0, 'rect': None, 'point': None, 'interval_us': 0}
log = []

def boot_ms(): return int((time.time() - t0) * 1000)
def pad(s, n): return (s.encode()[:n]).ljust(n, b'\0')

def send_camera_information():
    flags = (mv.CAMERA_CAP_FLAGS_CAPTURE_VIDEO | mv.CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM |
             mv.CAMERA_CAP_FLAGS_HAS_TRACKING_POINT | mv.CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE)
    m.mav.camera_information_send(boot_ms(), pad('Phase0', 32), pad('FakeTrackingCam', 32), 0x01000000,
                                  float('nan'), float('nan'), float('nan'), a.width, a.height, 0, flags, 0, b'')
    print(f'[tx] CAMERA_INFORMATION flags={flags} ({a.width}x{a.height})')

def send_video_stream_information():
    t = mv.VIDEO_STREAM_TYPE_RTSP if a.video_type == 'rtsp' else mv.VIDEO_STREAM_TYPE_RTPUDP
    m.mav.video_stream_information_send(1, 1, t, mv.VIDEO_STREAM_STATUS_FLAGS_RUNNING, a.fps, a.width, a.height,
                                        4000, 0, 90, b'Phase0 test video', a.video_uri.encode()[:160])
    print(f'[tx] VIDEO_STREAM_INFORMATION uri={a.video_uri}')

def send_tracking_status():
    nan = float('nan')
    if track['mode'] == 0:
        m.mav.camera_tracking_image_status_send(0, 0, 0, nan, nan, nan, nan, nan, nan, nan)
    elif track['mode'] == 1:
        x, y, r = track['point']
        m.mav.camera_tracking_image_status_send(mv.CAMERA_TRACKING_STATUS_FLAGS_ACTIVE, 1, mv.CAMERA_TRACKING_TARGET_DATA_IN_STATUS, x, y, r, nan, nan, nan, nan)
    else:
        x0, y0, x1, y1 = track['rect']
        m.mav.camera_tracking_image_status_send(mv.CAMERA_TRACKING_STATUS_FLAGS_ACTIVE, 2, mv.CAMERA_TRACKING_TARGET_DATA_IN_STATUS, nan, nan, nan, x0, y0, x1, y1)

def ack(cmd, result, src_sys, src_comp):
    m.mav.command_ack_send(cmd, result, 0, 0, src_sys, src_comp)

def handle_command(msg, params, cmd):
    s, c = msg.get_srcSystem(), msg.get_srcComponent()
    if cmd == mv.MAV_CMD_REQUEST_MESSAGE:
        mid = int(params[0]); ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c)
        if mid == mv.MAVLINK_MSG_ID_CAMERA_INFORMATION: send_camera_information()
        elif mid == mv.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION: send_video_stream_information()
        elif mid == mv.MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS: send_tracking_status()
        else: print(f'[rx] REQUEST_MESSAGE for unsupported id {mid}')
    elif cmd == mv.MAV_CMD_REQUEST_CAMERA_INFORMATION:
        ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c); send_camera_information()
    elif cmd == mv.MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION:
        ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c); send_video_stream_information()
    elif cmd == mv.MAV_CMD_SET_MESSAGE_INTERVAL:
        mid, us = int(params[0]), int(params[1])
        if mid == mv.MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS:
            track['interval_us'] = max(0, us); ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c)
        else: ack(cmd, mv.MAV_RESULT_UNSUPPORTED, s, c)
    elif cmd == mv.MAV_CMD_CAMERA_TRACK_POINT:
        track.update(mode=1, point=(params[0], params[1], params[2])); ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c)
        print(f'[rx] ***** TRACK_POINT  x={params[0]:.3f} y={params[1]:.3f} radius={params[2]:.3f}  from {s}/{c}')
        log.append(('track_point', params[:3]))
    elif cmd == mv.MAV_CMD_CAMERA_TRACK_RECTANGLE:
        track.update(mode=2, rect=(params[0], params[1], params[2], params[3])); ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c)
        print(f'[rx] ***** TRACK_RECTANGLE top-left=({params[0]:.3f},{params[1]:.3f}) bottom-right=({params[2]:.3f},{params[3]:.3f})  from {s}/{c}')
        log.append(('track_rect', params[:4]))
    elif cmd == mv.MAV_CMD_CAMERA_STOP_TRACKING:
        track.update(mode=0); ack(cmd, mv.MAV_RESULT_ACCEPTED, s, c); print(f'[rx] ***** STOP_TRACKING from {s}/{c}'); log.append(('stop', ()))
    else:
        print(f'[rx] command {cmd} from {s}/{c} (not handled)')

last_hb = 0; last_status = 0; n_rx = 0
print(f'camera component {a.sysid}/{a.compid} on {a.connect}; video {a.video_uri}')
while True:
    now = time.time()
    if a.seconds and now - t0 > a.seconds: break
    if now - last_hb >= 1.0:
        m.mav.heartbeat_send(mv.MAV_TYPE_CAMERA, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE); last_hb = now
    if track['interval_us'] and now - last_status >= track['interval_us'] / 1e6:
        send_tracking_status(); last_status = now
    msg = m.recv_match(blocking=True, timeout=0.05)
    if msg is None: continue
    t = msg.get_type()
    if t == 'COMMAND_LONG' and msg.target_component in (0, a.compid) and msg.target_system in (0, a.sysid):
        handle_command(msg, [msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7], msg.command)
    elif t == 'COMMAND_INT' and msg.target_component in (0, a.compid) and msg.target_system in (0, a.sysid):
        handle_command(msg, [msg.param1, msg.param2, msg.param3, msg.param4, msg.x, msg.y, msg.z], msg.command)
print('done; commands received:', log)
