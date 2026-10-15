#!/usr/bin/env python3
"""Phase 0 (item 2) – SITL checks for the GPS-based follow design. No project code involved.

 A. GUIDED velocity commands (body frame, type mask 0x0DC7 + yaw-rate variant): response, stopping distance.
 B. The 3-second command timeout (what the drone does if the companion dies mid-command).
 C. Pilot take-back: after the COMPANION switched to GUIDED over MAVLink, does moving the transmitter's
    flight-mode switch regain control? (ArduPilot reads the switch on CHANGE only.)  The transmitter is emulated
    with RC_CHANNELS_OVERRIDE sent from the GCS system id (255 = SYSID_MYGCS) – overrides from other ids are ignored.
 D. Stick-movement detection: do RC_CHANNELS expose the pilot's stick values (for a "pilot moved sticks" takeover)?

Usage:  python3 guided_takeback_test.py --start-sitl      (starts its OWN SITL instance 1 on ports 5770/5772; kills nothing else)
        python3 guided_takeback_test.py --connect tcp:127.0.0.1:5770
"""
import argparse, math, os, subprocess, sys, tempfile, time
os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil
mv = mavutil.mavlink
ap = argparse.ArgumentParser()
ap.add_argument('--connect', default='tcp:127.0.0.1:5770'); ap.add_argument('--start-sitl', action='store_true')
ap.add_argument('--ardupilot', default=os.path.expanduser('~/ardupilot'))
a = ap.parse_args()
sitl = None
if a.start_sitl:
    wd = tempfile.mkdtemp(prefix='sitl_takeback_')
    sitl = subprocess.Popen([a.ardupilot + '/build/sitl/bin/arducopter', '-w', '--model', '+', '--speedup', '1', '--defaults',
                             a.ardupilot + '/Tools/autotest/default_params/copter.parm', '--sim-address=127.0.0.1', '-I1'],
                            cwd=wd, stdout=open(wd + '/sitl.log', 'w'), stderr=subprocess.STDOUT)
    time.sleep(4)
m = mavutil.mavlink_connection(a.connect, source_system=255, source_component=190)   # 255 = the ground station
m.wait_heartbeat(timeout=20)
S = {'mode': None, 'armed': None, 'alt': 0.0, 'pos': (0, 0, 0), 'vel': (0, 0, 0), 'rc': None, 'roll': 0, 'pitch': 0, 'txt': []}
def hb(): m.mav.heartbeat_send(mv.MAV_TYPE_GCS, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE)
def interval(mid, hz): m.mav.command_long_send(1, 1, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
for mid, hz in [(32, 10), (33, 10), (65, 10), (30, 10), (1, 2)]: interval(mid, hz)
def poll(dur):
    t0 = time.time()
    while time.time() - t0 < dur:
        hb(); r = m.recv_match(blocking=True, timeout=0.05)
        if r is None: continue
        t = r.get_type()
        if t == 'HEARTBEAT' and r.get_srcSystem() == 1 and r.get_srcComponent() == 1: S['mode'] = r.custom_mode; S['armed'] = bool(r.base_mode & 128)
        elif t == 'LOCAL_POSITION_NED': S['pos'] = (r.x, r.y, r.z); S['vel'] = (r.vx, r.vy, r.vz)
        elif t == 'GLOBAL_POSITION_INT': S['alt'] = r.relative_alt / 1000.0
        elif t == 'RC_CHANNELS': S['rc'] = (r.chan1_raw, r.chan2_raw, r.chan3_raw, r.chan4_raw, r.chan5_raw)
        elif t == 'ATTITUDE': S['roll'], S['pitch'] = math.degrees(r.roll), math.degrees(r.pitch)
        elif t == 'STATUSTEXT': S['txt'].append(r.text)
def spd(): return math.hypot(S['vel'][0], S['vel'][1])
def setmode(n): m.mav.command_long_send(1, 1, mv.MAV_CMD_DO_SET_MODE, 0, 1, n, 0, 0, 0, 0, 0)
def vel(vx, vy, vz, yaw_rate=0.0, mask=0x0DC7):
    # body frame (MAV_FRAME_BODY_OFFSET_NED = 9): velocity relative to the vehicle's heading
    m.mav.set_position_target_local_ned_send(0, 1, 1, mv.MAV_FRAME_BODY_OFFSET_NED, mask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)
def stream_vel(dur, vx, vy, vz, yaw_rate=0.0, mask=0x0DC7, hz=10):
    t0 = time.time()
    while time.time() - t0 < dur:
        vel(vx, vy, vz, yaw_rate, mask); poll(1.0 / hz)
print('waiting for GPS/EKF (up to 90 s)...')
t0 = time.time()
while time.time() - t0 < 90 and not any('using GPS' in t for t in S['txt']): poll(1.0)
print('  EKF using GPS:', any('using GPS' in t for t in S['txt']), f'({time.time()-t0:.0f} s)')
setmode(4); poll(1.5); print('mode GUIDED ->', S['mode'])
m.mav.command_long_send(1, 1, mv.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0); poll(2.5); print('armed:', S['armed'], [t for t in S['txt'][-2:]])
m.mav.command_long_send(1, 1, mv.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10); t1 = time.time()
while time.time() - t1 < 40 and S['alt'] < 9.0: poll(0.5)
print(f'takeoff: alt = {S["alt"]:.1f} m ({time.time()-t1:.0f} s)'); poll(4)

print('\nA. forward velocity command vx = +2 m/s (body frame) for 5 s, then zero-velocity commands for 8 s')
p0 = S['pos']; stream_vel(5.0, 2.0, 0, 0); print(f'   after 5 s: ground speed {spd():.2f} m/s, moved {math.hypot(S["pos"][0]-p0[0], S["pos"][1]-p0[1]):.1f} m')
p1 = S['pos']
for k in range(4):
    stream_vel(2.0, 0, 0, 0); print(f'   zero-velocity cmd +{2*(k+1)} s: speed {spd():.2f} m/s')
print(f'   extra distance while stopping with zero-velocity commands: {math.hypot(S["pos"][0]-p1[0], S["pos"][1]-p1[1]):.1f} m')
print('   yaw-rate variant (mask 0x07C7 = our app\'s mask, with yaw-rate 0.3 rad/s) – does it still accept?')
stream_vel(3.0, 0, 0, 0, yaw_rate=0.3, mask=0b0000011111000111); print(f'   speed {spd():.2f} m/s (just checking the message is accepted; yaw rate not read back here)')
stream_vel(2.0, 0, 0, 0)

print('\nB. companion "dies": vx = +2 m/s for 4 s, then NO more commands')
stream_vel(4.0, 2.0, 0, 0); print(f'   speed when commands stop: {spd():.2f} m/s'); pb = S['pos']
for k in range(8):
    poll(1.0); print(f'   +{k+1} s without commands: speed {spd():.2f} m/s, mode {S["mode"]}')
print(f'   extra distance after commands stopped: {math.hypot(S["pos"][0]-pb[0], S["pos"][1]-pb[1]):.1f} m')

print('\nC. pilot take-back (transmitter emulated by RC override from sysid 255, FLTMODE_CH=5)')
setmode(4); poll(1.5); print('   companion set GUIDED over MAVLink: mode =', S['mode'])
stream_vel(3.0, 0.5, 0, 0)
print('   switch NOT moved for 3 s: mode =', S['mode'], '(stays GUIDED -> FC does not fall back to the switch by itself)' if S['mode'] == 4 else '(CHANGED!)')
t = time.time()
while time.time() - t < 4:
    m.mav.rc_channels_override_send(1, 1, 1500, 1500, 1500, 1500, 1700, 0, 0, 0); hb(); poll(0.1)
print('   switch moved to slot 5 (PWM 1700 = LOITER in this param set): mode =', S['mode'], '(5 = LOITER) ->', 'TAKE-BACK WORKS' if S['mode'] == 5 else 'no change')
print('D. RC_CHANNELS as seen by the companion while the "pilot" holds roll stick at 1700:')
t = time.time()
while time.time() - t < 2:
    m.mav.rc_channels_override_send(1, 1, 1700, 1500, 1500, 1500, 1700, 0, 0, 0); hb(); poll(0.1)
print('   chan1..5 =', S['rc'], '-> the companion can see stick movement' if S['rc'] and S['rc'][0] == 1700 else '-> not visible')
for _ in range(15): m.mav.rc_channels_override_send(1, 1, 0, 0, 0, 0, 0, 0, 0, 0); hb(); poll(0.1)
if sitl: sitl.kill()
