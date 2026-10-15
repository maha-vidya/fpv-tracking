"""Phase 0 SITL test (see CUBE_ORANGE_PARAMS.md). Starts its OWN SITL instance 1 (ports 5770/5772) and kills only that process,
so it cannot disturb a SITL you are running on 5760. Needs ~/ardupilot built for SITL."""
import os, math, subprocess, sys, tempfile, time
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
mv = mavutil.mavlink
wd = tempfile.mkdtemp(prefix='fs_test_'); ap = os.path.expanduser('~/ardupilot')
open(wd + '/extra.parm', 'w').write("GUID_TIMEOUT 1.0\nFS_GCS_ENABLE 5\nFS_GCS_TIMEOUT 2\n")
sitl = subprocess.Popen([ap + '/build/sitl/bin/arducopter', '-w', '--model', '+', '--speedup', '1', '--defaults',
    ap + '/Tools/autotest/default_params/copter.parm,' + wd + '/extra.parm', '--sim-address=127.0.0.1', '-I1'],
    cwd=wd, stdout=open(wd + '/sitl.log', 'w'), stderr=subprocess.STDOUT)
time.sleep(4)
g = mavutil.mavlink_connection('tcp:127.0.0.1:5770', source_system=255, source_component=190); g.wait_heartbeat(timeout=20)
S = {'mode': None, 'armed': None, 'alt': 0.0, 'pos': (0, 0, 0), 'vel': (0, 0, 0), 'txt': []}
def hbg(): g.mav.heartbeat_send(mv.MAV_TYPE_GCS, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE)
for mid, hz in [(32, 10), (33, 10), (1, 2)]: g.mav.command_long_send(1, 1, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
send_g_hb = [True]
def poll(dur):
    t0 = time.time()
    while time.time() - t0 < dur:
        if send_g_hb[0]: hbg()
        r = g.recv_match(blocking=True, timeout=0.05)
        if r is None: continue
        t = r.get_type()
        if t == 'HEARTBEAT' and r.get_srcSystem() == 1 and r.get_srcComponent() == 1: S['mode'] = r.custom_mode; S['armed'] = bool(r.base_mode & 128)
        elif t == 'LOCAL_POSITION_NED': S['pos'] = (r.x, r.y, r.z); S['vel'] = (r.vx, r.vy, r.vz)
        elif t == 'GLOBAL_POSITION_INT': S['alt'] = r.relative_alt / 1000.0
        elif t == 'STATUSTEXT': S['txt'].append(r.text); print('   STATUSTEXT:', r.text, flush=True)
def spd(): return math.hypot(S['vel'][0], S['vel'][1])
def vel(vx): g.mav.set_position_target_local_ned_send(0, 1, 1, mv.MAV_FRAME_BODY_OFFSET_NED, 0x0DC7, 0, 0, 0, vx, 0, 0, 0, 0, 0, 0, 0)
def stream(dur, vx):
    t0 = time.time()
    while time.time() - t0 < dur: vel(vx); poll(0.1)
t0 = time.time()
while time.time() - t0 < 90 and not any('using GPS' in t for t in S['txt']): poll(1.0)
g.mav.command_long_send(1, 1, mv.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0); poll(1.5)
g.mav.command_long_send(1, 1, mv.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0); poll(2.5)
g.mav.command_long_send(1, 1, mv.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
t1 = time.time()
while time.time() - t1 < 40 and S['alt'] < 9.0: poll(0.5)
print(f'airborne: alt {S["alt"]:.1f} m, mode {S["mode"]}', flush=True); poll(3)
print('\nT1. GUID_TIMEOUT = 1.0 s: vx=+2 m/s for 4 s, then the companion "dies" (no more commands)', flush=True)
stream(4.0, 2.0); print(f'   speed when commands stop: {spd():.2f} m/s', flush=True); pb = S['pos']
for k in range(5):
    poll(1.0); print(f'   +{k+1} s without commands: speed {spd():.2f} m/s', flush=True)
print(f'   extra distance after commands stopped: {math.hypot(S["pos"][0]-pb[0], S["pos"][1]-pb[1]):.1f} m   (with GUID_TIMEOUT=3 s it was 7.7 m)', flush=True)
print('\nT2. GCS failsafe (FS_GCS_ENABLE=5 Land, FS_GCS_TIMEOUT=2 s): the real GCS link goes silent', flush=True)
comp = mavutil.mavlink_connection('tcp:127.0.0.1:5772', source_system=255, source_component=191)
def hbc(): comp.mav.heartbeat_send(mv.MAV_TYPE_ONBOARD_CONTROLLER, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE)
t = time.time()
while time.time() - t < 4: hbc(); poll(0.5)
send_g_hb[0] = False                    # the pilot's ground station stops sending heartbeats
print('   (a) real GCS silent; companion keeps sending HEARTBEAT with sysid 255 (= inside the GCS id range) for 8 s ...', flush=True)
t = time.time()
while time.time() - t < 8: hbc(); poll(0.5)
print(f'       mode after 8 s: {S["mode"]}  ->', 'NO FAILSAFE (companion heartbeat masks the lost GCS!)' if S['mode'] == 4 else f'mode changed to {S["mode"]}', flush=True)
print('   (b) companion switches to its own system id 190 (outside the GCS range) ...', flush=True)
comp.mav.srcSystem = 190
t = time.time()
while time.time() - t < 8 and S['mode'] == 4: hbc(); poll(0.5)
print(f'       mode: {S["mode"]} (9 = LAND)  ->', 'FAILSAFE TRIGGERED' if S['mode'] == 9 else 'no failsafe', f'({time.time()-t:.1f} s)', flush=True)
sitl.kill()
