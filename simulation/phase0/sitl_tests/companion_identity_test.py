"""Phase 0 SITL test (see CUBE_ORANGE_PARAMS.md). Starts its OWN SITL instance 1 (ports 5770/5772) and kills only that process,
so it cannot disturb a SITL you are running on 5760. Needs ~/ardupilot built for SITL."""
import os, math, subprocess, tempfile, time
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
mv = mavutil.mavlink
wd = tempfile.mkdtemp(prefix='cid_test_'); ap = os.path.expanduser('~/ardupilot')
open(wd + '/extra.parm', 'w').write("GUID_TIMEOUT 1.0\nFS_GCS_ENABLE 5\nFS_GCS_TIMEOUT 2\nWP_SPD 2.0\n")
sitl = subprocess.Popen([ap + '/build/sitl/bin/arducopter', '-w', '--model', '+', '--speedup', '1', '--defaults',
    ap + '/Tools/autotest/default_params/copter.parm,' + wd + '/extra.parm', '--sim-address=127.0.0.1', '-I1'], cwd=wd, stdout=open(wd + '/sitl.log', 'w'), stderr=subprocess.STDOUT)
time.sleep(4)
g = mavutil.mavlink_connection('tcp:127.0.0.1:5770', source_system=255, source_component=190); g.wait_heartbeat(timeout=20)
comp = mavutil.mavlink_connection('tcp:127.0.0.1:5772', source_system=1, source_component=191)   # companion = vehicle's system id, ONBOARD_COMPUTER component
S = {'mode': None, 'armed': None, 'alt': 0.0, 'vel': (0, 0, 0), 'txt': []}
g_hb = [True]; last_hbc = [0]
for mid, hz in [(32, 10), (33, 10), (1, 2)]: g.mav.command_long_send(1, 1, mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
def poll(dur):
    t0 = time.time()
    while time.time() - t0 < dur:
        if g_hb[0]: g.mav.heartbeat_send(mv.MAV_TYPE_GCS, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE)
        if time.time() - last_hbc[0] >= 1.0: comp.mav.heartbeat_send(mv.MAV_TYPE_ONBOARD_CONTROLLER, mv.MAV_AUTOPILOT_INVALID, 0, 0, mv.MAV_STATE_ACTIVE); last_hbc[0] = time.time()
        r = g.recv_match(blocking=True, timeout=0.05)
        if r is None: continue
        t = r.get_type()
        if t == 'HEARTBEAT' and r.get_srcSystem() == 1 and r.get_srcComponent() == 1: S['mode'] = r.custom_mode; S['armed'] = bool(r.base_mode & 128)
        elif t == 'LOCAL_POSITION_NED': S['vel'] = (r.vx, r.vy, r.vz)
        elif t == 'GLOBAL_POSITION_INT': S['alt'] = r.relative_alt / 1000.0
        elif t == 'STATUSTEXT': S['txt'].append(r.text)
def spd(): return math.hypot(S['vel'][0], S['vel'][1])
t0 = time.time()
while time.time() - t0 < 90 and not any('using GPS' in t for t in S['txt']): poll(1.0)
g.mav.command_long_send(1, 1, mv.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0); poll(1.5)
g.mav.command_long_send(1, 1, mv.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0); poll(2.5)
g.mav.command_long_send(1, 1, mv.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
t1 = time.time()
while time.time() - t1 < 40 and S['alt'] < 9.0: poll(0.5)
print(f'airborne: alt {S["alt"]:.1f} m, mode {S["mode"]}', flush=True); poll(3)
print('\nA. WP_SPD = 2.0 m/s as a parameter cap; the COMPANION (sysid 1, compid 191) commands vx = +4 m/s for 6 s', flush=True)
t = time.time(); peak = 0
while time.time() - t < 6:
    comp.mav.set_position_target_local_ned_send(0, 1, 1, mv.MAV_FRAME_BODY_OFFSET_NED, 0x0DC7, 0, 0, 0, 4.0, 0, 0, 0, 0, 0, 0, 0); poll(0.1); peak = max(peak, spd())
print(f'   peak ground speed: {peak:.2f} m/s (commanded 4.0) ->', 'CAPPED by WP_SPD' if peak < 2.6 else 'NOT capped', flush=True)
t = time.time()
while time.time() - t < 3:
    comp.mav.set_position_target_local_ned_send(0, 1, 1, mv.MAV_FRAME_BODY_OFFSET_NED, 0x0DC7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); poll(0.1)
print('\nB. companion (1/191) sends DO_SET_MODE -> LOITER(5), then -> GUIDED(4)', flush=True)
comp.mav.command_long_send(1, 1, mv.MAV_CMD_DO_SET_MODE, 0, 1, 5, 0, 0, 0, 0, 0); poll(2.0); m1 = S['mode']
comp.mav.command_long_send(1, 1, mv.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0); poll(2.0); m2 = S['mode']
print(f'   mode after request LOITER: {m1}, after request GUIDED: {m2} ->', 'ACCEPTED from the companion id' if (m1 == 5 and m2 == 4) else 'NOT accepted', flush=True)
print('\nC. GCS failsafe with the companion heartbeating as sysid 1 / compid 191 (real GCS goes silent)', flush=True)
g_hb[0] = False; t = time.time()
while time.time() - t < 9 and S['mode'] == 4: poll(0.5)
print(f'   mode after {time.time()-t:.1f} s: {S["mode"]} (9 = LAND) ->', 'FAILSAFE TRIGGERED (companion heartbeat does not mask a lost GCS)' if S['mode'] == 9 else 'no failsafe', flush=True)
sitl.kill()
