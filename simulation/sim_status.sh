#!/bin/bash
# Show what simulation/sim_up.sh started, whether it is alive, the drone/target poses and the SITL link.
RUN_DIR="${RUN_DIR:-$HOME/sim_run}"
[ -s "$RUN_DIR/pids" ] || { echo "simulation not started (no $RUN_DIR/pids)"; exit 1; }
while read -r name pid; do
  if kill -0 "$pid" 2>/dev/null; then echo "  up    $name (pgid $pid)"; else echo "  DEAD  $name  -> see $RUN_DIR/$name.log"; fi
done < "$RUN_DIR/pids"
echo "poses (Gazebo):"
gz topic -e -t /world/iris_runway/pose/info -n 1 2>/dev/null | python3 -c "
import sys,re
t=sys.stdin.read()
for n in ('iris_with_gimbal','target_red','lookalike_orange'):
    i=t.find('name: \"%s\"'%n)
    if i<0: print('  %-18s not in world'%n); continue
    b=t[i:i+500]; p=re.search(r'position \{(.*?)\}',b,re.S)
    v=dict(re.findall(r'([xyz]): (\S+)',p.group(1))) if p else {}
    print('  %-18s x=%.1f y=%.1f z=%.1f'%(n,*[float(v.get(k,0)) for k in 'xyz']))"
python3 - <<'E'
from pymavlink import mavutil
m=mavutil.mavlink_connection('tcp:127.0.0.1:5762')
hb=m.wait_heartbeat(timeout=4)
if not hb: print('SITL: no heartbeat on 5762')
else:
    print('SITL: mode_num=%d armed=%s'%(hb.custom_mode, bool(hb.base_mode & 128)))
E
