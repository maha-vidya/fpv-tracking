#!/usr/bin/env python3
"""Record the simulated drone camera straight from Gazebo (no tracker, no MAVLink -- safe while a tracker is running) for the
offline tracking benchmark (tracker/tests/track_bench.py).

  python3 simulation/record_camera.py ~/sim_run/recordings/scene.avi 90
"""
import os, sys, time
import cv2, numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"

def main():
    if len(sys.argv) < 3: sys.exit(__doc__)
    out, secs = os.path.expanduser(sys.argv[1]), float(sys.argv[2])
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    vw = [None]; n = [0]
    def cb(m):
        im = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)
        if vw[0] is None: vw[0] = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"MJPG"), 20, (im.shape[1], im.shape[0]))
        vw[0].write(im); n[0] += 1
    node = Node(); node.subscribe(Image, TOPIC, cb)
    t = time.time()
    while time.time() - t < secs: time.sleep(0.5)
    if vw[0]: vw[0].release()
    print("%d frames -> %s" % (n[0], out))

if __name__ == "__main__":
    main()
