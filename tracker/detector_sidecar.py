#!/usr/bin/env python3
"""ReID sidecar for target_tracker: DINOv2-small appearance fingerprints over localhost UDP.

Only needed when target_tracker was built WITHOUT OpenVINO -- normally `--reid` runs the same model inside the tracker (native_ai.hpp).
No detector and no segmenter: the project tracks any clicked object without classes (the YOLO / FastSAM / MobileSAM / EdgeSAM paths
were removed 2026-10-15; they were never used).

  python3 detector_sidecar.py --reid ~/models/dinov2            # then: target_tracker ... --reid --ai sidecar

Protocol (127.0.0.1):
  tracker -> sidecar (port P):    "TTE1 <id>\\n" + JPEG crop
  sidecar -> tracker (port P+1):  "TTV1 <id> <n> v1 ... vn"   (n = 0: could not embed)
"""
import argparse
import os
import queue
import socket
import sys
import threading
import time

import cv2
import numpy as np


class DinoReid:
    """Class-free appearance fingerprint for ReID: DINOv2-small (Apache-2.0; ONNX from huggingface.co/onnx-community/dinov2-small,
    onnx/model.onnx). A crop -> 768 numbers (CLS token + mean of the patch tokens, unit length); two crops of the same object give a
    high cosine similarity. It captures SHAPE / texture, not colour (a red and an orange box scored 0.89-0.91), so the tracker FUSES it
    with its colour check. 112 px input: ~45-75 ms per crop on the laptop (OpenVINO)."""
    SIDE = 112
    MEAN = np.array([0.485, 0.456, 0.406], np.float32)
    STD = np.array([0.229, 0.224, 0.225], np.float32)

    def __init__(self, model_dir, threads=0, runtime="auto"):
        path = os.path.join(model_dir, "model.onnx")
        self.runtime = None
        if runtime in ("auto", "openvino"):
            try:
                import openvino as ov
                core = ov.Core(); core.set_property({"CACHE_DIR": os.path.expanduser("~/.cache/openvino_models")})
                cfg = {"PERFORMANCE_HINT": "LATENCY"}
                if threads > 0: cfg["INFERENCE_NUM_THREADS"] = threads
                m = core.compile_model(path, "CPU", cfg); self.run = lambda x: m(x)[0]
                self.runtime = "openvino " + ov.__version__.split("-")[0]
            except ImportError:
                if runtime == "openvino": raise
        if self.runtime is None:
            import onnxruntime as ort
            so = ort.SessionOptions()
            if threads > 0: so.intra_op_num_threads = threads
            sess = ort.InferenceSession(path, so, providers=["CPUExecutionProvider"]); self.run = lambda x: sess.run(None, {"pixel_values": x})[0]
            self.runtime = "onnxruntime"

    def embed(self, crop_bgr):
        x = cv2.resize(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB), (self.SIDE, self.SIDE), interpolation=cv2.INTER_CUBIC).astype(np.float32) / 255.0
        h = self.run(np.ascontiguousarray(((x - self.MEAN) / self.STD).transpose(2, 0, 1)[None]))[0]
        v = np.concatenate([h[0], h[1:].mean(0)])
        return v / (np.linalg.norm(v) + 1e-9)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=5810)
    ap.add_argument("--reid", required=True, help="directory with DINOv2-small model.onnx")
    ap.add_argument("--threads", type=int, default=0, help="inference threads (0 = default)")
    ap.add_argument("--runtime", choices=["auto", "openvino", "onnxruntime"], default="auto",
                    help="OpenVINO when installed (about 2x faster on an Intel CPU), else onnxruntime (e.g. Raspberry Pi)")
    a = ap.parse_args()
    reid = DinoReid(os.path.expanduser(a.reid), a.threads, a.runtime)
    try: reid.embed(np.zeros((112, 112, 3), np.uint8))                 # first inference compiles / allocates: not on a live request
    except Exception as e: print("[sidecar] warm-up failed: %s" % e, flush=True)   # noqa: BLE001
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    rx.bind(("127.0.0.1", a.port))
    reply_to = ("127.0.0.1", a.port + 1)
    print("[sidecar] DINOv2 ReID on %s, listening on 127.0.0.1:%d" % (reid.runtime, a.port), flush=True)
    q = queue.Queue()

    def worker():
        while True:
            t_in, p = q.get()
            try:
                hdr, _, jpg = p.partition(b"\n"); rid = hdr.split()[1].decode()
                if time.time() - t_in > 0.5:                                    # the tracker has given up on it: no backlog
                    rx.sendto(("TTV1 %s 0" % rid).encode(), reply_to); continue
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    v = reid.embed(img); msg = "TTV1 %s %d %s" % (rid, len(v), " ".join("%.4f" % f for f in v))
                else:
                    msg = "TTV1 %s 0" % rid
                rx.sendto(msg.encode(), reply_to)
            except Exception as e:                                           # noqa: BLE001 -- one bad packet must not stop the service
                print("[sidecar] bad request: %s" % e, flush=True)

    threading.Thread(target=worker, daemon=True).start()
    while True:
        p = rx.recv(70000)
        if p.startswith(b"TTE1"): q.put((time.time(), p))


if __name__ == "__main__":
    main()
