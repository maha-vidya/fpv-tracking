#!/usr/bin/env python3
"""Tiny TCP forwarder between the companion and a SITL serial-like port, so a LINK LOSS can be injected by freezing this
process (simulation/fault.sh link-loss N sends SIGSTOP/SIGCONT to its process group).

  python3 mav_gate.py --listen 5770 --to 127.0.0.1:5763      # the follower then uses tcp:127.0.0.1:5770

ONE persistent upstream connection for the life of this process; companion clients come and go behind it (the newest client
gets the vehicle's bytes). Previously every client opened its own upstream connection: SITL's TCP serial port serves one client
and did not re-accept after a client left, so after a few tracker restarts the companion link stayed DOWN ("upstream refused:
timed out", the old connection still ESTABLISHED inside SITL).
"""
import argparse
import socket
import threading
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--listen", type=int, default=5770)
    ap.add_argument("--to", default="127.0.0.1:5763")
    a = ap.parse_args()
    host, port = a.to.rsplit(":", 1)
    srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.listen)); srv.listen(2)
    print("mav_gate: 127.0.0.1:%d -> %s" % (a.listen, a.to), flush=True)

    state = {"up": None, "client": None}
    lock = threading.Lock()

    def upstream():                                        # connect once (retrying), then vehicle -> current client, forever
        while True:
            try:
                u = socket.create_connection((host, int(port)), timeout=5); u.settimeout(None)
                print("upstream connected", flush=True)
            except OSError as e:
                print("upstream connect failed (%s), retrying" % e, flush=True); time.sleep(2); continue
            with lock: state["up"] = u
            try:
                while True:
                    d = u.recv(4096)
                    if not d: break
                    with lock: c = state["client"]
                    if c is not None:
                        try: c.sendall(d)
                        except OSError: pass
            except OSError:
                pass
            print("upstream closed, reconnecting", flush=True)
            with lock: state["up"] = None
            try: u.close()
            except OSError: pass
            time.sleep(1)

    def client_reader(c):                                  # client -> vehicle, until the client leaves
        try:
            while True:
                d = c.recv(4096)
                if not d: break
                with lock: u = state["up"]
                if u is not None:
                    try: u.sendall(d)
                    except OSError: pass
        except OSError:
            pass
        with lock:
            if state["client"] is c: state["client"] = None
        try: c.close()
        except OSError: pass
        print("client left", flush=True)

    threading.Thread(target=upstream, daemon=True).start()
    while True:
        c, addr = srv.accept()
        with lock:
            old = state["client"]; state["client"] = c
        if old is not None:                                # a newer client replaces the old one
            try: old.close()
            except OSError: pass
        print("client connected from %s:%d" % addr, flush=True)
        threading.Thread(target=client_reader, args=(c,), daemon=True).start()


if __name__ == "__main__":
    main()
