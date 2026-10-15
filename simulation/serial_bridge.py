#!/usr/bin/env python3
"""Present a SITL TCP port as a SERIAL device (a pseudo-terminal), to rehearse the Pi <-> Cube Orange UART path on a laptop.

  python3 serial_bridge.py --to 127.0.0.1:5763        # prints e.g. /dev/pts/7
  target_tracker --connection serial:/dev/pts/7:921600 --link-test 10

It forwards raw bytes both ways. A pty ignores the baud rate, so this proves the open/termios/read/write path, NOT UART timing.
"""
import argparse, os, pty, select, socket, sys, tty

ap = argparse.ArgumentParser()
ap.add_argument("--to", default="127.0.0.1:5763")
a = ap.parse_args()
host, port = a.to.rsplit(":", 1)
master, slave = pty.openpty()
tty.setraw(master)
print(os.ttyname(slave), flush=True)
s = socket.create_connection((host, int(port)))
while True:
    r, _, _ = select.select([master, s], [], [], 1.0)
    if master in r:
        d = os.read(master, 4096)
        if d: s.sendall(d)
    if s in r:
        d = s.recv(4096)
        if not d: break
        os.write(master, d)
