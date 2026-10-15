#!/usr/bin/env python3
"""UDP echo server for the G20 <-> Pi link test (docs/phase0/G20_CAMERA_TESTS.md T3). Usage: python3 udp_echo.py [port]"""
import socket, sys
port = int(sys.argv[1]) if len(sys.argv) > 1 else 9000
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", port))
print("echoing UDP on port", port, "(Ctrl+C to stop)", flush=True)
while True:
    data, addr = s.recvfrom(2048)
    s.sendto(data, addr)
