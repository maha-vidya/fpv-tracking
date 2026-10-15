#!/usr/bin/env python3
"""Tests for c12_protocol.py and fake_c12.py. No hardware needed. Python 3.6+.  Run:  python3 test_c12.py"""
import threading
import time
import unittest

import c12_protocol as p
from fake_c12 import FakeC12

# Every worked example in "TOP Protocol EN.docx" that has a full frame (checksum included).
DOC_EXAMPLES = [
    "#TPUG2wPTZ006A", "#TPUG2wGSYE276", "#TPUG8wGOT0280016895", "#TPUG2wSUM0162",
    "#TPUG2wSUM0061", "#TPUG2wGAA0136", "#TPUD2wREC0A54", "#TPUGCwFAIEC780000000064",
    "#TPUD2wCAP013E", "#TPUD2rVER0051", "#TPUD2rMOD0044", "#TPUG6wGAYEF073248",
    "#TPMU2wERE!!30", "#TPUG2wFAE0139", "#TPUD2wDZM0A65",
]


class Frames(unittest.TestCase):
    def test_doc_example_checksums(self):
        for ex in DOC_EXAMPLES:
            self.assertEqual(p.crc(ex[:-2]), ex[-2:], ex)

    def test_builders_match_doc(self):
        self.assertEqual(p.ptz("center")[:-2], "#TPUG2wPTZ05")
        self.assertEqual(p.build("U", "G", "w", "PTZ", "00"), "#TPUG2wPTZ006A")
        self.assertEqual(p.track_point(640, 360), "#TPUG8wGOT0280016895")
        self.assertEqual(p.track_confirm(), "#TPUG2wSUM0162")
        self.assertEqual(p.track_stop(), "#TPUG2wSUM0061")
        self.assertEqual(p.attitude_push(1), "#TPUG2wGAA0136")
        self.assertEqual(p.record("toggle"), "#TPUD2wREC0A54")
        self.assertEqual(p.fc_attitude_enable(True), "#TPUG2wFAE0139")
        self.assertEqual(p.fc_attitude(-50, 0, 0), "#TPUGCwFAIEC780000000064")

    def test_parse_roundtrip_and_errors(self):
        f = p.parse(p.gimbal_angle(-50, 5, 20))
        self.assertEqual((f["src"], f["dst"], f["ctrl"], f["ident"]), ("U", "G", "w", "GAM"))
        self.assertEqual(len(f["data"]), 12)
        with self.assertRaises(ValueError):
            p.parse("#TPUG2wPTZ006B")          # bad checksum
        with self.assertRaises(ValueError):
            p.parse("hello")
        with self.assertRaises(ValueError):
            p.parse("#TPUG3wPTZ006A")          # length mismatch (checksum aside)

    def test_speed_encoding_and_clamp(self):
        # +3.0 deg/s -> 6 -> "06"; -3.0 -> -6 -> 0xFA; clamp 1000 deg/s -> 127 -> 7F
        self.assertEqual(p.parse(p.gimbal_speed(3.0, -3.0))["data"], "06FA")
        self.assertEqual(p.parse(p.gimbal_speed(1000, -1000))["data"], "7F81")

    def test_track_point_bounds(self):
        with self.assertRaises(ValueError):
            p.track_point(1281, 10)
        with self.assertRaises(ValueError):
            p.track_point(10, -1)


class WithFake(unittest.TestCase):
    PORT = 15999

    @classmethod
    def setUpClass(cls):
        cls.cam = FakeC12()
        ready = threading.Event()
        cls.th = threading.Thread(target=cls.cam.serve, args=("127.0.0.1", cls.PORT, ready), daemon=True)
        cls.th.start()
        ready.wait(2)

    @classmethod
    def tearDownClass(cls):
        cls.cam.stop()
        cls.th.join(2)

    def setUp(self):
        self.c = p.C12Client("127.0.0.1", self.PORT, timeout=0.5)
        self.c.request(p.ptz("center"))

    def tearDown(self):
        self.c.close()

    def test_control_is_echoed_with_swapped_addresses(self):
        r = self.c.request(p.ptz("up"))
        self.assertEqual((r["src"], r["dst"], r["ident"], r["data"]), ("G", "U", "PTZ", "01"))
        self.assertEqual(self.cam.pitch, 5.0)

    def test_queries(self):
        self.assertEqual(self.c.request(p.query("VER"))["data"], "V1.0.0")
        self.assertEqual(self.c.request(p.query("MOD"))["data"], "C12")

    def test_angle_command_and_limits(self):
        self.c.request(p.gimbal_angle(45, -30, 20))
        self.assertAlmostEqual(self.cam.yaw, 45.0)
        self.assertAlmostEqual(self.cam.pitch, -30.0)
        self.c.request(p.gimbal_angle(90, 90, 20))      # pitch above +10 must clamp
        self.assertAlmostEqual(self.cam.pitch, 10.0)

    def test_speed_moves_gimbal_and_stop_works(self):
        self.c.request(p.gimbal_speed(20.0, 0.0))
        time.sleep(0.5)
        self.c.request(p.ptz("stop"))
        moved = self.cam.yaw
        self.assertTrue(5.0 < moved < 14.0, moved)       # ~10 deg expected
        time.sleep(0.2)
        self.assertAlmostEqual(self.cam.yaw, moved, places=3)   # stayed stopped

    def test_tracking_flow_and_invalid_point(self):
        self.assertIsNotNone(self.c.request(p.track_point(640, 360)))
        self.c.request(p.track_confirm())
        self.assertTrue(self.cam.tracking)
        self.assertEqual(self.cam.target, (640, 360))
        self.c.request(p.track_stop())
        self.assertFalse(self.cam.tracking)

    def test_confirm_without_point_does_not_track(self):
        self.c.request(p.track_stop())
        self.c.request(p.track_confirm())
        self.assertFalse(self.cam.tracking)

    def test_attitude_push(self):
        self.c.request(p.gimbal_angle(-50, -10, 20))
        self.c.send(p.attitude_push(20))
        got = None
        for _ in range(10):
            fr = self.c.recv_push(1.0)
            if fr and "GAC" in fr:
                got = p.parse_attitude_push(fr)
                break
        self.c.request(p.attitude_push(0))
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got[0], -50.0, places=1)
        self.assertAlmostEqual(got[1], -10.0, places=1)

    def test_garbage_gets_error_frame(self):
        self.c.sock.sendto(b"garbage", self.c.addr)
        data, _ = self.c.sock.recvfrom(256)
        self.assertEqual(data.decode(), "#TPMU2wERE!!30")


if __name__ == "__main__":
    unittest.main(verbosity=2)
