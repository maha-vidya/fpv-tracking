# Skydroid C12 control (TOP protocol) + fake camera

Source documents: *Skydroid C12 User Manual* and *TOP Protocol EN.docx* v1.1.6 (UDP port 5000).
Nothing here touches `main.cpp` or any existing file. Python 3.6+.

| File | What |
|---|---|
| `c12_protocol.py` | Frame builder/parser, checksum, helpers for gimbal speed/angle, attitude push, GOT/SUM tracking, FAE/FAI, REC, queries, a small UDP client. `python3 c12_protocol.py <ip>` = read-only smoke test (VER, MOD, VID). |
| `fake_c12.py` | A pretend C12 on UDP (default 5000; use `--port` for another). Models gimbal limits, speed integration, attitude push, tracking flags. **Never reports tracking status** (the doc has no such message). |
| `test_c12.py` | 13 tests: every checksum example in the doc, builders vs doc frames, fake-camera round trips. `python3 test_c12.py` |

## Open items to confirm on the real C12 (the document contradicts itself)
1. **Angle unit** for GAY/GAP/GAM: text says 0.5 deg; the worked example (-50 deg = 0xEC78 = -5000) and the attitude push use 0.01 deg. Code uses 0.01 deg.
2. **Speed examples** for GSY/GSM mix numbers (-6 vs 0xE2). Code uses value = deg/s / 0.5 as a two's-complement byte. Start with tiny speeds and watch the direction.
3. **FAI** field list mentions a mode byte, the example has only yaw/pitch/roll. Code sends yaw/pitch/roll.
4. **Pitch range**: manual says -90..+10, TOP doc says +-90. The fake uses the manual's.
5. **Tracking feedback**: no message reports lost/target box. Unknown whether GOT then SUM(01) is the right order or whether SUM(01) alone follows a screen tap. Test with the G20 app first.
6. Whether the G20 app and our UDP client can both control the gimbal.

Safety: send only the read-only smoke test first. Do not send motion commands to the real gimbal until items 1-2 are confirmed with tiny values and the gimbal is free to move.
