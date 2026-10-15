#!/bin/bash
# Every test level, in order (stops at the first failing level). See TESTING.md.
#   ./run_all_tests.sh            # 1-3: build + unit tests + offline benchmarks (no simulator needed)
#   ./run_all_tests.sh --sim      # + 4: the simulator scenarios (needs sim_up.sh running, the drone airborne, NO other tracker)
#   ./run_all_tests.sh --sim --long   # + the 10-minute endurance run
set -e
cd "$(dirname "$0")"
SIM=0; LONG=0
for a in "$@"; do case "$a" in --sim) SIM=1;; --long) LONG=1;; esac; done
echo "== 1. build (warnings are errors, as in CI)"
cmake -S tracker -B tracker/build-test -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Werror > /dev/null
cmake --build tracker/build-test -j"$(nproc)" 2>&1 | grep -E "error|warning" && exit 1 || true
echo "== 2. unit tests (control, FSM, geometry, appearance, crossing/occlusion with the real CSRT, GO pursuit, ...)"
tracker/build-test/target_tracker --selftest | tail -1 | grep -q " 0 failed" && echo "   selftest: OK" || { tracker/build-test/target_tracker --selftest | grep FAIL; exit 1; }
for f in simulation/*.py tracker/*.py tracker/tests/*.py; do python3 -m py_compile "$f"; done; echo "   python scripts compile: OK"
echo "== 3. offline benchmarks"
if ls ~/sim_run/*/click_*_frame_*.png >/dev/null 2>&1; then python3 tracker/tests/lock_bench.py ~/sim_run; else echo "   (no saved clicks: skipped lock_bench)"; fi
for v in ~/sim_run/recordings/*.avi; do [ -f "$v" ] && { echo "   $v"; python3 tracker/tests/track_bench.py "$v" | tail -1; }; done
if [ "$SIM" = 1 ]; then
  echo "== 4. simulator scenarios"
  T="lock reject relock mode linkloss vfreeze rescue abortfollow losthover qgc goto gostop large occlusion people"
  if [ "$LONG" = 1 ]; then python3 simulation/verify_suite.py $T --sustained 10; else python3 simulation/verify_suite.py $T; fi
fi
echo "== done"
