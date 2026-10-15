#!/bin/bash
# Build target_tracker (run on the Jetson).
set -e
cd "$(dirname "$0")/.."
[ -f third_party/mavlink/ardupilotmega/mavlink.h ] || scripts/fetch_mavlink.sh
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j"$(nproc)"
echo "[INFO] Built: $(pwd)/target_tracker"
