#!/bin/bash
# Build a NEWER OpenCV (+ contrib trackers) on a Jetson Nano into a PRIVATE folder. Stage by stage; nothing runs without --yes.
#   ./jetson_build_opencv.sh preflight            read-only checks (disk, RAM, swap, compiler, cmake, temperature)
#   ./jetson_build_opencv.sh fetch      --yes     git clone opencv + opencv_contrib (network, ~0.4 GB)
#   ./jetson_build_opencv.sh configure  --yes     cmake only (fast; shows whether Python bindings configure)
#   ./jetson_build_opencv.sh build      --yes     make -j2 under nice, with a temperature watchdog (2-3 h expected)
#   ./jetson_build_opencv.sh install    --yes     make install into $PREFIX (NOT /usr, no sudo)
#   ./jetson_build_opencv.sh probe                compile + run tracker_probe.cpp against the new build
#   ./jetson_build_opencv.sh cleanup    --yes     delete ONLY the build tree + sources in $BUILD_ROOT (keeps the install in $PREFIX) to get the disk space back
# It never touches the system OpenCV (/usr), never uses sudo, and never changes any system setting.
# Remove everything again with:  rm -rf "$BUILD_ROOT" "$PREFIX"
set -u
STAGE="${1:-help}"; YES="${2:-}"
OPENCV_VERSION="${OPENCV_VERSION:-4.9.0}"            # 4.9.0: first release with TrackerVit; TrackerNano since 4.7
BUILD_ROOT="${BUILD_ROOT:-$HOME/opencv_build}"
PREFIX="${PREFIX:-$HOME/opencv-$OPENCV_VERSION-install}"
JOBS="${JOBS:-2}"                                     # 4 GB RAM: keep low
MAX_TEMP_C="${MAX_TEMP_C:-80}"; RESUME_TEMP_C="${RESUME_TEMP_C:-68}"
MIN_FREE_KB="${MIN_FREE_KB:-1500000}"                 # abort the build (cleanly) if free disk drops below ~1.5 GB
HERE="$(cd "$(dirname "$0")" && pwd)"
need_yes() { [ "$YES" = "--yes" ] || { echo "[dry-run] stage '$STAGE' would run as described above; re-run with --yes to execute."; exit 0; }; }
temp_c() { for z in /sys/class/thermal/thermal_zone*; do case "$(cat $z/type 2>/dev/null)" in *CPU*|*cpu*) echo $(( $(cat $z/temp)/1000 )); return;; esac; done; echo 0; }

case "$STAGE" in
preflight)
  echo "== disk (need >= 8 GB free where $BUILD_ROOT and $PREFIX live)"; df -h "$HOME" | tail -1
  FREE_KB=$(df -k "$HOME" | tail -1 | awk '{print $4}'); [ "$FREE_KB" -ge 8000000 ] && echo "   OK" || echo "   TOO LITTLE FREE SPACE - free some space first"
  echo "== memory / swap (need swap >= 4 GB; this script does not create swap)"; free -h | sed -n 1,3p
  echo "== compiler / cmake"; gcc --version | head -1; g++ --version | head -1; cmake --version | head -1
  echo "== python (only needed for Python bindings)"; python3 --version; python3 -c "import numpy; print('numpy', numpy.__version__)" 2>&1 | tail -1
  echo "== system OpenCV (will NOT be touched)"; pkg-config --modversion opencv4 2>/dev/null || pkg-config --modversion opencv 2>/dev/null || echo "   none via pkg-config"
  echo "== GStreamer dev (for WITH_GSTREAMER)"; pkg-config --modversion gstreamer-1.0 gstreamer-app-1.0 2>&1 | tr '\n' ' '; echo
  echo "== network to GitHub (read-only)"; git ls-remote --heads https://github.com/opencv/opencv.git 4.x 2>&1 | head -1 | cut -c1-60 || true
  echo "== temperature now: $(temp_c) C"; echo "== existing build dirs:"; ls -d "$BUILD_ROOT" "$PREFIX" 2>/dev/null || echo "   none"
  ;;
fetch)
  echo "Will: git clone --depth 1 --branch $OPENCV_VERSION opencv and opencv_contrib into $BUILD_ROOT (network, about 0.4 GB)"; need_yes
  mkdir -p "$BUILD_ROOT" && cd "$BUILD_ROOT" || exit 1
  [ -d opencv ] || git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv.git
  [ -d opencv_contrib ] || git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv_contrib.git
  git -C opencv log --oneline -1; git -C opencv_contrib log --oneline -1
  ;;
configure)
  echo "Will: cmake configure (no compiling) in $BUILD_ROOT/build with a reduced module list, no CUDA, install prefix $PREFIX"; need_yes
  cd "$BUILD_ROOT" && mkdir -p build && cd build || exit 1
  PYOPT="-DBUILD_opencv_python3=OFF -DBUILD_opencv_python2=OFF"
  [ "${WITH_PYTHON:-0}" = "1" ] && PYOPT="-DBUILD_opencv_python3=ON -DBUILD_opencv_python2=OFF -DPYTHON3_EXECUTABLE=$(command -v python3)"
  cmake ../opencv -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DOPENCV_EXTRA_MODULES_PATH="$BUILD_ROOT/opencv_contrib/modules" \
    -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui,video,dnn,tracking,python3 \
    -DWITH_CUDA=OFF -DWITH_GSTREAMER=ON -DWITH_FFMPEG=ON -DWITH_GTK=ON -DWITH_OPENCL=OFF \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_apps=OFF \
    -DOPENCV_GENERATE_PKGCONFIG=ON -DCMAKE_INSTALL_RPATH="$PREFIX/lib" $PYOPT 2>&1 | tee ../configure.log | tail -40
  echo "---- check these lines in $BUILD_ROOT/configure.log: 'GStreamer: YES', 'GTK+: YES', 'To be built:' (must list tracking, video, dnn)"
  grep -E "GStreamer:|GTK\+:|FFMPEG:|To be built:|Python 3:|Interpreter" ../configure.log | head -12
  ;;
build)
  echo "Will: make -j$JOBS under nice, pausing if CPU > ${MAX_TEMP_C} C until < ${RESUME_TEMP_C} C. Expected 2-3 hours. Log: $BUILD_ROOT/build.log"; need_yes
  cd "$BUILD_ROOT/build" || exit 1
  nice -n 10 make -j"$JOBS" > ../build.log 2>&1 &
  MP=$!
  while kill -0 "$MP" 2>/dev/null; do
    FREE=$(df -k "$BUILD_ROOT" | tail -1 | awk '{print $4}')
    if [ "$FREE" -lt "$MIN_FREE_KB" ]; then echo "[watchdog] free disk ${FREE} KB < ${MIN_FREE_KB} KB: stopping make (nothing deleted; free space and re-run 'build' to resume)"; pkill -TERM -P "$MP" 2>/dev/null; kill -TERM "$MP" 2>/dev/null; wait "$MP" 2>/dev/null; exit 2; fi
    T=$(temp_c)
    if [ "$T" -ge "$MAX_TEMP_C" ]; then echo "[watchdog] $T C >= $MAX_TEMP_C C: pausing make"; pkill -STOP -P "$MP" 2>/dev/null; kill -STOP "$MP" 2>/dev/null
       while [ "$(temp_c)" -gt "$RESUME_TEMP_C" ]; do sleep 20; done; echo "[watchdog] cooled to $(temp_c) C: resuming"; pkill -CONT -P "$MP" 2>/dev/null; kill -CONT "$MP" 2>/dev/null; fi
    sleep 20
  done
  wait "$MP"; RC=$?; echo "make exit code: $RC"; tail -5 ../build.log; exit $RC
  ;;
install)
  echo "Will: make install into $PREFIX (no sudo, nothing under /usr)"; need_yes
  cd "$BUILD_ROOT/build" && make install 2>&1 | tail -5; ls "$PREFIX/lib" | head
  ;;
probe)
  echo "Will: compile $HERE/tracker_probe.cpp against $PREFIX and run it"
  export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"; export LD_LIBRARY_PATH="$PREFIX/lib:${LD_LIBRARY_PATH:-}"
  g++ -std=c++14 "$HERE/tracker_probe.cpp" -o "$BUILD_ROOT/tracker_probe" $(pkg-config --cflags --libs opencv4) && "$BUILD_ROOT/tracker_probe"
  ;;
cleanup)
  echo "Will delete ONLY: $BUILD_ROOT  (sources + build tree). The install in $PREFIX is kept. Size now: $(du -sh "$BUILD_ROOT" 2>/dev/null | cut -f1)"; need_yes
  [ -x "$PREFIX/lib" ] || [ -d "$PREFIX/lib" ] || { echo "no install found in $PREFIX - refusing to delete the build tree"; exit 1; }
  rm -rf "$BUILD_ROOT"; df -h "$HOME" | tail -1
  ;;
*) sed -n 2,13p "$0";;
esac
