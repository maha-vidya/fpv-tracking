#!/bin/bash
# Fetch the MAVLink C headers at the revision the project was built against.
set -e
cd "$(dirname "$0")/.."
DEST=third_party/mavlink
REV=a6abd1787f90ba38ec117dcfeeb511beab75b398   # c_library_v2, 2026-08-13
if [ ! -d "$DEST/.git" ]; then
    mkdir -p third_party
    git clone https://github.com/mavlink/c_library_v2.git "$DEST"
fi
git -C "$DEST" fetch -q origin
git -C "$DEST" checkout -q "$REV"
echo "[INFO] MAVLink headers at $REV"
