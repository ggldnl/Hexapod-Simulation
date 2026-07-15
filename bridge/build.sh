#!/usr/bin/env bash
# Compile the firmware core into a shared library for the Python sim (ctypes).
# The core is SDK-free header-only, so this is the same g++ the host tests use.
set -e
cd "$(dirname "$0")"
FW_SRC="../Hexapod-Firmware/src"
g++ -std=c++17 -O2 -Wall -Wextra -shared -fPIC -I"$FW_SRC" \
    sim_bridge.cpp -o libhexapod_fw.so
echo "built $(pwd)/libhexapod_fw.so"
