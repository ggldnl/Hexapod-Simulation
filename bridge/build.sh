#!/usr/bin/env bash
# Compile the firmware core into a shared library for the Python sim (ctypes).
# The core is SDK-free header-only, so this is the same g++ the host tests use.
set -e
cd "$(dirname "$0")"
FW_SRC="../Hexapod-Firmware/src"
# On Windows (MinGW g++ from Git Bash/MSYS2) the library would depend on the
# MinGW C++ runtime DLLs, and Python 3.8+ does not look for a DLL's dependencies
# on PATH, so ctypes fails to load it. Link that runtime in statically there.
EXTRA=()
case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) EXTRA=(-static-libstdc++ -static-libgcc -static) ;;
esac
g++ -std=c++17 -O2 -Wall -Wextra -shared -fPIC -I"$FW_SRC" \
    sim_bridge.cpp "${EXTRA[@]}" -o libhexapod_fw.so
echo "built $(pwd)/libhexapod_fw.so"
