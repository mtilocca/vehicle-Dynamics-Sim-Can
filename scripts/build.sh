#!/usr/bin/env bash
set -e

# -------- Configuration --------
BUILD_DIR="build"
BUILD_TYPE="${1:-Debug}"   # Pass "Release" as first arg, defaults to Debug
# --------------------------------

echo "==> Building XCMG XDE320 plant-sensor-can-sim"
echo "    Build type: ${BUILD_TYPE}"
echo "    Build dir : ${BUILD_DIR}"
echo

# Create build directory if missing
mkdir -p "${BUILD_DIR}"

# Configure
CMAKE_BIN="${CMAKE_BIN:-/opt/homebrew/bin/cmake}"

echo "==> Configuring with CMake (${CMAKE_BIN})"
"${CMAKE_BIN}" -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

# Build
echo "==> Building"
"${CMAKE_BIN}" --build "${BUILD_DIR}" -j "$(nproc 2>/dev/null || sysctl -n hw.logicalcpu)"

echo
echo "==> Build complete"
