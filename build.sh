#!/usr/bin/env bash
set -e

# -------- Configuration --------
BUILD_DIR="build"
CMAKE_GENERATOR=""   # leave empty to use default
BUILD_TYPE="Debug"   # Debug or Release
# --------------------------------

echo "==> Building plant-sensor-can-sim"
echo "    Build type: ${BUILD_TYPE}"
echo "    Build dir : ${BUILD_DIR}"
echo

# Create build directory if missing
mkdir -p "${BUILD_DIR}"

# Configure (only re-runs if CMakeCache.txt missing)
if [ ! -f "${BUILD_DIR}/CMakeCache.txt" ]; then
    echo "==> Configuring with CMake"
    cmake -S . -B "${BUILD_DIR}" \
          -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
          ${CMAKE_GENERATOR}
else
    echo "==> CMake already configured"
fi

# Build
echo "==> Building"
cmake --build "${BUILD_DIR}" -j "$(nproc)"

echo
echo "==> Build complete"
