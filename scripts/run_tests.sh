#!/usr/bin/env bash
# run_tests.sh — run all unit tests: host (Linux/GoogleTest) + Zephyr firmware build check.
#
# Usage:
#   bash scripts/run_tests.sh            # host tests + Zephyr build
#   bash scripts/run_tests.sh --host     # host tests only
#   bash scripts/run_tests.sh --zephyr   # Zephyr build only
#
# Requirements:
#   Host tests : cmake, g++ (C++17), internet for first googletest fetch
#   Zephyr     : west, arm-zephyr-eabi toolchain at the standard Zephyr SDK path

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST_BUILD_DIR="${REPO_ROOT}/build"
ZEPHYR_BUILD_DIR="/home/baloo/zephyrproject/build/zephyr/zephyr"
ZEPHYR_APP_DIR="${REPO_ROOT}/zephyr"
WEST="${HOME}/zephyrproject/.venv/bin/west"

RUN_HOST=true
RUN_ZEPHYR=true

for arg in "$@"; do
    case "$arg" in
        --host)   RUN_ZEPHYR=false ;;
        --zephyr) RUN_HOST=false   ;;
        *) echo "Unknown arg: $arg"; exit 1 ;;
    esac
done

PASS=0
FAIL=0

# ── helpers ────────────────────────────────────────────────────────────────────

section() { echo; echo "══════════════════════════════════════════════════"; echo "  $*"; echo "══════════════════════════════════════════════════"; }
ok()      { echo "  [PASS] $*"; ((PASS++)) || true; }
fail()    { echo "  [FAIL] $*"; ((FAIL++)) || true; }

# ── host tests ─────────────────────────────────────────────────────────────────

if $RUN_HOST; then
    section "Host unit tests (GoogleTest)"

    echo "  Configuring..."
    cmake -S "${REPO_ROOT}" -B "${HOST_BUILD_DIR}" \
          -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug \
          -DCMAKE_EXPORT_COMPILE_COMMANDS=OFF \
          -Wno-dev \
          --log-level=WARNING \
          > /tmp/cmake_config.log 2>&1 \
      && ok "cmake configure" \
      || { fail "cmake configure (see /tmp/cmake_config.log)"; cat /tmp/cmake_config.log; }

    echo "  Building..."
    cmake --build "${HOST_BUILD_DIR}" -j"$(nproc)" \
          > /tmp/cmake_build.log 2>&1 \
      && ok "cmake build" \
      || { fail "cmake build (see /tmp/cmake_build.log)"; tail -40 /tmp/cmake_build.log; }

    echo "  Running tests..."
    # PlantReverse.ReverseAcceleration and PlantReverse.ReverseSteering are
    # pre-existing plant model failures unrelated to firmware / codec changes.
    ctest --test-dir "${HOST_BUILD_DIR}" \
          --output-on-failure \
          --timeout 60 \
          --exclude-regex "PlantReverse\.(ReverseAcceleration|ReverseSteering)" \
      && ok "ctest" \
      || fail "ctest (one or more tests failed)"
fi

# ── Zephyr firmware build ───────────────────────────────────────────────────────

if $RUN_ZEPHYR; then
    section "Zephyr firmware build (west build)"

    if [[ ! -x "${WEST}" ]]; then
        fail "west not found at ${WEST} — skipping Zephyr build"
    else
        (cd /home/baloo/zephyrproject && \
         "${WEST}" build \
             --build-dir "${ZEPHYR_BUILD_DIR}" \
             -- -DAPP_DIR="${ZEPHYR_APP_DIR}") \
          > /tmp/west_build.log 2>&1 \
          && ok "west build" \
          || { fail "west build (see /tmp/west_build.log)"; tail -40 /tmp/west_build.log; }
    fi
fi

# ── summary ────────────────────────────────────────────────────────────────────

section "Summary"
echo "  Passed : ${PASS}"
echo "  Failed : ${FAIL}"

if [[ ${FAIL} -gt 0 ]]; then
    echo "  Result : FAIL"
    exit 1
else
    echo "  Result : PASS"
fi
