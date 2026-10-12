#!/usr/bin/env bash
# scripts/ota_flash.sh
# Build the web UI + Zephyr firmware and OTA-flash it to the HDV-Sim board over HTTPS.
#
# Usage:
#   bash scripts/ota_flash.sh            # build + upload
#   bash scripts/ota_flash.sh --build    # build only (no upload)
#
# Environment overrides:
#   HDV_HOST=192.168.1.80   HDV_TOKEN=<hex>
#   ZEPHYR_WS=~/zephyr-3.7  (west workspace pinned to Zephyr v3.7.0, see zephyr/west.yml)

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ZEPHYR_WS="${ZEPHYR_WS:-$HOME/zephyr-3.7}"
VENV="${ZEPHYR_VENV:-$HOME/zephyr-3.7-venv}"
BUILD_DIR="$ZEPHYR_WS/build"
BIN="$BUILD_DIR/zephyr/zephyr/zephyr.signed.bin"

# Zephyr 3.7 only accepts SDK 0.16.x; reuse the ARM compiler from the installed
# SDK via the cross-compile toolchain variant.
SDK="${ZEPHYR_SDK:-$HOME/zephyr-sdk-1.0.1}"
export ZEPHYR_BASE="$ZEPHYR_WS/zephyr"
export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
export CROSS_COMPILE="$SDK/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-"
unset ZEPHYR_SDK_INSTALL_DIR

echo "====================================================="
echo "  HDV-Sim Build + OTA"
echo "====================================================="

echo ""
echo "[1/3] Building web UI…"
(cd "$REPO_ROOT/webui" && { [ -d node_modules ] || npm ci; } && npm run build)

echo ""
echo "[2/3] Building firmware…"
# shellcheck disable=SC1091
source "$VENV/bin/activate"
cd "$ZEPHYR_WS"
west build -b nucleo_h753zi -d "$BUILD_DIR" "$REPO_ROOT/zephyr" --sysbuild
echo "Image: $(ls -l "$BIN" | awk '{print $5}') bytes"

if [ "${1:-}" = "--build" ]; then
    echo "Build only — skipping upload."
    exit 0
fi

echo ""
echo "[3/3] OTA flash…"
python3 "$REPO_ROOT/scripts/ota_py.py" "$BIN"
