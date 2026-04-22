#!/usr/bin/env bash
# scripts/ota_flash.sh
# Build Zephyr firmware and OTA-flash it to the HDV-Sim board over HTTPS.
#
# Usage:
#   bash scripts/ota_flash.sh
#
# Environment overrides:
#   HDV_HOST=192.168.1.80   HDV_TOKEN=<hex>   bash scripts/ota_flash.sh

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ZEPHYR_ROOT="/home/baloo/zephyrproject"

# ── Step 1: Build ──────────────────────────────────────────────────────────────
echo "====================================================="
echo "  HDV-Sim Build + OTA"
echo "====================================================="
echo ""
echo "[1/2] Building firmware…"
cd "$ZEPHYR_ROOT"
.venv/bin/west build -b nucleo_h753zi \
    "$REPO_ROOT/zephyr" \
    --sysbuild

# ── Step 2: OTA flash ─────────────────────────────────────────────────────────
echo ""
echo "[2/2] OTA flash…"
python3 "$REPO_ROOT/scripts/ota_py.py"
