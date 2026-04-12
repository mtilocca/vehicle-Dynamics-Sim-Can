#!/usr/bin/env bash
# scripts/ota_flash.sh
# Upload a signed Zephyr firmware image to the HDV-Sim board via HTTPS OTA.
#
# Usage:
#   bash scripts/ota_flash.sh [path/to/zephyr.signed.bin]
#
# Defaults:
#   BIN  — /home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.bin
#   HOST — 192.168.1.80
#   TOKEN — read from zephyr/src/http/http_auth.hpp (auto-generated)
#
# Environment overrides:
#   HDV_HOST=192.168.1.80   HDV_TOKEN=<hex>   bash scripts/ota_flash.sh

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEFAULT_BIN="/home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.bin"
AUTH_HPP="$REPO_ROOT/zephyr/src/http/http_auth.hpp"

BIN="${1:-$DEFAULT_BIN}"
HOST="${HDV_HOST:-192.168.1.80}"
BASE_URL="https://${HOST}"

# ── Resolve token ─────────────────────────────────────────────────────────────
if [[ -n "${HDV_TOKEN:-}" ]]; then
    TOKEN="$HDV_TOKEN"
elif [[ -f "$AUTH_HPP" ]]; then
    TOKEN=$(grep -oP '(?<=HDV_API_TOKEN\[\] = ")[^"]+' "$AUTH_HPP")
else
    echo "[error] Cannot find token: $AUTH_HPP not found."
    echo "        Run scripts/gen_http_token.sh or set HDV_TOKEN=<hex>."
    exit 1
fi

if [[ -z "$TOKEN" ]]; then
    echo "[error] Token is empty — check $AUTH_HPP"
    exit 1
fi

# ── Validate binary ───────────────────────────────────────────────────────────
if [[ ! -f "$BIN" ]]; then
    echo "[error] Firmware binary not found: $BIN"
    echo "        Build first:"
    echo "          cd /home/baloo/zephyrproject"
    echo "          .venv/bin/west build -b nucleo_h753zi \\"
    echo "            /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild"
    exit 1
fi

BIN_SIZE=$(stat -c%s "$BIN")
BIN_KB=$(( (BIN_SIZE + 1023) / 1024 ))

echo "====================================================="
echo "  HDV-Sim OTA Firmware Update"
echo "====================================================="
echo "  Target : $BASE_URL"
echo "  Binary : $BIN"
echo "  Size   : ${BIN_KB} KB (${BIN_SIZE} bytes)"
echo "  Token  : ${TOKEN:0:8}…"
echo "====================================================="

# ── Sanity check: board reachable ─────────────────────────────────────────────
echo ""
echo "[1/3] Checking board is reachable…"
if ! curl -sk --max-time 5 \
        -H "Authorization: Bearer $TOKEN" \
        "$BASE_URL/" -o /dev/null -w "%{http_code}" | grep -qE "^(200|302)"; then
    echo "[error] Board not responding at $BASE_URL"
    echo "        Check cable, IP, and that the board has booted."
    exit 1
fi
echo "      OK — board is up."

# ── Upload firmware ───────────────────────────────────────────────────────────
echo ""
echo "[2/3] Uploading firmware…"
echo "      (Flash write takes ~15 s — three 128 KB sector erases)"
echo ""

HTTP_RESPONSE=$(
    curl -sk --max-time 120 \
        -X POST "$BASE_URL/api/firmware" \
        -H "Authorization: Bearer $TOKEN" \
        -H "Content-Type: application/octet-stream" \
        -H "Content-Length: $BIN_SIZE" \
        --data-binary "@$BIN" \
        -w "\n%{http_code}"
)

HTTP_BODY=$(echo "$HTTP_RESPONSE" | head -n -1)
HTTP_CODE=$(echo "$HTTP_RESPONSE" | tail -n 1)

if [[ "$HTTP_CODE" == "200" ]]; then
    WRITTEN=$(echo "$HTTP_BODY" | grep -oP '(?<="bytes":)\d+' || echo "?")
    echo "      Upload OK — $WRITTEN bytes written to slot1."
elif [[ "$HTTP_CODE" == "401" ]]; then
    echo "[error] Authentication failed (HTTP 401)."
    echo "        Token mismatch — did you rebuild firmware after rotating credentials?"
    echo "        Token used: ${TOKEN:0:8}…"
    exit 1
elif [[ "$HTTP_CODE" == "503" ]]; then
    echo "[error] Another upload is already in progress (HTTP 503)."
    exit 1
else
    echo "[error] Upload failed (HTTP $HTTP_CODE): $HTTP_BODY"
    exit 1
fi

# ── Trigger reboot ────────────────────────────────────────────────────────────
echo ""
echo "[3/3] Triggering reboot…"

curl -sk --max-time 5 \
    -X POST "$BASE_URL/api/reboot" \
    -H "Authorization: Bearer $TOKEN" \
    -o /dev/null || true   # board resets — connection drop is expected

echo "      Reboot command sent."
echo ""
echo "====================================================="
echo "  MCUboot will swap slot1 → slot0 on next boot."
echo "  Board will be back in ~10 s."
echo ""
echo "  Verify:"
echo "    openssl s_client -connect ${HOST}:443 -tls1_2 2>&1 | grep Cipher"
echo "    picocom -b 115200 /dev/ttyACM0"
echo "    (look for: BOOT: image already confirmed)"
echo "====================================================="

# ── Wait for board to come back ───────────────────────────────────────────────
echo ""
echo "  Waiting for board to come back online…"
for i in $(seq 1 20); do
    sleep 1
    printf "  [%2ds] " "$i"
    if curl -sk --max-time 2 \
            -H "Authorization: Bearer $TOKEN" \
            "$BASE_URL/" -o /dev/null 2>/dev/null; then
        echo "board is back up!"
        echo ""
        echo "  OTA complete."
        exit 0
    else
        printf "waiting…\r"
    fi
done

echo ""
echo "  Board did not respond within 20 s."
echo "  Check UART: picocom -b 115200 /dev/ttyACM0"
exit 1
