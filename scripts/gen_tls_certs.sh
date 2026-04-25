#!/usr/bin/env bash
# scripts/gen_tls_certs.sh
# Generates a self-signed RSA-2048 TLS certificate for the HDV-Sim HTTPS server.
#
#   CN  : hdv-sim
#   SAN : IP:192.168.1.80  (avoids browser "NET::ERR_CERT_COMMON_NAME_INVALID")
#   Validity : 10 years
#
# Output (both .gitignored — never commit):
#   zephyr/certs/server.crt   ← embed in firmware via generate_inc_file_for_target()
#   zephyr/certs/server.key   ← embedded alongside cert
#
# Usage:
#   bash scripts/gen_tls_certs.sh
#
# Re-run any time you want to rotate the TLS certificate (also re-run gen_http_token.sh
# so the Bearer token is refreshed at the same time, then rebuild and reflash firmware).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CERT_DIR="$REPO_ROOT/zephyr/certs"
CRT="$CERT_DIR/server.crt"
KEY="$CERT_DIR/server.key"

mkdir -p "$CERT_DIR"

# Static IP of the board — must match CONFIG_NET_CONFIG_MY_IPV4_ADDR in prj.conf.
BOARD_IP="192.168.1.80"

openssl req -x509 -newkey rsa:2048 -days 3650 -nodes \
    -keyout "$KEY" \
    -out    "$CRT" \
    -subj   "/CN=hdv-sim/O=HDV/C=AU" \
    -addext "subjectAltName=IP:${BOARD_IP}" \
    2>/dev/null

echo "[cert]   Written to : $CRT"
echo "[key]    Written to : $KEY"
echo ""
echo "Cert details:"
openssl x509 -in "$CRT" -noout -subject -issuer -dates -fingerprint -sha256 2>/dev/null
echo ""
echo "SAN entries:"
openssl x509 -in "$CRT" -noout -ext subjectAltName 2>/dev/null
echo ""
echo "Rebuild and reflash firmware to deploy:"
echo "  cd ~/zephyrproject"
echo "  rm -rf build"
echo "  .venv/bin/west build -b nucleo_h753zi \$HOME/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild"
echo "  .venv/bin/west flash"
