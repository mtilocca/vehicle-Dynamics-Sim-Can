#!/usr/bin/env bash
# scripts/gen_mqtt_broker_cert.sh
# Generates a self-signed RSA-2048 TLS certificate for the MQTT broker.
#
# The STM32 acts as a TLS client connecting to an external broker. This cert
# is loaded into the firmware as the trusted CA (HDV_TLS_CA_TAG=2) and must
# also be used by the broker itself (Mosquitto certfile/keyfile).
#
#   CN  : hdv-mqtt-broker
#   SAN : IP:<broker-ip>   — must match the broker's IP that the STM32 dials
#   Validity : 10 years
#
# Usage:
#   bash scripts/gen_mqtt_broker_cert.sh [broker-ip]
#   bash scripts/gen_mqtt_broker_cert.sh 192.168.1.106   # MacBook default
#
# Output (both .gitignored — never commit):
#   zephyr/certs/broker_ca.crt   ← embedded in firmware via generate_inc_file_for_target()
#   zephyr/certs/broker_ca.key   ← copied to broker host for Mosquitto
#
# After running this script:
#   1. Rebuild + flash firmware:   bash scripts/ota_flash.sh
#   2. Copy cert+key to the broker host (MacBook):
#        scp zephyr/certs/broker_ca.crt  <user>@<mac>:~/
#        scp zephyr/certs/broker_ca.key  <user>@<mac>:~/
#   3. Configure Mosquitto (see docs/zephyr/MQTT.md for full config).
#
# Re-run any time the broker IP changes, then rebuild firmware and recopy to broker.

set -euo pipefail

BROKER_IP="${1:-192.168.1.106}"

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CERT_DIR="$REPO_ROOT/zephyr/certs"
CRT="$CERT_DIR/broker_ca.crt"
KEY="$CERT_DIR/broker_ca.key"

mkdir -p "$CERT_DIR"

openssl req -x509 -newkey rsa:2048 -days 3650 -nodes \
    -keyout "$KEY" \
    -out    "$CRT" \
    -subj   "/CN=hdv-mqtt-broker/O=HDV/C=AU" \
    -addext "subjectAltName=IP:${BROKER_IP}" \
    2>/dev/null

chmod 600 "$KEY"

echo "[cert]   Written to : $CRT"
echo "[key]    Written to : $KEY"
echo ""
echo "Cert details:"
openssl x509 -in "$CRT" -noout -subject -issuer -dates -fingerprint -sha256 2>/dev/null
echo ""
echo "SAN entries:"
openssl x509 -in "$CRT" -noout -ext subjectAltName 2>/dev/null
echo ""
echo "Next steps:"
echo "  1. Rebuild + flash:    bash scripts/ota_flash.sh"
echo "  2. Copy to broker:"
echo "       scp $CRT  <user>@${BROKER_IP}:~/"
echo "       scp $KEY  <user>@${BROKER_IP}:~/"
echo "  3. Configure Mosquitto — see docs/zephyr/MQTT.md"
