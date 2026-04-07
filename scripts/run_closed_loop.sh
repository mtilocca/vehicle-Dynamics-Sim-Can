#!/usr/bin/env bash
# run_closed_loop.sh - Closed-loop simulation via CAN

set -e

read -rp "Duration (seconds) [600]: " DURATION
DURATION=${DURATION:-600}

echo ""
echo "Starting simulation (duration: ${DURATION}s)..."
echo "Remember to start your controller in another terminal!"
echo ""

exec ./build/src/sim/sim_main \
    --can-rx \
    --real-time \
    --duration "$DURATION" \
    --vehicle config/vehicles/heavy_truck.yaml
