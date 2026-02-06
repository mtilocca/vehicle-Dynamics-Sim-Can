#!/usr/bin/env bash
# run_closed_loop_influx.sh - Simple closed-loop sim with InfluxDB

set -e

# Get inputs
read -p "Duration (seconds) [600]: " DURATION
DURATION=${DURATION:-600}


echo ""
echo "Starting simulation (duration: ${DURATION}s)..."
echo "Remember to start your controller in another terminal!"
echo ""

# Build command
CMD="./build/src/sim/sim_main \
  --can-rx \
  --real-time \
  --dynamic-model \
  --duration $DURATION \
  --vehicle config/vehicles/heavy_truck.yaml \
  "

# Run
eval $CMD