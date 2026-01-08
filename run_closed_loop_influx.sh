#!/usr/bin/env bash
# run_closed_loop_influx.sh - Simple closed-loop sim with InfluxDB

set -e

# Get inputs
read -p "Duration (seconds) [600]: " DURATION
DURATION=${DURATION:-600}

read -p "InfluxDB URL [http://localhost:8086]: " INFLUX_URL
INFLUX_URL=${INFLUX_URL:-http://localhost:8086}

read -sp "InfluxDB Token (empty for no auth): " INFLUX_TOKEN
echo

read -p "Organization [Autonomy]: " INFLUX_ORG
INFLUX_ORG=${INFLUX_ORG:-Autonomy}

read -p "Bucket [vehicle-sim]: " INFLUX_BUCKET
INFLUX_BUCKET=${INFLUX_BUCKET:-vehicle-sim}

echo ""
echo "Starting simulation (duration: ${DURATION}s)..."
echo "Remember to start your controller in another terminal!"
echo ""

# Build command
CMD="./build/src/sim/sim_main \
  --can-rx \
  --real-time \
  --duration $DURATION \
  --vehicle config/vehicles/heavy_truck.yaml \
  --influx \
  --influx-url $INFLUX_URL \
  --influx-org $INFLUX_ORG \
  --influx-bucket $INFLUX_BUCKET"

# Add token if provided
if [ -n "$INFLUX_TOKEN" ]; then
    CMD="$CMD --influx-token $INFLUX_TOKEN"
fi

# Run
eval $CMD