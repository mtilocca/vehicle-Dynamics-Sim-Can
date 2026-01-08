#!/usr/bin/env bash
# run_closed_loop_influx.sh - Simple closed-loop sim with InfluxDB

set -e

# Get inputs
read -p "Duration (seconds) [600]: " DURATION
DURATION=${DURATION:-600}

read -p "InfluxDB URL [http://localhost:8086]: " INFLUX_URL
INFLUX_URL=${INFLUX_URL:-http://localhost:8086}

# Token is REQUIRED
while true; do
    read -sp "InfluxDB Token (required): " INFLUX_TOKEN
    echo
    if [ -z "$INFLUX_TOKEN" ]; then
        echo "Error: Token is required. Get it with: influx auth list"
        continue
    fi
    
    # Test the token
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
        -H "Authorization: Token $INFLUX_TOKEN" \
        "$INFLUX_URL/api/v2/buckets?org=Autonomy")
    
    if [ "$HTTP_CODE" = "200" ]; then
        echo "✓ Token validated"
        break
    else
        echo "✗ Token validation failed (HTTP $HTTP_CODE)"
        echo "Try again or press Ctrl+C to exit"
    fi
done

read -p "Organization [Autonomy]: " INFLUX_ORG
INFLUX_ORG=${INFLUX_ORG:-Autonomy}

read -p "Bucket [vehicle-sim]: " INFLUX_BUCKET
INFLUX_BUCKET=${INFLUX_BUCKET:-vehicle-sim}

read -p "Write interval in ms [500]: " INFLUX_INTERVAL
INFLUX_INTERVAL=${INFLUX_INTERVAL:-500}

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
  --influx-token $INFLUX_TOKEN \
  --influx-org $INFLUX_ORG \
  --influx-bucket $INFLUX_BUCKET \
  --influx-interval $INFLUX_INTERVAL"

# Run
eval $CMD