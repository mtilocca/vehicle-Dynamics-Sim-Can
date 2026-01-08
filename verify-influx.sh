#!/usr/bin/env bash
# quick_verify.sh - Simple data verification using influx CLI

echo "=================================="
echo "InfluxDB Data Quick Check"
echo "=================================="
echo ""

# Get token
read -sp "Enter InfluxDB token: " TOKEN
echo ""
echo ""

BUCKET="vehicle-sim"
ORG="Autonomy"

echo "[1/3] Checking measurements in bucket..."
INFLUX_TOKEN=$TOKEN influx query "
import \"influxdata/influxdb/schema\"
schema.measurements(bucket: \"$BUCKET\")
" --org $ORG

echo ""
echo "[2/3] Counting vehicle_truth data points..."
INFLUX_TOKEN=$TOKEN influx query "
from(bucket: \"$BUCKET\")
  |> range(start: 0)
  |> filter(fn: (r) => r._measurement == \"vehicle_truth\")
  |> filter(fn: (r) => r._field == \"x_m\")
  |> count()
" --org $ORG

echo ""
echo "[3/3] Showing last 10 data points..."
INFLUX_TOKEN=$TOKEN influx query "
from(bucket: \"$BUCKET\")
  |> range(start: 0)
  |> filter(fn: (r) => r._measurement == \"vehicle_truth\")
  |> filter(fn: (r) => r._field == \"x_m\" or r._field == \"y_m\" or r._field == \"v_mps\")
  |> pivot(rowKey:[\"_time\"], columnKey: [\"_field\"], valueColumn: \"_value\")
  |> sort(columns: [\"_time\"], desc: true)
  |> limit(n: 10)
" --org $ORG --raw

echo ""
echo "=================================="
echo "✓ Done!"
echo "=================================="