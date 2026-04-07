# Contributing

## Prerequisites

**Ubuntu / Debian:**
```bash
sudo apt-get install -y cmake pkg-config libyaml-cpp-dev liblua5.4-dev libcurl4-openssl-dev
```

**CAN interface (virtual, for testing):**
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Build (Linux host)

```bash
# Debug (default)
bash scripts/build.sh

# Release
bash scripts/build.sh Release
```

## Test

```bash
ctest --test-dir build --output-on-failure -j$(nproc)
```

## Run the simulation

**Open-loop (safe mode — no external controller needed):**
```bash
./build/src/sim/sim_main --real-time --duration 60 --vehicle config/vehicles/heavy_truck.yaml
```

**Closed-loop (CAN RX — requires a controller on vcan0):**
```bash
bash scripts/run_closed_loop.sh
```

**With InfluxDB telemetry:**
```bash
bash scripts/run_closed_loop_influx.sh
```

**Monitor CAN traffic:**
```bash
./build/src/can/vcan_listener vcan0 config/can_map.dbc
```

## Zephyr RTOS HIL (STM32H753ZI)

See the full build and flash instructions in the docs:
[docs/zephyr/ZEPHYR_BUILD_FLASH_UART.md](docs/zephyr/ZEPHYR_BUILD_FLASH_UART.md)

Quick reference:
```bash
cd ~/zephyrproject
west build -b nucleo_h753zi /path/to/vehicle-Dynamics-Sim-Can/zephyr
west flash
picocom -b 115200 /dev/ttyACM0
```

## Analysis / plotting

```bash
python3 scripts/plotter_analysis.py
```

Individual plot scripts are in `tools/analytics/`.
