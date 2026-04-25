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

All Zephyr work is under `zephyr/`. Uses West + sysbuild (MCUboot + app in one shot).
Board: NUCLEO-H753ZI, connected via ST-Link USB.

### Build

```bash
cd /home/baloo/zephyrproject

# Full rebuild — both MCUboot and app (use after first clone or after mcuboot.conf changes)
rm -rf build
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr \
  --sysbuild --pristine

# Incremental rebuild — app only (MCUboot Kconfig unchanged)
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr \
  --sysbuild
```

### Flash (pyocd — use instead of `west flash`)

`west flash` wraps OpenOCD 0.11 which has a dual-bank STM32H7 bug. Use pyocd directly.

```bash
# Normal flow — app only (MCUboot already in flash at 0x08000000)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  /home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.hex

# Full recovery — MCUboot first, then app (after chip erase or new board)
pyocd flash --target stm32h743xx --connect attach --erase sector \
  /home/baloo/zephyrproject/build/mcuboot/zephyr/zephyr.hex

pyocd flash --target stm32h743xx --connect attach --erase sector \
  /home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.hex
```

> **Note:** A USB I/O error printed *after* the write completes is benign — the board
> resets and drops the USB connection. The write has already succeeded.
> If the error occurs *during* the write, change USB cable and port (Raspberry Pi USB
> hub contention is the most common cause).

### UART monitor

```bash
picocom -b 115200 /dev/ttyACM0
# Ctrl-A Ctrl-X to exit
```

### HTTPS verification

```bash
# Quick TLS check (self-signed cert — skip verification)
openssl s_client -connect 192.168.1.80:443 -tls1_2
# Should show: New, TLSv1.2, Cipher is ECDHE-RSA-AES128-GCM-SHA256

# With our cert (no browser warning)
curl --cacert zephyr/certs/server.crt https://192.168.1.80/

# Skip cert check
curl -k https://192.168.1.80/
```

### Signing key note

Sysbuild uses `root-rsa-2048.pem` (MCUboot's public test key) by default.
`MCUBOOT_SIGNING_KEY_FILE` in `CMakeLists.txt` is ignored by sysbuild.
Before enabling RDP Level 1, generate a project-specific RSA-2048 key and set
`SB_CONFIG_BOOT_SIGNATURE_KEY_FILE` in `zephyr/sysbuild.conf`.
See [docs/mcuboot_signing_and_flash_recovery.md](docs/mcuboot_signing_and_flash_recovery.md).

## Analysis / plotting

```bash
python3 scripts/plotter_analysis.py
```

Individual plot scripts are in `tools/analytics/`.
