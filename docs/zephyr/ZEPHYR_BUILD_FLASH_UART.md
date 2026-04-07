# Zephyr — Build, Flash & UART Reference

Target: **STM32H753ZI** (Nucleo-H753ZI) cross-compiled from **Raspberry Pi 5** (Ubuntu 24.04 arm64).

---

## Prerequisites

West workspace must already be initialised at `~/zephyrproject` and the Zephyr SDK installed at `~/zephyr-sdk-0.17.0`.
See `docs/ZEPHYR_RPI5_SETUP.md` for the one-time setup.

Activate the venv before any `west` command:

```bash
source ~/zephyrproject/.venv/bin/activate
```

---

## Build

```bash
cd ~/zephyrproject
west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr
```

**Clean build** (required when switching phases, board configs, or Kconfig):

```bash
rm -rf ~/zephyrproject/build
west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr
```

**Memory usage** is printed at the end of a successful build:

```
Memory region    Used Size   Region Size   %age Used
           FLASH:      99040 B         2 MB      4.72%
             RAM:      11072 B       512 KB      2.11%
```

---

## Flash

Connect the Nucleo board via **CN1 (ST-Link USB)** — the USB connector near the ethernet jack, labelled ST-LINK on the PCB. Do **not** use CN13 (user USB).

```bash
west flash
```

OpenOCD will print `read_memory` warnings during init — these are non-fatal quirks with OpenOCD 0.11 and STM32H7. Flash succeeds when you see:

```
wrote XXXXX bytes from file zephyr.hex in X.XXXs
shutdown command invoked
```

---

## Serial Monitor (UART Shell)

The board exposes a virtual COM port over the same ST-Link USB cable.

**Find the device:**

```bash
ls /dev/ttyACM*
# typically /dev/ttyACM0
```

**Connect:**

```bash
picocom -b 115200 /dev/ttyACM0
```

**Exit:** `Ctrl+A` then `Ctrl+X`

### If the port is busy

A previous session may still hold the device:

```bash
lsof /dev/ttyACM0        # find the PID
kill <PID>               # release it
# or
screen -wipe             # clean up all detached screen sessions
```

### If you are not in the `dialout` / `plugdev` group

```bash
sudo usermod -aG dialout $USER
# log out and back in, then reconnect
```

---

## Shell Commands

The Zephyr shell is available at the `uart:~$` prompt as soon as the board boots.

| Command | Description |
|---|---|
| `help` | List all registered commands |
| `plant state` | Dump all PlantState fields (v, x, y, yaw, SOC, wheel speeds, tire forces) |
| `plant mu <val>` | Set surface friction coefficient at runtime (0.1 – 1.0) |
| `plant reset` | Zero the plant state |
| `can stats` | TX / RX frame counts, timeout count, last RX timestamp |
| `can rx_frame` | Print last decoded ACTUATOR_CMD_1 fields |
| `vehicle info` | Heavy-Duty Electric Vehicle parameter summary + live surface mu |
| `kernel version` | Zephyr kernel version |
| `kernel threads` | All running threads — priority, state, stack usage |
| `kernel stacks` | Peak stack usage per thread (useful for tuning stack sizes) |
| `device list` | All registered Zephyr devices and their status |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Error: open failed` in west flash | Wrong USB cable (power-only) or wrong port (use CN1, not CN13) |
| ST-Link not in `lsusb` | Swap cable; try direct RPi USB port instead of hub |
| `[screen is terminating]` immediately | Port busy — run `lsof /dev/ttyACM0` and kill the holder |
| `FATAL: cannot open /dev/ttyACM0: Device or resource busy` | Same as above |
| Double log lines in terminal | Normal — log backend and shell backend both on USART3 |
| `west: command not found` | Activate the venv: `source ~/zephyrproject/.venv/bin/activate` |
| `cmake: command not found` | `sudo apt install cmake` |
| `No module named 'elftools'` | `pip install pyelftools` |

---

## Quick Reference

```bash
# One-liner: clean build + flash
source ~/zephyrproject/.venv/bin/activate && \
  cd ~/zephyrproject && \
  rm -rf build && \
  west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr && \
  west flash

# Attach serial monitor
picocom -b 115200 /dev/ttyACM0
# Exit: Ctrl+A  Ctrl+X
```
