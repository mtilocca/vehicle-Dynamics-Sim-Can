# ST67W611M NCP Firmware Flash — Raspberry Pi 4B procedure

The X-NUCLEO-67W61M1 ships with a **locked NCP**. The chip emits a fixed
8-byte bootloader handshake (`01 55 00 00 00 00 01 00`) and ignores SPI
AT commands until runtime firmware has been written to it once.
This is independent of any host driver work — until the NCP is flashed,
no Zephyr SPI driver change can unstick communication.

This document covers the one-time flash using the Raspberry Pi 4B as
both the UART bridge and the QConn_Flash host (via `qemu-user-static`).

## Hardware checklist

| Item | Notes |
|---|---|
| Raspberry Pi 4B | Already in lab. PL011 UART (GPIO 14/15) supports the required 2 Mbaud. |
| NUCLEO-H753ZI + X-NUCLEO-67W61M1 HAT | Already in lab. H753ZI provides 5V/3V3 power to the HAT through the Arduino headers; its firmware is rebuilt with `CONFIG_WIFI=n` for the duration of the flash so it doesn't fight the Pi on PE9/PE11. |
| 5 jumper wires (F-F) | Pi GPIO header → Arduino-header pins on the HAT or H753ZI. |
| x-cube-st67w61 v1.3.0 cloned at `/tmp/x-cube-st67w61` | `git clone --depth 1 --branch main https://github.com/STMicroelectronics/x-cube-st67w61.git /tmp/x-cube-st67w61` |

## Wiring

| Pi 4B header | Pi BCM | Pi function | → | HAT Arduino pin | NCP signal | NUCLEO-H753ZI pin (passthrough) |
|---|---|---|---|---|---|---|
| pin 8 | GPIO 14 | TXD0 (PL011) | → | D1 | NCP UART **RX** | PG14 |
| pin 10 | GPIO 15 | RXD0 (PL011) | ← | D0 | NCP UART **TX** | PG9 |
| pin 11 | GPIO 17 | GPIO out | → | D6 | BOOT | PE9 |
| pin 13 | GPIO 27 | GPIO out | → | D5 | CHIP_EN | PE11 |
| pin 6 | GND | reference | — | any GND on HAT or CN6 | GND | GND |

Pick the wire endpoints on either the HAT top-side Arduino sockets or
the H753ZI morpho/Arduino headers — they are electrically the same net
once the HAT is plugged in. Do **not** connect Pi 5V; the HAT is powered
by the H753ZI through CN6.

## One-time Pi setup

```bash
# 1. Free the high-speed PL011 from Bluetooth and expose it on GPIO 14/15.
sudo tee -a /boot/firmware/config.txt <<'EOF'

# Free PL011 (ttyAMA0) for ST67W611M NCP flashing — see docs/wifi/NCP_FLASH_PROCEDURE.md
dtoverlay=disable-bt
EOF
sudo systemctl disable --now hciuart      # stop the BT firmware loader from grabbing UART
sudo reboot
# After reboot: /dev/ttyAMA0 should appear.

# 2. GPIO toolkit.
sudo apt update
sudo apt install -y libgpiod-bin

# 3. x86_64 emulation + glibc runtime (needed because QConn_Flash is x86_64-only).
sudo apt install -y qemu-user-static binfmt-support
sudo dpkg --add-architecture amd64
sudo apt update
sudo apt install -y libc6:amd64 libstdc++6:amd64 zlib1g:amd64

# 4. Verify everything is in place.
ls -la /dev/ttyAMA0                       # device should exist
gpiodetect                                 # lists gpiochipN
qemu-x86_64-static --version               # qemu-user works
file /tmp/x-cube-st67w61/Projects/ST67W6X_Scripts/Binaries/QConn_Flash/QConn_Flash_Cmd-ubuntu
qemu-x86_64-static /tmp/x-cube-st67w61/Projects/ST67W6X_Scripts/Binaries/QConn_Flash/QConn_Flash_Cmd-ubuntu --help | head
```

## Per-flash procedure

### 1. Put the H753ZI into "passive" mode

Disable the Zephyr WiFi driver so the H753ZI doesn't drive PE9/PE11
while the Pi is using them. The HAT-facing pins fall back to their
reset default (input, high-Z) which is what the Pi needs.

```bash
# In zephyr/prj.conf, comment out CONFIG_WIFI=y (or set =n) and rebuild.
# Rebuild + flash the H753ZI as usual:
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild --pristine
.venv/bin/pyocd flash --target stm32h743xx build/zephyr/zephyr/zephyr.signed.hex
.venv/bin/pyocd reset --target stm32h743xx
```

The H753ZI now powers the HAT but leaves its NCP-control pins alone.

### 2. Connect the 5 jumper wires per the wiring table above.

### 3. Run the Pi flash script

```bash
sudo /home/baloo/repos/vehicle-Dynamics-Sim-Can/scripts/ncp_flash_pi.sh
```

What this does:
1. Re-asserts the UART line settings on `/dev/ttyAMA0` at 2 Mbaud.
2. Drives `BOOT` HIGH on GPIO 17.
3. Pulses `CHIP_EN` low → high on GPIO 27 (NCP enters Qualcomm bootloader).
4. Invokes `QConn_Flash_Cmd-ubuntu` under `qemu-x86_64-static` with the
   `mission_t02` config (Wi-Fi runtime, LwIP-on-host — appropriate for
   Zephyr which has its own net stack).
5. On success, drops `BOOT` back LOW and pulses `CHIP_EN` so the NCP
   boots into its new runtime firmware.

Successful output ends with `QConn_Flash` reporting an exit code of 0 and
the partition log showing all blocks programmed.

### 4. Restore the H753ZI's normal firmware

```bash
# Re-enable CONFIG_WIFI=y in prj.conf
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild --pristine
.venv/bin/pyocd flash --target stm32h743xx build/zephyr/zephyr/zephyr.signed.hex
.venv/bin/pyocd reset --target stm32h743xx
```

Disconnect the 5 Pi jumper wires. The H753ZI's Zephyr driver should now
see a proper `AA 55 ...` boot frame from the NCP on the SPI bus.

## Sanity checks if QConn_Flash fails

| Symptom | Likely cause |
|---|---|
| `qemu: uncaught target signal 11` | Multiarch libc not installed; re-run `apt install libc6:amd64 zlib1g:amd64`. |
| `Open serial port failed` | `/dev/ttyAMA0` missing — `dtoverlay=disable-bt` not effective; check config.txt and reboot. |
| `Sync timeout` / no bootloader response | BOOT not HIGH when CHIP_EN rose. Verify with `gpioget gpiochip0 17 27` — both must read 1 once the script's gpioset ran. |
| Garbage on RX | UART pins reversed. Pi TX (BCM 14) → HAT D1 (NCP RX); Pi RX (BCM 15) → HAT D0 (NCP TX). |
| Crash mid-flash | Loose wires, or H753ZI still driving PE9/PE11. Re-confirm WiFi is disabled in `prj.conf` and the H753ZI is running that build. |

## Why `mission_t02` and not `t01`

- `t01` runs LwIP inside the NCP. Useful if the host doesn't have a TCP/IP stack.
- `t02` runs LwIP on the host. Zephyr already has its own net stack, so
  we want to bypass the NCP's LwIP and let Zephyr do the IP work.

Either works for our planned driver integration, but `t02` is closer
to the architecture we're targeting and avoids a redundant stack on the
NCP that we'd never use.
