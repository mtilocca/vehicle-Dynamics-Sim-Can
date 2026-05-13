# ST67W61 Wi-Fi HAT — Hardware Handover

Snapshot of the physical setup, pin assignments, and the single most
important hardware finding from the bring-up session: the X-NUCLEO-67W61M1
ships with a **locked NCP** that needs a one-time UART firmware flash
before SPI communication can work.

## Boards and parts

| Item | P/N | Role |
|---|---|---|
| Host MCU board | NUCLEO-H753ZI (STM32H753ZI, M7 @ 480 MHz) | Runs the Zephyr application + WiFi driver. Has on-board ST-Link/V3E for flash/debug + USART3 VCP. |
| Wi-Fi / BLE expansion | X-NUCLEO-67W61M1 (MB2230) | Plugs onto the H753ZI's Arduino headers. Carries the ST67W611M coprocessor (NCP). |
| Programming probe | On-board ST-Link/V3E + pyocd | Used for flashing/debug of the H753ZI. |
| Dev host | Raspberry Pi 4B (aarch64) | Will be used for the one-time NCP UART flash via `qemu-user-static`. |
| Power | USB-C to NUCLEO ST-Link port | Drives both H753ZI and HAT via Arduino 5V/3V3 lines on CN6. |

## Pin map — Arduino headers, HAT signals, H753ZI MCU pins

These are the connections we have verified in this session. They match the
official `x-cube-st67w61` reference for other supported boards.

| HAT signal | Arduino pin | NUCLEO-H753ZI MCU pin | DTS polarity used | Status |
|---|---|---|---|---|
| SPI1 SCK   | D13 | PA5  | — | OK |
| SPI1 MISO  | D12 | PA6  | — | OK |
| SPI1 MOSI  | D11 | PB5  | — | OK |
| SPI1 CS    | D10 | PD14 | overridden to `GPIO_ACTIVE_HIGH` (board default was `_LOW` — wrong for ST67W61) | OK |
| CHIP_EN    | D5  | PE11 | `GPIO_ACTIVE_LOW` (logical-active = physical LOW = module off) | OK |
| BOOT       | D6  | PE9  | `GPIO_ACTIVE_HIGH` (`0` = AT mode, `1` = firmware-update mode) | OK |
| SPI_RDY    | D3  | PE13 | `GPIO_ACTIVE_HIGH \| GPIO_PULL_DOWN` | OK |
| NCP UART TX (NCP → host) | D0 | PG9  | not used by H753ZI app | only relevant during the one-time UART flash from a Pi/PC |
| NCP UART RX (host → NCP) | D1 | PG14 | not used by H753ZI app | as above |

Pin mapping is in `zephyr/app.overlay` (search for `&spi1` and the
`st67w61_dev:` child node).

## Verified hardware behaviour

The following have been **electrically proven** during debug:

- `CHIP_EN` and `BOOT` GPIOs are wired through to the NCP and the NCP samples them.
  - `BOOT=LOW` → after `CHIP_EN` rises, NCP asserts RDY at ~640 ms and clocks out an 8-byte frame on MISO.
  - `BOOT=HIGH` → NCP is completely silent (no RDY, MISO stays at pull-down level). This is the "firmware-update mode" handshake — it waits for UART traffic from a host.
- SPI1 is electrically OK: byte 1 of the NCP's first frame consistently reads `0x55`, proving MISO is being driven by the NCP and the clock phase is correct (Mode 0, MSB-first).
- MISO is pulled low on the board — confirmed by the `BASELINE (module OFF)` diagnostic which shows `00 00 00 00 00 00 00 00` when CHIP_EN is held off.
- No MOSI→MISO short or coupling: varying the MOSI dummy byte from `0xAA` to `0xFF` does not change MISO readout.

## The single critical finding

The boot frame the NCP emits with `BOOT=LOW` is **fixed** across power
cycles and SPI configurations:

```
01 55 00 00 00 00 01 00
```

This is *not* a sampling artefact. The official ST `x-cube-st67w61`
upstream confirms two things that explain it:

1. The NCP runs a **Qualcomm-derived bootloader** when no runtime firmware
   is programmed. From `Projects/ST67W6X_Scripts/Binaries/README.md`:
   > *"The delivered binaries are only usable with a locked NCP."*
2. The runtime SPI protocol expects the `spi_header` struct with magic
   `0x55AA` (LE → wire bytes `AA 55`) and a full-duplex header exchange.
   The `01 55 …` we see is the bootloader's permanent handshake, not the
   runtime protocol.

**Conclusion**: until the NCP is flashed once with the runtime firmware,
no host-side SPI driver change will make AT commands work. The hardware
is fine; the silicon is empty.

## What you need to do, physically

1. **Free 5 jumper wires (F-F)** — you have these.
2. **Confirm the Pi 4B is on the same bench** — it will act as the UART bridge and the QConn_Flash host (via `qemu-user-static`).
3. **Follow [NCP_FLASH_PROCEDURE.md](NCP_FLASH_PROCEDURE.md)** end-to-end. It covers:
   - One-time Pi config (`dtoverlay=disable-bt`, apt installs)
   - Exact wiring table (Pi GPIO ↔ HAT Arduino pins)
   - Running `scripts/ncp_flash_pi.sh`
   - Restoring the H753ZI to normal-mode firmware after the flash

After step 3, the SPI bus should — for the first time in this project —
return a real `AA 55 …` header from the NCP. That unlocks the entire
driver-integration work described in [NEXT_STEPS.md](NEXT_STEPS.md).

## What you do **not** need

- ❌ A NUCLEO-U575ZI-Q (the upstream's "supported" host MCU). The Pi
  replaces that role entirely.
- ❌ A USB-UART adapter. The Pi's PL011 on GPIO 14/15 runs at 2 Mbaud and
  is already 3.3 V TTL.
- ❌ A Windows machine. `qemu-user-static` on the Pi runs the x86_64
  `QConn_Flash_Cmd-ubuntu` binary fine.
- ❌ Any reverse-engineering of the boot protocol. ST + Qualcomm tools
  do it for us.

## Hardware safety notes

- The HAT is 3.3 V logic everywhere. **Do not** connect Pi 5 V to any HAT
  signal line. 5 V to the HAT's CN6 5 V pin is fine for power (the HAT
  has its own LDO), but we leave power supplied by the H753ZI via the
  Arduino header — no Pi 5 V wire is needed at all.
- During the NCP flash the H753ZI is reflashed with `CONFIG_WIFI=n` so it
  doesn't drive PE9/PE11 while the Pi is asserting BOOT/CHIP_EN. This
  avoids GPIO contention. Detailed sequence in `NCP_FLASH_PROCEDURE.md`.
- The on-HAT MISO line is **not** driven when the NCP is tri-stated. There
  is a pull-down to GND somewhere (board confirmed: MISO reads 0 with
  module off). So a missing wire shows up as all-zero reads, not floating
  garbage — useful diagnostic.
