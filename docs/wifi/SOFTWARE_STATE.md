# ST67W61 Driver — Software State

Snapshot of the in-tree Zephyr driver as of session end.

## High-level: what works, what doesn't

### What works ✅
- **DTS binding + GPIO/SPI plumbing**. The `st,st67w61` device-tree node, pinctrl, CS-polarity override, RDY pull-down, BOOT/CHIP_EN polarity, RDY rising-edge interrupt. All electrically proven (see [HARDWARE_HANDOVER.md](HARDWARE_HANDOVER.md)).
- **Driver scaffolding compiles cleanly and is integrated with `wifi_mgmt`**. The Zephyr WiFi subsystem sees an interface; the driver-init order is correct; the deferred-init `k_work` pattern correctly defers slow probe work out of `SYS_INIT`.
- **Diagnostic infrastructure**. Log macros, 32-byte RX dumps (`RX[0..7]`, `RX[8..15]`, `RX[16..23]`, `RX[24..31]`), the `BASELINE (module OFF)` MISO-pull-down test, the boot-drain loop with timeout, `AT init` logging.
- **Build pipeline**. `west build --sysbuild` + `pyocd flash` + UART log capture all work; this is the iteration loop we used all session.

### What does **not** work ❌
- **AT command communication**. The NCP never responds to `AT\r\n` or `AT+GMR\r\n`. Root cause is **not** the driver: the NCP has no runtime firmware. See [HARDWARE_HANDOVER.md](HARDWARE_HANDOVER.md) for the smoking-gun trace and [NEXT_STEPS.md](NEXT_STEPS.md) for the fix path.
- **The framing protocol itself is wrong for this firmware family**. We implemented the `AA 55 / payload / 0x88-pad` framing from the older `stm32-hotspot/ST67W61-Bare-metal-implementation` repo. The current ST67W611M NCP uses the Qualcomm-derived `spi_header` framing shipped in the official `STMicroelectronics/x-cube-st67w61` middleware (v1.3.0). Even with a flashed NCP, our framing layer will need replacing — see [NEXT_STEPS.md](NEXT_STEPS.md).

## File-by-file state

All paths relative to repo root.

### `zephyr/modules/st67w61/Kconfig`
Top-level Kconfig for the driver. **Many new options added this session, mostly diagnostic.** Keep, delete, or promote each based on phase plan:

| Option | Purpose | Keep after NCP flash? |
|---|---|---|
| `ST67W61_SSID` / `_PASSWORD` | Pre-baked credentials (currently empty). | Keep (or move to per-board) |
| `ST67W61_SPI_MAX_HZ` | UI for max clock — currently ignored, DTS wins. | Remove or actually wire up |
| `ST67W61_AT_TIMEOUT_MS` | Per-command timeout. | Keep |
| `ST67W61_LOG_LEVEL` | Driver log level. | Keep |
| `ST67W61_CS_SETUP_US` | µs busy-wait after CS HIGH before SPI starts. | **Remove**: H1 hypothesis disproved. |
| `ST67W61_SPI_CPOL` / `_CPHA` / `_LSB_FIRST` | Matrix-sweep flags for SPI mode. | **Remove**: Mode 0 / MSB confirmed correct. |
| `ST67W61_RX_DUMMY_BYTE` | Fill byte for MOSI during RX-only. | **Remove** (or keep at `0xFF`): proved MOSI does not affect MISO. |
| `ST67W61_BOOT_LEVEL_FOR_AT` | BOOT pin polarity (0=AT, 1=update). | **Keep**: useful for one-line entry to FW update mode. |
| `ST67W61_CHIP_EN_OFF_HOLD_MS` | Cold-cycle hold time. | Keep with default 500 ms |
| `ST67W61_FORCE_RX_ON_NORDY` | Force 32-byte RX even on RDY timeout. | **Remove** (debug only). |
| `ST67W61_AT_INIT_GMR_FALLBACK` | Probe `AT+GMR` if first AT fails. | **Remove** (debug only). |

### `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_spi.c`
SPI framing + boot drain + RDY handshake. **All in-tree framing is for the wrong (legacy `AA 55`) protocol.** Significant changes this session:

- `st67w61_spi_init()`: CS GPIO explicit-configure, BOOT level configurable, RDY rising-edge ISR + semaphore, runtime CPOL/CPHA/LSB override on the cached `spi_no_cs_cfg`.
- `st67w61_spi_hw_reset()`: BASELINE (module-OFF) diagnostic transfer, configurable CHIP_EN off-hold, boot-drain loop with 32-byte RX dumps and RDY-poll logic.
- `st67w61_spi_transact()`: two-phase RX (8-byte hdr + payload), full hex dump of first 32 bytes at INF, magic check (looks for `AA 55`, returns "bad magic" warning otherwise), force-RX fallback on RDY timeout.

Status: **everything except the GPIO/SPI plumbing and the diagnostic logs should be considered scratch work**. Will be replaced by either:
- the upstream `spi_iface.c` + a Zephyr `spi_port.c` shim, OR
- a manual reimplementation of the `spi_header` exchange.

See Approach decision in [NEXT_STEPS.md](NEXT_STEPS.md).

### `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_at.c`
AT command encode/decode. Currently sends `AT\r\n` framed with the legacy
`AA 55 / pad-0x88` envelope. Includes the `AT+GMR` fallback added for
diagnostics. Will be replaced when the framing changes.

### `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_wifi.c`
Zephyr `net_if` + `wifi_mgmt` glue. Drives the `hw_init_work` (deferred
reset + AT init + MAC read). Currently `hw_init_work` always fails at
`st67w61_at_init`. This file is **largely keepable** — when we wire to
the upstream `W6X_*` API, the same shape is appropriate (a `k_work` does
`W6X_Init` → `W6X_WiFi_Init` → `W6X_WiFi_Station_GetMACAddress` → mark
iface up). Only the function calls inside change.

### `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61.h`
Shared structs (`st67w61_config`, `st67w61_data`). Currently carries
`tx_buf` / `rx_buf` sized for the legacy framing. Will need adjustment
to match the upstream API's buffer sizes.

### `zephyr/app.overlay`
DTS overlay. **All hardware-side configuration is correct and stays.**
SPI1 freq is currently `DT_FREQ_M(8)` → ~7.5 MHz on this clock tree. The
reference design runs at 10 MHz; either is fine.

### `zephyr/prj.conf`
- `CONFIG_WIFI=y` enables the driver (set to `n` before NCP flash).
- `CONFIG_ST67W61_LOG_LEVEL=4` is currently DBG. Drop to 3 (INF) once
  protocol is healthy.
- Networking, MQTT, TLS, MCUboot, CAN config is unchanged from the
  pre-WiFi state — not touched by this session.

### New files this session
- [`docs/wifi/NCP_FLASH_PROCEDURE.md`](NCP_FLASH_PROCEDURE.md) — one-time NCP UART flash from a Pi 4B.
- [`scripts/ncp_flash_pi.sh`](../../scripts/ncp_flash_pi.sh) — driver script for the above.
- This file, [HARDWARE_HANDOVER.md](HARDWARE_HANDOVER.md), and [NEXT_STEPS.md](NEXT_STEPS.md).

### Stale: `docs/wifi/ST67W61_SPI_DEBUG_STATUS.md`
Predates the BOOT-pin and Qualcomm-protocol findings. Useful for historical
context but **do not use it as ground truth**. The three new handover docs
supersede it.

## Upstream reference (not yet vendored)

The official ST middleware is cloned at `/tmp/x-cube-st67w61` (will be lost
on Pi reboot — re-clone with `git clone --depth 1 --branch main
https://github.com/STMicroelectronics/x-cube-st67w61.git /tmp/x-cube-st67w61`).
Pinned SHA `572b72dcb0c5bf505c088536e8196bdb5011ba12` (release V1.3.0,
NCP runtime binary v2.0.106).

Key files when we vendor:

| Path under `Middlewares/ST/ST67W6X_Network_Driver/` | What it gives us |
|---|---|
| `Driver/W61_bus/spi_iface.{c,h}` | The Qualcomm-derived SPI state machine — header struct, queue, RX-stall, ACK semantics. |
| `Driver/W61_bus/spi_port.h` | The HAL contract we must implement in Zephyr terms (the existing `st67w61_spi.c` is ~80% of what we need to expose this contract). |
| `Driver/W61_at/` | AT command encoders/decoders. |
| `Api/` | Public `W6X_*` API the application calls. |
| `Core/` | State machine + blocking helpers. |
| `Conf/` | Build-time configuration template. |
| `LICENSE.md` | BSD-3-Clause — free to vendor and modify. |

The Qualcomm `QConn_Flash` binaries (`Projects/ST67W6X_Scripts/Binaries/QConn_Flash/`) are NOT vendored: they're shipped binary-only under a Qualcomm permissive license that allows redistribution with their products, but we don't redistribute — we just use them once on the Pi.

## Tested baseline build

Current code on `security-hardening` branch:
- Built with `west build -b nucleo_h753zi … --sysbuild`
- Flash usage: `~370 KB` slot0 of 640 KB (plenty of headroom)
- AXI SRAM usage: `~350 KB` of 512 KB
- All other workloads (HTTPS, MQTT, CAN, plant sim) build alongside the
  WiFi driver and do not regress when WiFi is disabled or fails.
