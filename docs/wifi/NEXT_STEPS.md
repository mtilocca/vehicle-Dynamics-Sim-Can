# ST67W61 — Next Steps

Ordered roadmap. Each phase has a clear gate: don't start the next phase
until the previous one's gate is green.

## Phase 0 — Flash the NCP firmware (BLOCKING)

**Why first**: until this is done, no driver work helps. The HAT is
physically OK but its NCP has no runtime firmware.

**How**: follow [NCP_FLASH_PROCEDURE.md](NCP_FLASH_PROCEDURE.md) end-to-end.
TL;DR:

1. Pi: `dtoverlay=disable-bt` + reboot + `apt install` the listed packages.
2. H753ZI: set `CONFIG_WIFI=n` in `zephyr/prj.conf`, rebuild + flash.
3. Wire the 5 jumpers (Pi GPIO ↔ HAT Arduino pins per the doc).
4. `sudo /home/baloo/repos/vehicle-Dynamics-Sim-Can/scripts/ncp_flash_pi.sh`
5. Disconnect jumpers, set `CONFIG_WIFI=y` back, rebuild + flash H753ZI.

**Gate**: at the next H753ZI boot, the `RX[0..7]` log line must show
`aa 55 ??  ??  00 00 00 00` — i.e. the magic bytes are now `AA 55`, with
a non-zero length field at bytes [2:3] and probably `0x00` in the flags
byte. The current `01 55 00 00 00 00 01 00` becoming anything-with-AA-55
is the green light.

If the gate is still red after a successful `QConn_Flash` run, suspect
the flashing didn't actually program (look at `QConn_Flash` exit code +
console output for "all blocks programmed"), or the H753ZI got reflashed
with a stale build. Re-run the procedure.

## Phase 1 — Decide: vendor the upstream driver or hand-write the framing?

The user has indicated a preference for **vendoring the ST upstream driver**
(`x-cube-st67w61`) over reinventing the protocol state machine. That's
the assumed path below, but here are both options for completeness.

### Option A (recommended) — vendor `x-cube-st67w61` middleware
Pros: ST maintains it; future NCP firmware updates "just work"; protocol
correctness is not our problem; the public API (`W6X_WiFi_Connect` etc.)
is stable.
Cons: depends on FreeRTOS primitives → need a small Zephyr shim. Brings
~5-10k LOC of vendored C into the tree.

### Option B — hand-write framing in our existing Zephyr driver
Pros: no shim, no vendored code, no FreeRTOS surface, full control.
Cons: re-implement a Qualcomm protocol state machine. Higher bug surface
during bring-up. We become responsible for protocol-level changes
across NCP firmware releases.

The phases below describe Option A. If you pick B instead, swap "vendor
upstream + write shim" for "reimplement spi_iface.c semantics in
`st67w61_spi.c`" — but the GPIO/SPI plumbing stays.

## Phase 2 — Vendor the upstream sources (Option A)

Snapshot the relevant subset of `STMicroelectronics/x-cube-st67w61`
@ `572b72dcb0c5bf505c088536e8196bdb5011ba12` (V1.3.0) into:

```
zephyr/modules/st67w61/upstream/
├── Api/             (W6X_* public headers + sources used by app)
├── Core/            (state machine, blocking helpers)
├── Driver/W61_at/   (AT command encoders/decoders)
├── Driver/W61_bus/  (spi_iface.{c,h}, spi_port.h)  ← spi_port.c is OURS
├── Conf/            (template w61_config.h)
└── Utils/           (logging/misc helpers actually referenced)
```

Skip BLE / Thread / mfg / project examples. Wi-Fi STA only for first
integration. Record the upstream SHA in `upstream/UPSTREAM_VERSION.md`.

License: BSD-3-Clause (see upstream `LICENSE.md`). Free to vendor.

**Gate**: `west build` succeeds with the vendored sources opt-in via a
new `CONFIG_ST67W61_USE_UPSTREAM` Kconfig (default `n` until we're ready).

## Phase 3 — FreeRTOS → Zephyr compatibility shim

The upstream code uses FreeRTOS APIs. Add a new file
`zephyr/modules/st67w61/freertos_shim/{freertos.h,freertos.c}` that
provides ~12 wrappers, all backed by Zephyr primitives:

| FreeRTOS API | Zephyr backing |
|---|---|
| `xEventGroupCreate / SetBits / WaitBits / ClearBits` | `k_event` |
| `xQueueCreate / Send / Receive` | `k_msgq` |
| `xSemaphoreCreateBinary / Take / Give` | `k_sem` |
| `xSemaphoreCreateMutex / Take / Give` | `k_mutex` |
| `xTaskCreate / vTaskDelete` | `k_thread_create` |
| `vTaskDelay`, `pdMS_TO_TICKS`, `portMAX_DELAY` | `k_msleep`, identity, `K_FOREVER` |

Expected size: ~200 LOC of mostly one-liners. The upstream code stays
unmodified — just `#include "freertos.h"` resolves to our shim.

**Gate**: the vendored sources compile against the shim with no upstream
patches. Linker-time, not runtime, gate.

## Phase 4 — `spi_port.c` (Zephyr-side HAL)

Implement the `spi_port_*` contract expected by `spi_iface.c`:

| Upstream call | Zephyr implementation |
|---|---|
| `spi_port_init(cb)` | Verify `spi_dt_spec`, register completion callback. Reuse `dat->spi_no_cs_cfg` cached config from current driver. |
| `spi_port_set_cs(int)` | `gpio_pin_set_raw(cs->port, cs->pin, n)` — keep `ACTIVE_HIGH` polarity proven in this session. |
| `spi_port_is_ready()` | `gpio_pin_get_dt(&cfg->rdy)`. |
| `spi_port_transfer(tx, rx, len)` | `spi_transceive(cfg->spi.bus, &spi_no_cs_cfg, …)`. |
| `spi_port_wait_for_rdy(state, timeout_ms)` | Existing semaphore + RDY ISR pattern; expose a "wait LOW" form too. |

This is essentially `st67w61_spi.c` minus all the `AA 55` framing
(which moves to upstream `spi_iface.c`). About 60% of our current code
gets stripped, 40% becomes the new `spi_port.c`.

**Gate**: from a Zephyr `main()`, calling `W6X_Init()` produces a
successful header exchange (the upstream code's debug log should show
the first `spi_header` round-trip with magic = `0x55AA` on both sides).

## Phase 5 — Wi-Fi management glue

Refit `zephyr/modules/st67w61/drivers/wifi/st67w61/st67w61_wifi.c` to use
the upstream API:

| Zephyr `wifi_mgmt_ops` callback | Call into upstream |
|---|---|
| `connect(req)` | `W6X_WiFi_Connect(&opts)` |
| `disconnect()` | `W6X_WiFi_Disconnect()` |
| `iface_status(out)` | `W6X_WiFi_Station_GetState(&state)` |
| MAC at iface init | `W6X_WiFi_Station_GetMACAddress(mac)` |
| Scan | `W6X_WiFi_Scan(cb)` |

The deferred-init `k_work` pattern stays — the only change is what runs
inside the worker:

```c
W6X_Init();                      // was: hw_reset + at_init
W6X_WiFi_Init();
W6X_WiFi_Station_Start();
W6X_WiFi_Station_GetMACAddress(dat->mac);
net_if_set_link_addr(...);
dat->hw_ready = true;
```

**Gate**: `wifi connect <ssid> <psk>` from the Zephyr shell joins an AP,
gets a DHCP lease, and a `wifi status` shows `IP=<addr>`.

## Phase 6 — Build/Kconfig wiring

- Add a top-level CMake for the vendored sources at
  `zephyr/modules/st67w61/upstream/CMakeLists.txt`.
- Add `CONFIG_ST67W61_USE_UPSTREAM` Kconfig (default `y` once Phase 5 passes).
- Gate the old `st67w61_at.c` + framing code on `!CONFIG_ST67W61_USE_UPSTREAM`
  so we can switch back if needed (fallback during bring-up).
- Set ST middleware compile-time flags: `W61_USE_WIFI=1`, `W61_USE_BLE=0`,
  `W61_USE_THREAD=0`.

**Gate**: full `west build` with `CONFIG_ST67W61_USE_UPSTREAM=y` succeeds
and the resulting firmware connects to Wi-Fi.

## Phase 7 — Cleanup

Once Phase 5 gate is green and Phase 6 is stable:

- Delete `st67w61_at.c` (replaced by upstream `W61_at/`).
- Strip the `AA 55 / pad-0x88` framing logic from `st67w61_spi.c`
  (whatever isn't already `spi_port.c`).
- Remove debug Kconfig options (see [SOFTWARE_STATE.md](SOFTWARE_STATE.md) "Keep after NCP flash?" column).
- Reduce `CONFIG_ST67W61_LOG_LEVEL` from 4 (DBG) to 3 (INF).
- Mark [ST67W61_SPI_DEBUG_STATUS.md](ST67W61_SPI_DEBUG_STATUS.md) as
  superseded or delete it — these three handover docs cover everything.

## Out-of-scope for now

These are explicitly **deferred** until basic STA Wi-Fi works end-to-end:

- BLE host integration (`W6X_BLE_*`)
- Thread / Matter
- Soft-AP mode (`W6X_WiFi_AP_*`)
- TWT / power-save
- mbedTLS hand-off to the NCP
- LittleFS on NCP credentials store
- FOTA path for the NCP (we have `W6X_FWU_*` available, but defer)

The H753ZI already has its own HTTPS server + MQTT-TLS + Plant simulator
on top of the existing networking. None of that depends on the WiFi
driver, so the existing OTA/HTTPS/CAN workloads stay green throughout
the WiFi bring-up.

## Quick-reference: useful commands during integration

```bash
# Re-clone upstream after Pi reboot:
git clone --depth 1 --branch main \
    https://github.com/STMicroelectronics/x-cube-st67w61.git /tmp/x-cube-st67w61

# Verify upstream SHA is the one we tested:
git -C /tmp/x-cube-st67w61 log -1 --format=%H
# Expected: 572b72dcb0c5bf505c088536e8196bdb5011ba12

# Build + flash H753ZI (full pipeline)
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild --pristine
.venv/bin/pyocd flash --target stm32h743xx build/zephyr/zephyr/zephyr.signed.hex
.venv/bin/pyocd reset --target stm32h743xx

# Capture boot log (the one-liner we ended the session on)
timeout 25 cat /dev/ttyACM0 > /tmp/boot.log 2>/dev/null

# Filter ST67W61 lines from a captured log
grep -a -E "st67w61|Boot drain|RX\[|BASELINE|HW reset|BOOT pin|AT init|AT<|AT>|SPI cfg" /tmp/boot.log | sed 's/\x1b\[[0-9;]*m//g' | sort -u
```
