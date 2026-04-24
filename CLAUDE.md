# CLAUDE.md — HDV Simulator codebase context

Zephyr RTOS firmware running on an STM32H753ZI (NUCLEO-H753ZI) that executes a
heavy-duty electric vehicle plant simulator and exposes it via an HTTPS dashboard,
OTA firmware update endpoint, and FDCAN1 CAN bus interface.

---

## Build & Flash

```bash
# Generate TLS cert + bearer token (once, or to rotate credentials)
bash scripts/gen_http_token.sh

# Build firmware + OTA flash in one command
bash scripts/ota_flash.sh
```

`ota_flash.sh` calls `west build` then `scripts/ota_py.py`. The Python OTA client
sends the firmware in 4096-byte TLS records to stay within mbedTLS `SSL_MAX_CONTENT_LEN`.

If the board is not yet running firmware (fresh MCU), flash via pyocd:
```bash
cd /home/baloo/zephyrproject
.venv/bin/pyocd flash --target stm32h743xx build/zephyr/zephyr/zephyr.signed.hex
.venv/bin/pyocd reset --target stm32h743xx
```

---

## Key Constants

| Item | Value |
|------|-------|
| Board IP | 192.168.1.80 |
| HTTPS port | 443 |
| CAN bus | FDCAN1 — loopback mode (`CONFIG_HDV_CAN_LOOPBACK=y`) |
| MCUboot slot0 | 0x08040000 (640 KB) |
| MCUboot slot1 | 0x08100000 (640 KB) |
| Build dir | `/home/baloo/zephyrproject/build/zephyr/zephyr/` |
| Token file | `zephyr/src/http/http_auth.hpp` (git-ignored, auto-generated) |
| Cert/key | `zephyr/certs/server.crt` / `server.key` (git-ignored) |

---

## Thread Map

| Thread | Priority | Stack | Role |
|--------|----------|-------|------|
| `watchdog_tid` | 2 | 512 B | Feeds IWDG every 500 ms; expects semaphore from plant |
| `stats_tid` | 3 | 1 KB | 1 Hz — reads heap stats into `g_sys_stats` |
| `plant_tid` | 5 | 8 KB | 10 ms deterministic plant loop (timer-driven) |
| `led_tid` | 12 | 576 B | Random LED patterns on LD1/LD2/LD3 |
| `http_tid` | 10 | 16 KB | HTTPS server — TLS 1.2, ECDHE-RSA-AES128-GCM-SHA256 |
| `can_rx_tid` | 6 | 1 KB | FDCAN1 RX → decodes ACTUATOR_CMD_1 → writes `g_cmd` |
| `stm_eth` (HAL) | 2 | 1.5 KB | Ethernet RX DMA handler |

---

## Shared Globals (defined in `zephyr/src/main.cpp`)

| Symbol | Type | Protected by |
|--------|------|-------------|
| `g_state` | `plant::PlantState` | `g_state_mutex` |
| `g_cmd` | `sim::ActuatorCmd` | `g_cmd_mutex` |
| `g_sys_stats` | `SysStats` | `g_stats_mutex` |
| `g_can_tx_count` | `atomic_t` | atomic (no mutex) |
| `g_can_rx_count` | `atomic_t` | atomic |
| `g_can_timeout_count` | `atomic_t` | atomic |
| `g_surface_mu` | `double` | none (shell write / plant read — benign) |

`SysStats` is declared in `zephyr/src/stats/sys_stats.hpp`.

---

## Known Quirks

**CAN loopback** — `CONFIG_HDV_CAN_LOOPBACK=y` in `prj.conf`. No physical transceiver
needed. Set `n` only when SN65HVD230 is wired to **CN11 morpho pins 55 (TX) / 57 (RX)**
with the RS pin pulled to GND. CN9 ZIO exposes the same PD0/PD1 signals.

**MCUboot signing** — Uses MCUboot's public test key (`root-rsa-2048.pem`). Signing key
is set via `MCUBOOT_SIGNING_KEY_FILE` in `zephyr/CMakeLists.txt`. Acceptable for
development; replace with a project-specific RSA-2048 key before production
(see `docs/zephyr/SECURITY_HARDENING.md`).

**OTA small-record workaround** — `scripts/ota_py.py` sends POST body in 4096-byte
chunks so each becomes one TLS record ≤ `SSL_MAX_CONTENT_LEN`. Standard curl sends
16 KB records by default, which mbedTLS accepts now that `SSL_MAX_CONTENT_LEN=16384`.
The Python script is still the default OTA tool; `ota_flash.sh` wraps it.

**Image auto-confirm** — `main.cpp` calls `boot_write_img_confirmed()` on first boot.
No manual confirmation step needed after OTA.

**Watchdog semaphore** — `plant_tid` gives `g_wdt_sem` every 10 ms step.
`watchdog_tid` waits up to 500 ms. If the plant thread stalls (e.g. CAN TX blocking),
the IWDG fires after ~10 s and resets the board.

---

## Memory Budget (at rest)

| Region | Used | Total | % |
|--------|------|-------|---|
| Flash (slot0) | ~370 KB | 512 KB | ~72% |
| AXI SRAM | ~350 KB | 512 KB | ~67% |
| mbedTLS heap | ~60 KB peak (handshake) | 96 KB pool | — |
| General heap | 128 KB pool | — | — |

---

## Credential Rotation

```bash
bash scripts/gen_http_token.sh   # new token + cert + key
bash scripts/ota_flash.sh        # rebuild + OTA flash
```

Both `http_auth.hpp` and `zephyr/certs/` are git-ignored. Never commit them.
