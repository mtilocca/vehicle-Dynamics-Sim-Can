# Session Handover — CAN Bring-Up & MQTT(S) Roadmap

*Generated after the CAN bring-up session. Use this as context for the next chat where
MQTT(S) control is implemented.*

---

## 1. What Was Accomplished This Session

### 1.1 OTA Flash Script (`scripts/ota_flash.sh`)

A full automation script was created and debugged. Key lessons:

**Bug: `Expect: 100-continue` killed every upload.**
curl silently adds `Expect: 100-continue` for POST bodies > 1 KB. The Zephyr HTTP server
reads headers (including `Expect`) then immediately calls `zsock_recv` for the body — but
curl is still waiting for `HTTP/1.1 100 Continue` that never comes. `zsock_recv` returned 0
(connection appeared closed). Fix: add `-H "Expect:"` to suppress the header entirely.

**Bug: manual `Content-Length` header was redundant.**
`--data-binary @file` already makes curl set the correct `Content-Length`. Specifying it
manually with `-H "Content-Length: $BIN_SIZE"` is harmless but confusing — removed.

**Bug: reachability timeout too short.**
The STM32 TLS handshake (mbedTLS RSA-2048, 480 MHz M7) takes 3–5 seconds. The original
5-second `--max-time` on the curl reachability check was not enough. Increased to 15 s.
Post-reboot polling window also increased from 20 s to 30 s, per-poll timeout from 2 s to 10 s.

**Correct firmware binary path** (note the double `zephyr/zephyr/`):
```
/home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.bin   ← OTA (binary)
/home/baloo/zephyrproject/build/zephyr/zephyr/zephyr.signed.hex   ← pyocd (hex)
```

### 1.2 Plant Thread Fixes

**Plant loop overruns (40 ms vs 10 ms budget):**
- Primary cause: `CONFIG_LOG_MODE_IMMEDIATE=y` was blocking the plant thread at UART speed
  (~6 ms per log message at 115200 baud). Removed from `prj.conf`.
- Secondary: `std::pow(load_ratio, 0.5)` in `TyreDugoff::compute_Cx/Cy` called 8×/step.
  `std::pow` uses soft log+exp (~2 µs). `std::sqrt` uses the Cortex-M7 VSQRT instruction
  (~30 ns). Replaced.
- Tertiary: unconditional `sqrt` inside force-limit verification block ran every step.
  Moved to `LOG_DBG` (compiled out at `LOG_DEFAULT_LEVEL=3`).

**Brake diagnostic logging removed** (`plant_thread.cpp`):
A `LOG_INF` block firing every 100 ms (every 10th step) when `brake_cmd_pct > 1.0` was
left over from debugging. Removed entirely — it was the primary source of periodic UART
noise in steady state.

### 1.3 Reverse Drive Bug (`DrivePlant` + `VehicleSubsystem`)

**Root cause (three-layer):**
1. The HTTP dashboard always sends `torque` as a positive integer (slider 0 → max).
   `drive_torque_cmd_nm` is therefore always ≥ 0.
2. `DrivePlant::step()` was not applying gear position to the drive torque sign, so
   rear-axle torques were always positive regardless of REVERSE gear.
3. `VehicleSubsystem::update_direction_latch()` used `sgn(cmd.drive_torque_cmd_nm)` to
   determine intent — always +1 since torque is always positive.

**Fix — `DrivePlant::step()` STEP 4:**
```cpp
int gear_dir = 1;
if (s.gear_position == sim::GearPosition::REVERSE) gear_dir = -1;
else if (s.gear_position == sim::GearPosition::NEUTRAL) gear_dir = 0;

s.tau_drive_rl_nm = wheel_tq_total * 0.5 * gear_dir;
s.tau_drive_rr_nm = wheel_tq_total * 0.5 * gear_dir;
```

**Fix — `VehicleSubsystem::update_direction_latch()`:**
```cpp
switch (cmd.gear_position) {
    case sim::GearPosition::FORWARD: dir_latch_ = +1; break;
    case sim::GearPosition::REVERSE: dir_latch_ = p_.allow_reverse ? -1 : +1; break;
    default: dir_latch_ = 0; break;
}
```

### 1.4 CAN Subsystem Changes

**Loopback disabled** (`prj.conf`):
```conf
CONFIG_HDV_CAN_LOOPBACK=n
```
Gated by `#ifdef CONFIG_HDV_CAN_LOOPBACK` in `can_rx.cpp` — clean switch.

**CAN TX timeout: `K_MSEC(5)` → `K_NO_WAIT`** (`can_tx.cpp`):

*Critical finding:* With loopback disabled and no transceiver ACK-ing frames, FDCAN enters
error-passive then BUS-OFF. In those states `can_send()` with `K_MSEC(5)` may block for the
full timeout per frame. With 13+ frames per `can_tx_send_all()` call:
`13 × 5 ms = 65 ms` of blocking per plant step. After the 10-second WDT grace period,
the watchdog semaphore may not be given fast enough → hardware reset → 10-second reset loop.

Symptom: ping RTT spikes to 800–900 ms (network stack starved), 15–30% packet loss,
periodic drops every ~10 s. OTA impossible under these conditions.

Fix: CAN telemetry is best-effort — drop frames rather than blocking the plant thread:
```cpp
can_send(dev, &zf, K_NO_WAIT, nullptr, nullptr);
```

**CAN state-change callback added** (`can_rx.cpp`):
Gives clean one-time log messages for state transitions rather than driver-level spam:
```
CAN: bus OK (error-active) tx_err=0 rx_err=0
CAN: error-warning tx_err=96 rx_err=0 — check wiring/termination
CAN: BUS-OFF — no ACK received. Check: RS pin to GND, CAN-H/L wiring, 120Ω termination
```

### 1.5 Raspberry Pi CAN-over-SPI Setup

**Hardware:** MCP2515 module (8 MHz crystal) connected via SPI0. Ubuntu 24.04 on Pi.

**`/boot/firmware/config.txt` addition:**
```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25,spimaxfrequency=2000000
```
GPIO25 = INT pin (forum.raspberrypi.com/viewtopic.php?t=141052 wiring).

**Bring up at 500 kbps:**
```bash
sudo ip link set can0 up type can bitrate 500000
candump can0
```

**Confirmed working:** `mcp251x spi0.0 can0: MCP2515 successfully initialized.`

---

## 2. What Is NOT Done (CAN Physical Layer Deferred)

The STM32 FDCAN1 pins are on **CN11** (long morpho header), which requires soldering to
access. The transceiver was incorrectly connected to CN9 (ZIO/Arduino header).

**Correct FDCAN1 physical pins:**
```
CN11 pin 55  →  PD1  →  FDCAN1_TX  →  SN65HVD230 D  (driver input)
CN11 pin 57  →  PD0  →  FDCAN1_RX  →  SN65HVD230 R  (receiver output)
```

**SN65HVD230 wiring checklist (for when morpho is accessible):**
| Pin | Connect to | Note |
|-----|-----------|------|
| VCC | Nucleo 3.3V (CN11/CN12) | NOT Pi 3.3V |
| GND | Nucleo GND (CN11/CN12) | common ground essential |
| RS  | GND | MUST be pulled LOW — floating = standby mode, no TX |
| D   | CN11 pin 55 (PD1) | FDCAN1_TX |
| R   | CN11 pin 57 (PD0) | FDCAN1_RX |
| CANH/CANL | Pi MCP2515 CANH/CANL | add 120 Ω terminator at each end |

---

## 3. Codebase Architecture Reference

### 3.1 Control Flow

```
┌─────────────────────────────────────────────────────────┐
│                  STM32H753ZI @ 480 MHz                   │
│                                                          │
│  CAN RX thread (prio 3)                                  │
│    FDCAN1 → decode ACTUATOR_CMD_1 → g_cmd (mutex)        │
│                         │                                │
│  Plant thread (prio 5, 10 ms timer)                      │
│    reads g_cmd → PlantModel::step() → g_state (mutex)   │
│    → can_tx_send_all() — 13 frames @ 100/50/10 ms rates  │
│    → k_sem_give(g_wdt_sem)                               │
│                         │                                │
│  HTTP server (prio 10)                                   │
│    GET /dash?... → apply_web_cmd() → g_cmd (mutex)       │
│    (alternative control path — same g_cmd)               │
│                                                          │
│  WDT thread (prio 2)                                     │
│    10 s grace → then k_sem_take(g_wdt_sem, K_MSEC(500)) │
└─────────────────────────────────────────────────────────┘
```

### 3.2 Key Shared State

| Symbol | Type | Protected by | Written by | Read by |
|--------|------|-------------|-----------|--------|
| `g_cmd` | `sim::ActuatorCmd` | `g_cmd_mutex` | CAN RX, HTTP | Plant thread |
| `g_state` | `plant::PlantState` | `g_state_mutex` | Plant thread | HTTP, CAN TX |
| `g_surface_mu` | `double` | none (atomic write) | Shell | Plant thread |
| `g_sys_stats` | `SysStats` | `g_stats_mutex` | Stats thread, Plant | HTTP, Shell |

### 3.3 `sim::ActuatorCmd` Fields

```cpp
struct ActuatorCmd {
    bool          system_enable;          // master enable
    GearPosition  gear_position;          // NEUTRAL=0, FORWARD=1, REVERSE=2
    double        steer_cmd_deg;          // [-45, +45]
    double        drive_torque_cmd_nm;    // [0, 145000] — always positive from UI/CAN
    double        brake_cmd_pct;          // [0, 100]
    double        last_update_t_s;        // k_uptime timestamp — drives CAN watchdog
    int           mode;                   // mode bits (unused)
};
```

**Important:** `drive_torque_cmd_nm` is always ≥ 0 from both the HTTP dashboard and CAN.
Direction is encoded exclusively via `gear_position`. Both `DrivePlant` and
`VehicleSubsystem` were fixed to read `gear_position` for direction intent.

### 3.4 CAN TX Frame Map (plant → bus, 500 kbps)

| ID | Name | Rate | Key signals |
|----|------|------|-------------|
| `0x0CFF0028` | IMU_ACC | 10 ms | ax, ay (m/s²) |
| `0x0CFF0128` | IMU_GYR | 10 ms | gz = yaw_rate |
| `0x0CFF202A` | WHEELS_1 | 10 ms | ω FL/FR/RL/RR (rad/s) |
| `0x0CFF212A` | STEER_STATE | 10 ms | steer_deg, δ FL/FR |
| `0x18FF50F0` | VEHICLE_STATE_1 | 10 ms | v_mps, a_long, yaw_rate |
| `0x18FF51F0` | MOTOR_STATE_1 | 10 ms | torque, power, RPM |
| `0x18FF52F0` | BRAKE_STATE | 10 ms | brake_force_kN, regen_kW |
| `0x18FF302B` | BATT_STATE | 50 ms | V, I, SoC%, power_kW |
| `0x18FF402C` | RADAR_1 | 50 ms | (zeros — not modelled) |
| `0x18FF53F0` | POSITION_STATE | 50 ms | x_m, y_m |
| `0x18FF54F0` | ORIENTATION_STATE | 50 ms | yaw_deg, yaw_rate_dps |
| `0x18FF1029` | GNSS_LL | 100 ms | lat, lon (flat-earth) |
| `0x18FF1129` | GNSS_AV | 100 ms | alt, vn, ve, fix |
| `0x18FF55F0` | DRIVETRAIN_STATE | 100 ms | gear_ratio, wheel_radius |
| `0x18FF60F0` | DIAGNOSTIC_STATE | 100 ms | sim_time, loop_us |

CAN RX frame watched: `0x18EFF021` (ACTUATOR_CMD_1, J1939 PGN EFF0, source 0x21).

### 3.5 HTTP/HTTPS Server Routes

```
GET  /           → login page
POST /login      → set session cookie → redirect /dash
GET  /dash       → dashboard (auth required)
GET  /dash?...   → apply_web_cmd() + dashboard
GET  /logout     → invalidate session
GET  /ota        → OTA upload page (auth required)
POST /api/firmware → stream firmware to slot1 (auth required)
POST /api/reboot → cold reboot into MCUboot (auth required)
```

Auth: Bearer token (`Authorization: Bearer <hex>`) OR session cookie (`sid=`).
Token defined in `zephyr/src/http/http_auth.hpp` (gitignored, generated by
`scripts/gen_http_token.sh`).

### 3.6 TLS Setup

- Protocol: TLS 1.2, ECDHE-RSA-AES128-GCM-SHA256
- Cert + key: `zephyr/certs/server.crt` / `server.key` (self-signed, IP SAN 192.168.1.80)
- Embedded at build time via `generate_inc_file_for_target()` → `server_cert.inc`, `server_key.inc`
- Loaded via `tls_creds_init()` in `zephyr/src/tls/tls_creds.cpp`
- Credential tag: `HDV_TLS_SERVER_TAG = 1`

The TLS credentials infrastructure is already in place and **reusable for MQTT(S)**.

### 3.7 Flash Layout (MCUboot dual-bank)

```
0x08000000  MCUboot    128 KB  (bank 1, sector 0)
0x08020000  Storage    128 KB  (bank 1, sector 1)
0x08040000  Slot 0     640 KB  (active app — bank 1, sectors 2-6)
0x08100000  Slot 1     640 KB  (OTA staging — bank 2, sectors 0-4)
0x081E0000  Scratch    128 KB  (bank 2, sector 7)
```

App size ~370 KB (70% of 512 KB flash region). OTA uses `BOOT_UPGRADE_TEST` — new image
must call `boot_write_img_confirmed()` at boot or MCUboot rolls back.

### 3.8 Thread Priorities and Stacks

| Thread | Priority | Stack | Function |
|--------|----------|-------|---------|
| WDT | 2 | 2048 B | IWDG heartbeat |
| CAN RX | 3 | 1024 B | FDCAN1 → g_cmd |
| Plant | 5 | 16384 B | physics + CAN TX |
| HTTP | 10 | 16384 B | HTTPS server (TLS ~12 KB headroom) |
| Stats | 11 | 1024 B | heap + perf metrics at 1 Hz |
| LED | 12 | 512 B | status LED |

---

## 4. Next Session — MQTT(S) Control

### 4.1 Goal

Add an MQTT(S) subscriber on the STM32 that accepts ACTUATOR_CMD_1-equivalent messages
from an external broker (Pi or cloud). The web dashboard gets a toggle to switch the
active control source between **CAN** and **MQTT**.

### 4.2 Design

**New `g_cmd` source: MQTT subscriber thread**

The cleanest approach re-uses the existing `g_cmd` / `g_cmd_mutex` pattern. The MQTT
subscriber decodes incoming JSON (or binary) messages and writes to `g_cmd` exactly as
the CAN RX thread does. The plant thread is unaware of the source.

```
┌─────────────────────────────────────────────────────────┐
│                                                          │
│  CAN RX thread    ──┐                                    │
│                     ├─→ g_cmd_mutex → g_cmd             │
│  MQTT RX thread   ──┘        ↑                          │
│                               │                          │
│  HTTP /dash?...  ─────────────┘  (apply_web_cmd)        │
│                                                          │
│  Control source selector: g_ctrl_source (atomic)        │
│    CAN_SOURCE | MQTT_SOURCE | HTTP_SOURCE               │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**Source arbitration** — simple priority / enable flag:

Option A (simplest): Each source writes `g_cmd` independently. The CAN watchdog already
resets commands if no fresh CAN frame arrives in 500 ms. Add an equivalent MQTT watchdog.
The dashboard toggle just enables/disables which thread is allowed to write.

Option B (explicit): Add `atomic_t g_ctrl_source` (enum CAN/MQTT/HTTP). Each thread checks
before writing. Dashboard POST `/api/ctrl_source` toggles it.

**Recommendation: Option A** — least code, keeps the existing watchdog safety model.

### 4.3 TLS Reuse

`tls_creds_init()` already loads the server cert+key into tag `HDV_TLS_SERVER_TAG = 1`.
For MQTT connecting to an external broker (Mosquitto on Pi), the STM32 acts as a TLS
*client*. The broker's CA cert needs to be loaded into `HDV_TLS_CA_TAG = 2` (already
reserved in `tls_creds.hpp`). For a Pi-local broker with a self-signed cert, load the
same `server.crt` as the CA.

### 4.4 Zephyr MQTT Library

Zephyr has a built-in MQTT client (`CONFIG_MQTT_LIB=y`). It works over a pre-connected
TLS socket — the same `IPPROTO_TLS_1_2` socket pattern already used by the HTTP server.

Minimal Kconfig additions:
```conf
CONFIG_MQTT_LIB=y
CONFIG_MQTT_KEEPALIVE=60
CONFIG_MQTT_CLEAN_SESSION=y
```

MQTT topic structure suggestion:
```
hdv/cmd/actuator    ← STM32 subscribes — inbound control
hdv/state/vehicle   ← STM32 publishes — outbound telemetry (optional, mirrors CAN TX)
```

### 4.5 Dashboard Toggle

Add to `http_page.cpp` (near the existing gear/enable controls):

```html
<div class="ctrl-source">
  <label>Control source:</label>
  <a href="/dash?ctrl=can"  class="btn [active if CAN]">CAN</a>
  <a href="/dash?ctrl=mqtt" class="btn [active if MQTT]">MQTT</a>
</div>
```

`apply_web_cmd()` in `http_cmd.cpp` handles the new `ctrl=` parameter and writes
`g_ctrl_source`.

### 4.6 Files to Create/Modify

| File | Change |
|------|--------|
| `zephyr/src/mqtt/mqtt_client.cpp` | NEW — MQTT subscriber thread |
| `zephyr/src/mqtt/mqtt_client.hpp` | NEW — public API |
| `zephyr/CMakeLists.txt` | add `mqtt_client.cpp` to APP_SRCS |
| `zephyr/prj.conf` | add `CONFIG_MQTT_LIB=y` + keepalive |
| `zephyr/src/http/http_cmd.cpp` | add `ctrl=` param handler |
| `zephyr/src/http/http_page.cpp` | add source toggle to dashboard |
| `zephyr/src/main.cpp` | declare `g_ctrl_source` |

### 4.7 Mosquitto on the Pi

```bash
sudo apt install mosquitto mosquitto-clients
# For TLS: copy server.crt from repo as CA cert
sudo mosquitto_passwd -c /etc/mosquitto/passwd hdv
```

Test round-trip once STM32 MQTT is working:
```bash
# Pi publishes command:
mosquitto_pub -h localhost -t hdv/cmd/actuator \
  -m '{"enable":1,"gear":"F","torque":50000,"steer":0,"brake":0}'

# Watch STM32 UART for: [MQTT] cmd received → g_cmd updated
```

---

## 5. Build & Flash Reference

```bash
# Build
cd /home/baloo/zephyrproject
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild

# OTA flash (preferred — board must be up and reachable)
bash /home/baloo/repos/vehicle-Dynamics-Sim-Can/scripts/ota_flash.sh

# Direct flash via pyocd (when board is unstable or OTA unavailable)
.venv/bin/pyocd flash --target stm32h743xx \
  build/zephyr/zephyr/zephyr.signed.hex
# Note: USB uninit error after write is benign — board resets normally

# Force CMake reconfigure (after prj.conf / Kconfig changes)
.venv/bin/west build -b nucleo_h753zi \
  /home/baloo/repos/vehicle-Dynamics-Sim-Can/zephyr --sysbuild \
  -- -DFORCE_RECONFIGURE=1

# Monitor UART
picocom -b 115200 /dev/ttyACM0

# Shell diagnostics (no login required)
can stats
can rx_frame
stats
mem
threads

# Shell diagnostics (login required: login <token>)
can tx_test steer=0 torque=50000 brake=0 enable=1
plant reset
plant inject ...
```

---

## 6. Known Issues / Watch-outs

| Issue | Status | Notes |
|-------|--------|-------|
| CAN physical wiring | Deferred | CN11 pins need soldering; see §2 |
| FDCAN BUS-OFF WDT resets | Fixed (K_NO_WAIT) | Only manifests without transceiver |
| OTA `Expect: 100-continue` | Fixed | `-H "Expect:"` in script |
| First boot overrun (~87 ms) | Accepted | Init logging fills deferred buffer; one-time only |
| TLS handshake slow (~4 s) | By design | RSA-2048 on M7 without HW accel |
| Session table lost on reboot | By design | Browser must re-login after OTA |
| `root-rsa-2048.pem` signing key | Dev only | Public MCUboot test key — replace before Phase 4 RDP |
