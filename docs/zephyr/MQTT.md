# MQTT(S) Control — Implementation Plan

Add an MQTT(S) subscriber on the STM32 that accepts actuator commands from an external
broker (Mosquitto on Pi or cloud). The web dashboard gets a toggle to switch the active
control source between CAN and MQTT.

---

## Design

### `g_cmd` source arbitration

The cleanest approach reuses the existing `g_cmd` / `g_cmd_mutex` pattern. The MQTT
subscriber decodes incoming messages and writes to `g_cmd` exactly as the CAN RX thread
does. The plant thread is unaware of the source.

```
┌─────────────────────────────────────────────────────────┐
│                                                          │
│  CAN RX thread    ──┐                                    │
│                     ├─→ g_cmd_mutex → g_cmd             │
│  MQTT RX thread   ──┘        ↑                          │
│                               │                          │
│  HTTP /dash?...  ─────────────┘  (apply_web_cmd)        │
│                                                          │
│  Control source:  g_ctrl_source (atomic)                 │
│    CAN_SOURCE | MQTT_SOURCE | HTTP_SOURCE                │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

**Source arbitration** — Option A (recommended, least code):

Each source writes `g_cmd` independently. The existing CAN watchdog resets commands if no
fresh CAN frame arrives in 500 ms — add an equivalent MQTT watchdog. The dashboard toggle
just enables/disables which thread is allowed to write.

Option B (explicit `g_ctrl_source` atomic enum) adds more control but is only needed if
sources must be hard-locked out from each other.

---

## TLS Reuse

`tls_creds_init()` already loads the server cert + key into `HDV_TLS_SERVER_TAG = 1`.
For MQTT connecting to an external broker, the STM32 acts as a TLS *client*. The broker's
CA cert is loaded into `HDV_TLS_CA_TAG = 2` (reserved in `tls_creds.hpp`). For a Pi-local
broker with a self-signed cert, load the same `server.crt` as the CA.

---

## Zephyr MQTT Library

Zephyr's built-in MQTT client (`CONFIG_MQTT_LIB=y`) works over a pre-connected TLS socket
— the same `IPPROTO_TLS_1_2` pattern already used by the HTTP server.

**`prj.conf` additions:**
```conf
CONFIG_MQTT_LIB=y
CONFIG_MQTT_KEEPALIVE=60
CONFIG_MQTT_CLEAN_SESSION=y
```

**Topic structure:**
```
hdv/cmd/actuator    ← STM32 subscribes — inbound control commands
hdv/state/vehicle   ← STM32 publishes  — outbound telemetry (mirrors CAN TX, optional)
```

**Message format (JSON):**
```json
{"enable":1,"gear":"F","torque":50000,"steer":0,"brake":0}
```

---

## Dashboard Toggle

Add to `http_page.cpp` near the existing gear/enable controls:

```html
<div class="ctrl-source">
  <label>Control source:</label>
  <a href="/dash?ctrl=can"  class="btn [active if CAN]">CAN</a>
  <a href="/dash?ctrl=mqtt" class="btn [active if MQTT]">MQTT</a>
</div>
```

`apply_web_cmd()` in `http_cmd.cpp` handles the new `ctrl=` parameter and writes
`g_ctrl_source`.

---

## Files to Create / Modify

| File | Change |
|------|--------|
| `zephyr/src/mqtt/mqtt_client.hpp` | NEW — public API |
| `zephyr/src/mqtt/mqtt_client.cpp` | NEW — MQTT subscriber thread |
| `zephyr/CMakeLists.txt` | Add `mqtt_client.cpp` to `APP_SRCS` |
| `zephyr/prj.conf` | Add `CONFIG_MQTT_LIB=y` + keepalive |
| `zephyr/src/main.cpp` | Declare `g_ctrl_source` |
| `zephyr/src/http/http_cmd.cpp` | Add `ctrl=` param handler |
| `zephyr/src/http/http_page.cpp` | Add source toggle to dashboard |

---

## Mosquitto Setup on Pi

```bash
sudo apt install mosquitto mosquitto-clients
sudo mosquitto_passwd -c /etc/mosquitto/passwd hdv
# For TLS: copy server.crt from repo as the CA cert
```

**Test round-trip once STM32 MQTT is working:**
```bash
# Pi publishes a command:
mosquitto_pub -h localhost -t hdv/cmd/actuator \
  -m '{"enable":1,"gear":"F","torque":50000,"steer":0,"brake":0}'

# Watch STM32 UART for:
# [INF] MQTT: cmd received → g_cmd updated
```
