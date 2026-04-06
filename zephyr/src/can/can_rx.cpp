// zephyr/src/can/can_rx.cpp
// CAN RX thread — Phase 3.
//
// Opens FDCAN1 (DT_CHOSEN zephyr,canbus), registers a hardware RX filter
// for ACTUATOR_CMD_1 (J1939 0x98EFF021), then decodes each received frame
// directly into g_cmd under g_cmd_mutex.
//
// No heap, no std::string, no can_codec.cpp dependency.
// All signal positions are taken from can_map_static.hpp at compile time.
//
// Priority 3 — below ETH driver (-14), above plant sim (Phase 4, prio 5).

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>

#include "sim/actuator_cmd.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

// ── Shared state (defined in main.cpp) ───────────────────────────────────────
extern sim::ActuatorCmd  g_cmd;
extern struct k_mutex    g_cmd_mutex;
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_timeout_count;
extern volatile double   g_last_rx_t;

// ── ACTUATOR_CMD_1 — bare 29-bit J1939 ID (CAN_EFF_FLAG stripped) ────────────
// DBC stores 0x98EFF021 (SocketCAN EFF convention: 0x80000000 | 0x18EFF021).
// Zephyr CAN API requires the raw 29-bit value without the flag.
static const uint32_t ACTUATOR_CMD_ID = 0x18EFF021u;

// ── Zephyr msgq for received Zephyr-native CAN frames ────────────────────────
// CAN_MAX_DLEN is 8 for classic CAN; 4-byte aligned so alignment=4.
K_MSGQ_DEFINE(g_can_rx_msgq, sizeof(struct can_frame), 8, 4);

// ── Inline decoder ────────────────────────────────────────────────────────────
// Directly encodes ACTUATOR_CMD_1 bit layout (Intel/LSB-first byte order):
//   Byte 0  [0:0]   system_enable       unsigned, factor 1.0
//   Byte 0  [1:2]   gear_position       unsigned, factor 1.0
//   Byte 0  [3:4]   mode                unsigned, factor 1.0  (ignored)
//   Byte 1-2[8:23]  steer_cmd_deg       signed,   factor 0.1
//   Byte 3-4[24:39] drive_torque_cmd_nm signed,   factor 10.0
//   Byte 5  [40:47] brake_cmd_pct       unsigned, factor 1.0

static void decode_actuator_cmd(const struct can_frame& zf, sim::ActuatorCmd& c)
{
    const uint8_t* d = zf.data;

    c.system_enable        = (d[0] >> 0) & 0x01u;
    c.gear_position        = static_cast<sim::GearPosition>((d[0] >> 1) & 0x03u);
    c.mode                 = (d[0] >> 3) & 0x03u;

    int16_t steer_raw      = static_cast<int16_t>(
                                 (uint16_t)d[1] | ((uint16_t)d[2] << 8));
    int16_t torque_raw     = static_cast<int16_t>(
                                 (uint16_t)d[3] | ((uint16_t)d[4] << 8));

    c.steer_cmd_deg        = steer_raw  * 0.1;
    c.drive_torque_cmd_nm  = torque_raw * 10.0;
    c.brake_cmd_pct        = d[5] * 1.0;
}

// ── CAN RX thread ─────────────────────────────────────────────────────────────

static void can_rx_thread(void*, void*, void*)
{
    const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev)) {
        LOG_ERR("CAN: device not ready");
        return;
    }

#ifdef CONFIG_XCMG_CAN_LOOPBACK
    // Internal loopback — frames TX'd on this bus are echoed back to RX filters.
    // Useful for self-test without a physical transceiver.
    int lret = can_set_mode(dev, CAN_MODE_LOOPBACK);
    if (lret < 0) {
        LOG_WRN("CAN: can_set_mode(LOOPBACK) failed: %d (continuing)", lret);
    } else {
        LOG_INF("CAN: loopback mode enabled (no transceiver required)");
    }
#endif

    // Register hardware RX filter — exact 29-bit ID match.
    struct can_filter filter{};
    filter.id    = ACTUATOR_CMD_ID;
    filter.mask  = CAN_EXT_ID_MASK;  // exact match on all 29 bits
    filter.flags = CAN_FILTER_IDE;   // extended frame

    int fid = can_add_rx_filter_msgq(dev, &g_can_rx_msgq, &filter);
    if (fid < 0) {
        LOG_ERR("CAN: add_rx_filter_msgq failed: %d", fid);
        return;
    }

    int ret = can_start(dev);
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("CAN: can_start failed: %d", ret);
        return;
    }

    LOG_INF("CAN RX ready — FDCAN1 @ 500 kbps, filter_id=%d, "
            "watching 0x%08X (ACTUATOR_CMD_1)", fid, ACTUATOR_CMD_ID);

    // ── RX loop ───────────────────────────────────────────────────────────────
    // 500 ms timeout so we can count watchdog misses even without a plant thread.
    struct can_frame zf{};
    while (true) {
        int r = k_msgq_get(&g_can_rx_msgq, &zf, K_MSEC(500));
        if (r == 0) {
            sim::ActuatorCmd c{};
            decode_actuator_cmd(zf, c);
            c.last_update_t_s = k_uptime_get_32() / 1000.0;

            k_mutex_lock(&g_cmd_mutex, K_FOREVER);
            g_cmd = c;
            k_mutex_unlock(&g_cmd_mutex);

            g_can_rx_count++;
            g_last_rx_t = c.last_update_t_s;
        } else {
            // Timeout — no frame received in 500 ms.
            g_can_timeout_count++;
        }
    }
}

K_THREAD_DEFINE(can_rx_tid, 1024, can_rx_thread, NULL, NULL, NULL, 3, 0, 0);

// ── TX helper (used by shell 'can tx_test') ───────────────────────────────────
// Exposed via extern so debug_cmds.cpp can call it.
// Sends one ACTUATOR_CMD_1 frame with the supplied values.
// In loopback mode the frame comes back and g_cmd is updated by the RX thread.

extern "C" int can_tx_test_frame(double steer_deg, double torque_nm,
                                 double brake_pct, bool enable)
{
    const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev)) return -ENODEV;

    struct can_frame zf{};
    zf.id    = ACTUATOR_CMD_ID;
    zf.flags = CAN_FRAME_IDE;
    zf.dlc   = 8;

    // Encode ACTUATOR_CMD_1 signals (inverse of decode_actuator_cmd)
    auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };

    int16_t steer_raw  = static_cast<int16_t>(clamp(steer_deg  / 0.1,  -32768.0, 32767.0));
    int16_t torque_raw = static_cast<int16_t>(clamp(torque_nm  / 10.0, -32768.0, 32767.0));
    uint8_t brake_raw  = static_cast<uint8_t>(clamp(brake_pct,  0.0,   100.0));

    zf.data[0] = (enable ? 0x01u : 0x00u) | (0x01u << 1); // enable + FORWARD gear
    zf.data[1] = static_cast<uint8_t>(steer_raw  & 0xFF);
    zf.data[2] = static_cast<uint8_t>((steer_raw  >> 8) & 0xFF);
    zf.data[3] = static_cast<uint8_t>(torque_raw & 0xFF);
    zf.data[4] = static_cast<uint8_t>((torque_raw >> 8) & 0xFF);
    zf.data[5] = brake_raw;
    zf.data[6] = 0;
    zf.data[7] = 0;

    return can_send(dev, &zf, K_MSEC(100), nullptr, nullptr);
}
