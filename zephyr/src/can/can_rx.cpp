// zephyr/src/can/can_rx.cpp
// CAN RX thread — Phase 3.
//
// Opens FDCAN1 (DT_CHOSEN zephyr,canbus), registers a hardware RX filter
// for ACTUATOR_CMD_1 (J1939 0x98EFF021), then decodes each received frame
// directly into g_cmd under g_cmd_mutex.
//
// Security: signals are clamped to physical limits after decoding.
// Anti-replay: 16-bit sequence number in bytes 6-7 detects replayed frames.
//
// Priority 3 — below ETH driver (-14), above plant sim (Phase 4, prio 5).

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "sim/actuator_cmd.hpp"
#include "can/actuator_cmd_codec.hpp"
#include "mqtt/mqtt_client.hpp"
#include "state/sim_state.hpp"
#include "state/control_bus.hpp"
#include "utils/mutex_guard.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state buses (defined in main.cpp) ─────────────────────────────────
extern hdv::SimStateBus  g_sim_bus;
extern hdv::ControlBus   g_ctrl_bus;
extern struct k_mutex    g_sim_cmd_mtx;

// ── ACTUATOR_CMD_1 — bare 29-bit J1939 ID ────────────────────────────────────
static const uint32_t ACTUATOR_CMD_ID = 0x18EFF021u;

// ── Zephyr msgq for received frames ──────────────────────────────────────────
K_MSGQ_DEFINE(g_can_rx_msgq, sizeof(struct can_frame), 8, 4);

using can_codec::decode_actuator_cmd;

// ── CAN state change callback ─────────────────────────────────────────────────
// Logs bus-off and error-passive transitions so we can diagnose wiring issues
// without being flooded by Zephyr driver internals.
static void can_state_cb(const struct device* dev, enum can_state state,
                         struct can_bus_err_cnt err_cnt, void* /*user*/)
{
    (void)dev;
    switch (state) {
    case CAN_STATE_ERROR_ACTIVE:
        LOG_INF("CAN: bus OK (error-active) tx_err=%u rx_err=%u",
                err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
        break;
    case CAN_STATE_ERROR_WARNING:
        LOG_WRN("CAN: error-warning tx_err=%u rx_err=%u — check wiring/termination",
                err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
        break;
    case CAN_STATE_ERROR_PASSIVE:
        LOG_WRN("CAN: error-passive tx_err=%u rx_err=%u — transceiver or termination fault",
                err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
        break;
    case CAN_STATE_BUS_OFF:
        LOG_ERR("CAN: BUS-OFF — no ACK received. Check: RS pin to GND, "
                "CAN-H/L wiring, 120Ω termination at each end");
        break;
    default:
        break;
    }
}

// ── CAN RX thread ─────────────────────────────────────────────────────────────
static void can_rx_thread(void*, void*, void*)
{
    const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev)) {
        LOG_ERR("CAN: device not ready");
        return;
    }

#ifdef CONFIG_HDV_CAN_LOOPBACK
    int lret = can_set_mode(dev, CAN_MODE_LOOPBACK);
    if (lret < 0) {
        LOG_WRN("CAN: can_set_mode(LOOPBACK) failed: %d (continuing)", lret);
    } else {
        LOG_INF("CAN: loopback mode enabled (no transceiver required)");
    }
#endif

    struct can_filter filter{};
    filter.id    = ACTUATOR_CMD_ID;
    filter.mask  = CAN_EXT_ID_MASK;
    filter.flags = CAN_FILTER_IDE;

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

    can_set_state_change_callback(dev, can_state_cb, nullptr);

    LOG_INF("CAN RX ready — FDCAN1 @ 500 kbps, filter_id=%d, "
            "watching 0x%08X (ACTUATOR_CMD_1)", fid, ACTUATOR_CMD_ID);

    // Anti-replay: track last received sequence number (bytes 6-7)
    uint16_t last_seq = 0;
    bool     seq_init = false;

    struct can_frame zf{};
    while (true) {
        int r = k_msgq_get(&g_can_rx_msgq, &zf, K_MSEC(500));
        if (r == 0) {
            // Anti-replay check
            uint16_t seq = (uint16_t)zf.data[6] | ((uint16_t)zf.data[7] << 8);
            if (seq_init && seq <= last_seq) {
                LOG_WRN("CAN: possible replay — seq %u <= last %u, dropping", seq, last_seq);
                atomic_inc(&g_ctrl_bus.can_timeout_count);
                continue;
            }
            last_seq = seq;
            seq_init = true;

            sim::ActuatorCmd c{};
            decode_actuator_cmd(zf.data, c);
            c.last_update_t_s = k_uptime_get_32() / 1000.0;

            if (atomic_get(&g_ctrl_bus.ctrl_source) == CTRL_CAN) {
                hdv::MutexGuard g(g_sim_cmd_mtx);
                g_sim_bus.cmd = c;
            }

            atomic_inc(&g_ctrl_bus.can_rx_count);
        } else {
            atomic_inc(&g_ctrl_bus.can_timeout_count);
        }
    }
}

K_THREAD_DEFINE(can_rx_tid, 1024, can_rx_thread, NULL, NULL, NULL, 3, 0, 0);

// ── TX helper (used by shell 'can tx_test') ───────────────────────────────────
extern "C" int can_tx_test_frame(double steer_deg, double torque_nm,
                                 double brake_pct, bool enable)
{
    const struct device* dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(dev)) return -ENODEV;

    struct can_frame zf{};
    zf.id    = ACTUATOR_CMD_ID;
    zf.flags = CAN_FRAME_IDE;
    zf.dlc   = 8;

    auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };

    int16_t steer_raw  = static_cast<int16_t>(clamp(steer_deg  / 0.1,  -32768.0, 32767.0));
    int16_t torque_raw = static_cast<int16_t>(clamp(torque_nm  / 10.0, -32768.0, 32767.0));
    uint8_t brake_raw  = static_cast<uint8_t>(clamp(brake_pct,  0.0,   100.0));

    // Static sequence counter for loopback test frames
    static uint16_t tx_seq = 0;
    ++tx_seq;

    zf.data[0] = (enable ? 0x01u : 0x00u) | (0x01u << 1);
    zf.data[1] = static_cast<uint8_t>(steer_raw  & 0xFF);
    zf.data[2] = static_cast<uint8_t>((steer_raw  >> 8) & 0xFF);
    zf.data[3] = static_cast<uint8_t>(torque_raw & 0xFF);
    zf.data[4] = static_cast<uint8_t>((torque_raw >> 8) & 0xFF);
    zf.data[5] = brake_raw;
    zf.data[6] = static_cast<uint8_t>(tx_seq & 0xFF);
    zf.data[7] = static_cast<uint8_t>((tx_seq >> 8) & 0xFF);

    return can_send(dev, &zf, K_MSEC(100), nullptr, nullptr);
}
