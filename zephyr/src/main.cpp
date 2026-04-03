// zephyr/src/main.cpp
// XCMG XDE320 Plant Simulator — Zephyr RTOS entry point
// Phase 0 skeleton: boots, logs startup message, shell responds.
// Threads are defined but plant loop is stubbed until Phase 4.

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "plant/plant_model.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "config/vehicle_config.hpp"
#include "can/zephyr_can_iface.hpp"
#include "can/can_map.hpp"
#include "can/actuator_cmd_decoder.hpp"
#include "can/tx_scheduler.hpp"
#include "can/can_codec.hpp"
#include "can/sensor_state_packer.hpp"
#include "sensors/sensor_bank.hpp"

LOG_MODULE_REGISTER(xcmg_sim, LOG_LEVEL_INF);

// ── Global shared state (protected by mutexes) ───────────────────────────────

static plant::PlantState  g_state;
static sim::ActuatorCmd   g_cmd;
static double             g_last_rx_t = 0.0;

K_MUTEX_DEFINE(g_state_mutex);
K_MUTEX_DEFINE(g_cmd_mutex);

// ── CAN RX message queue (8 frames, filled from ISR filter) ──────────────────

#define CAN_RX_MSGQ_DEPTH 8
static struct can_frame can_rx_buf[CAN_RX_MSGQ_DEPTH];  // our SocketCAN-compat struct
K_MSGQ_DEFINE(can_rx_msgq, sizeof(struct can_frame), CAN_RX_MSGQ_DEPTH, 4);

// ── Semaphore kicked by 10 ms timer to wake plant thread ─────────────────────

K_SEM_DEFINE(plant_sem, 0, 1);

// ── 10 ms plant timer ─────────────────────────────────────────────────────────

static void plant_timer_expiry(struct k_timer*)
{
    k_sem_give(&plant_sem);
}
K_TIMER_DEFINE(plant_timer, plant_timer_expiry, nullptr);

// ── ZephyrCanIface singleton ──────────────────────────────────────────────────

static can::ZephyrCanIface g_can_iface;

// ── CAN RX thread — prio 2, drains msgq, decodes ACTUATOR_CMD_1 ──────────────

static void can_rx_thread_fn(void*, void*, void*)
{
    LOG_INF("CAN RX thread started");

    // Lazy init: wait until g_can_iface is open
    while (!g_can_iface.is_open()) {
        k_msleep(10);
    }

    // Decoder is constructed once CAN map is ready (set in main after open)
    extern can::ActuatorCmdDecoder* g_decoder;
    while (g_decoder == nullptr) {
        k_msleep(10);
    }

    struct can_frame frame;
    while (true) {
        if (k_msgq_get(&can_rx_msgq, &frame, K_FOREVER) == 0) {
            k_mutex_lock(&g_cmd_mutex, K_FOREVER);
            g_decoder->decode(frame, g_cmd, k_uptime_get_32() / 1000.0);
            g_last_rx_t = k_uptime_get_32() / 1000.0;
            k_mutex_unlock(&g_cmd_mutex);
        }
    }
}

K_THREAD_DEFINE(can_rx_tid, 2048,
                can_rx_thread_fn, nullptr, nullptr, nullptr,
                2, 0, 0);

// ── Plant thread — prio 5, wakes every 10 ms ─────────────────────────────────

static void plant_thread_fn(void*, void*, void*)
{
    LOG_INF("Plant thread started");

    extern plant::PlantModel*       g_plant;
    extern sensors::SensorBank*     g_sensor_bank;
    extern can::TxScheduler*        g_tx_scheduler;
    extern can::CanMap*             g_can_map;

    // Wait until main() finishes initialisation
    while (g_plant == nullptr || g_sensor_bank == nullptr) {
        k_msleep(10);
    }

    constexpr double dt      = 0.01;
    constexpr double timeout = 0.5;  // 500 ms CAN RX watchdog

    k_timer_start(&plant_timer, K_MSEC(10), K_MSEC(10));

    while (true) {
        k_sem_take(&plant_sem, K_FOREVER);

        const double t = k_uptime_get_32() / 1000.0;

        // ── Read command (with timeout watchdog) ──────────────────────────
        k_mutex_lock(&g_cmd_mutex, K_FOREVER);
        if ((t - g_last_rx_t) > timeout) {
            g_cmd.reset();  // safe mode — zero torque/brake/steer
        }
        sim::ActuatorCmd cmd = g_cmd;
        k_mutex_unlock(&g_cmd_mutex);

        // ── Step plant ────────────────────────────────────────────────────
        k_mutex_lock(&g_state_mutex, K_FOREVER);
        g_plant->step(g_state, cmd, dt);
        g_state.t_s = t;
        plant::PlantState state_copy = g_state;
        k_mutex_unlock(&g_state_mutex);

        // ── Step sensors ──────────────────────────────────────────────────
        g_sensor_bank->step(t, state_copy, dt);
        auto sensor_out = g_sensor_bank->get_output(t);

        // ── TX CAN frames ─────────────────────────────────────────────────
        if (g_tx_scheduler && g_can_map) {
            const auto& tx_frames = g_can_map->get_tx_frames();
            for (const auto& fd : tx_frames) {
                if (g_tx_scheduler->is_due(fd.name, t)) {
                    struct can_frame frame{};
                    frame.can_id  = fd.can_id;
                    frame.can_dlc = static_cast<uint8_t>(fd.dlc);
                    can::SensorStatePacker::pack(sensor_out, fd, frame, *g_can_map);
                    g_can_iface.write_frame(frame);
                }
            }
        }
    }
}

K_THREAD_DEFINE(plant_tid, 16384,
                plant_thread_fn, nullptr, nullptr, nullptr,
                5, 0, 0);

// ── Forward declarations for pointers used by threads ─────────────────────────

can::ActuatorCmdDecoder* g_decoder     = nullptr;
plant::PlantModel*       g_plant       = nullptr;
sensors::SensorBank*     g_sensor_bank = nullptr;
can::TxScheduler*        g_tx_scheduler = nullptr;
can::CanMap*             g_can_map     = nullptr;

// ── main() — Phase 0: boot, log, open CAN ────────────────────────────────────

int main()
{
    LOG_INF("========================================");
    LOG_INF("XCMG XDE320 Plant Simulator booting...");
    LOG_INF("Target: nucleo_h753zi (STM32H753ZI)");
    LOG_INF("========================================");

    // Vehicle configuration (always XCMG XDE320 hardcoded defaults)
    config::VehicleConfig vehicle = config::VehicleConfig::get_default();
    LOG_INF("Vehicle: %s  mass=%.0f kg", vehicle.name.c_str(),
            vehicle.params.drive.mass_kg);

    // Plant model
    static plant::PlantModel plant_inst(vehicle.params);
    g_plant = &plant_inst;

    // Sensor bank
    static sensors::SensorBank sensor_inst;
    g_sensor_bank = &sensor_inst;

    // CAN map
    static can::CanMap can_map;
    if (!can_map.load("can_map.dbc")) {
        LOG_ERR("CAN map load failed — running without CAN TX");
    } else {
        g_can_map = &can_map;
        LOG_INF("CAN map loaded: %zu TX frames",
                can_map.get_tx_frames().size());
    }

    // TX scheduler
    static can::TxScheduler tx_sched;
    if (g_can_map) {
        for (const auto& fd : g_can_map->get_tx_frames()) {
            tx_sched.register_frame(fd.name, fd.cycle_ms);
        }
        g_tx_scheduler = &tx_sched;
    }

    // Actuator command decoder
    static can::ActuatorCmdDecoder decoder(
        can_map, "ACTUATOR_CMD_1", 0.5 /* timeout_s */);
    g_decoder = &decoder;

    // Open FDCAN1
    if (!g_can_iface.open("fdcan1")) {
        LOG_ERR("FDCAN1 open failed — CAN disabled");
    } else {
        // Register RX filter: fill can_rx_msgq from ISR
        g_can_iface.add_rx_filter_msgq(&can_rx_msgq, decoder.get_frame_id());
        LOG_INF("FDCAN1 open. Listening for ACTUATOR_CMD_1 (0x%08X)",
                decoder.get_frame_id());
    }

    LOG_INF("Init complete — plant and CAN threads running");
    return 0;
}
