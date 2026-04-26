// zephyr/src/main.cpp
// Heavy-Duty Electric Vehicle Plant Simulator — Zephyr RTOS entry point

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/dfu/mcuboot.h>

#include "state/sim_state.hpp"
#include "state/control_bus.hpp"
#include "state/system_health.hpp"

LOG_MODULE_REGISTER(hdv_sim, LOG_LEVEL_INF);

// ── Shared state buses ────────────────────────────────────────────────────────
hdv::SimStateBus      g_sim_bus{};      // plant output + actuator command
hdv::ControlBus       g_ctrl_bus{};     // source arbitration + counters + mu
hdv::SystemHealthBus  g_health_bus{};   // system telemetry (stats_thread writes)

// K_MUTEX_DEFINE must be at file scope — Zephyr linker constraint.
K_MUTEX_DEFINE(g_sim_plant_mtx);   // guards g_sim_bus.plant
K_MUTEX_DEFINE(g_sim_cmd_mtx);     // guards g_sim_bus.cmd
K_MUTEX_DEFINE(g_health_mtx);      // guards g_health_bus.stats

// ── Watchdog semaphore — given by plant_thread every 10 ms ───────────────────
#ifdef CONFIG_WATCHDOG
K_SEM_DEFINE(g_wdt_sem, 0, 1);
#endif

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("Heavy-Duty Electric Vehicle Plant Simulator");
    LOG_INF("Board : nucleo_h753zi (STM32H753ZI)");
    LOG_INF("Phase : 3 - HTTPS + OTA + CAN (loopback)");
    LOG_INF("========================================");
    LOG_INF("Shell ready on USART3 — type 'help' for commands");

    // Confirm image to MCUboot — without this, MCUboot rolls back on next reset.
    if (boot_is_img_confirmed()) {
        LOG_INF("BOOT: image already confirmed");
    } else {
        int rc = boot_write_img_confirmed();
        if (rc == 0) {
            LOG_INF("BOOT: image confirmed OK");
        } else {
            LOG_ERR("BOOT: image confirm failed (rc=%d) — will roll back on reset", rc);
        }
    }

    while (true) {
        k_msleep(60000);
    }

    return 0;
}
