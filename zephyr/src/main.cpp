// zephyr/src/main.cpp
// Heavy-Duty Electric Vehicle Plant Simulator — Zephyr RTOS entry point

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/dfu/mcuboot.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

LOG_MODULE_REGISTER(hdv_sim, LOG_LEVEL_INF);

// ── Shared state (written by plant thread, read by shell + CAN TX) ────────────
plant::PlantState g_state{};
sim::ActuatorCmd  g_cmd{};

K_MUTEX_DEFINE(g_state_mutex);
K_MUTEX_DEFINE(g_cmd_mutex);

// ── CAN counters — atomic so no mutex needed for increment/read ───────────────
// last_rx_t_s lives inside g_cmd.last_update_t_s (written under g_cmd_mutex)
atomic_t g_can_tx_count      = ATOMIC_INIT(0);
atomic_t g_can_rx_count      = ATOMIC_INIT(0);
atomic_t g_can_timeout_count = ATOMIC_INIT(0);

// ── Surface friction (set via shell 'plant mu', read by plant in Phase 4) ─────
double g_surface_mu = 0.72;

// ── System stats (written by stats thread, read by shell + HTTP) ──────────────
struct SysStats {
    uint32_t plant_loop_us_max = 0;  // worst-case plant step duration
    uint32_t can_rx_total      = 0;
    uint32_t can_timeout_total = 0;
    size_t   heap_used         = 0;
    size_t   heap_free         = 0;
};
SysStats g_sys_stats{};
K_MUTEX_DEFINE(g_stats_mutex);

// ── Watchdog semaphore — given by plant_thread every 10 ms ───────────────────
#ifdef CONFIG_WATCHDOG
K_SEM_DEFINE(g_wdt_sem, 0, 1);
#endif

// ── Plant model pointer (set in Phase 4) ─────────────────────────────────────
namespace plant { class PlantModel; }
plant::PlantModel* g_plant = nullptr;

int main(void)
{
    LOG_INF("========================================");
    LOG_INF("Heavy-Duty Electric Vehicle Plant Simulator");
    LOG_INF("Board : nucleo_h753zi (STM32H753ZI)");
    LOG_INF("Phase : 3b - OTA firmware update");
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
