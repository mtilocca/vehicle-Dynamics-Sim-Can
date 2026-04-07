// zephyr/src/watchdog/watchdog_thread.cpp
// Hardware watchdog heartbeat thread.
//
// The plant_thread gives g_wdt_sem every 10 ms step.
// This thread waits on that semaphore with a 500 ms timeout and strokes
// the hardware IWDG. If the plant thread hangs for > 500 ms the IWDG
// fires and resets the MCU.
//
// Priority 2 — below CAN RX (3), above ETH driver handled at IRQ level.
// Stack 512 B — minimal; only calls wdt_feed().

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Semaphore given by plant_thread every 10 ms step (defined in main.cpp)
extern struct k_sem g_wdt_sem;

static void watchdog_thread(void*, void*, void*)
{
    // Grace period: wait for plant_thread to finish subsystem initialization
    // and enter its main loop before arming the IWDG.
    // 8 subsystems × complex init can take 1-2 s on first boot.
    k_msleep(50000);

    const struct device* wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));
    if (!device_is_ready(wdt)) {
        LOG_WRN("WDT: IWDG device not ready — watchdog disabled");
        // Thread stays alive but does nothing; plant still runs safely.
        while (true) {
            k_sem_take(&g_wdt_sem, K_MSEC(500));
        }
        return;
    }

    struct wdt_timeout_cfg cfg{};
    cfg.window.min = 0;
    cfg.window.max = 1000;  // 1 second hardware timeout
    cfg.callback   = nullptr;
    cfg.flags      = WDT_FLAG_RESET_SOC;

    int ch = wdt_install_timeout(wdt, &cfg);
    if (ch < 0) {
        LOG_ERR("WDT: wdt_install_timeout failed: %d", ch);
        return;
    }

    int rc = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (rc < 0) {
        LOG_ERR("WDT: wdt_setup failed: %d", rc);
        return;
    }

    LOG_INF("WDT: hardware watchdog armed (1 s timeout, stroked by plant_thread)");

    while (true) {
        int r = k_sem_take(&g_wdt_sem, K_MSEC(500));
        if (r == 0) {
            wdt_feed(wdt, ch);
        } else {
            // Plant thread did not step within 500 ms — MCU will reset
            // once the 1 s hardware timeout expires. Log the fault first.
            LOG_ERR("WDT: plant_thread stall detected — hardware reset imminent");
        }
    }
}

K_THREAD_DEFINE(watchdog_tid, 512, watchdog_thread, NULL, NULL, NULL, 2, 0, 0);
