// zephyr/src/watchdog/watchdog_thread.cpp
// Hardware watchdog heartbeat thread.
//
// Strategy:
//   1. Arm the IWDG immediately at thread start (handles IWDG already running
//      from a prior firmware flash — STM32H7 IWDG cannot be stopped once started).
//   2. Feed it every 500 ms for a 10-second grace period while plant_thread
//      initializes all 8 subsystems.
//   3. After the grace period, switch to semaphore-based feeding — plant_thread
//      gives g_wdt_sem every 10 ms step; if it stalls for >500 ms, MCU resets.
//
// Priority 2 — below CAN RX (3), above ETH driver (handled at IRQ level).
// Stack 512 B — minimal; only calls wdt_feed().

#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Semaphore given by plant_thread every 10 ms step (defined in main.cpp)
extern struct k_sem g_wdt_sem;

static void watchdog_thread(void*, void*, void*)
{
    const struct device* wdt = DEVICE_DT_GET(DT_NODELABEL(iwdg));
    if (!device_is_ready(wdt)) {
        LOG_WRN("WDT: IWDG device not ready — watchdog disabled");
        while (true) { k_sem_take(&g_wdt_sem, K_MSEC(500)); }
        return;
    }

    // Feed channel 0 NOW — the IWDG may already be running from a prior session
    // (STM32H7 IWDG persists across warm resets). This buys us 1 s to complete
    // setup before the old countdown expires. Channel 0 is the only STM32 IWDG
    // channel, so this is safe to call before wdt_install_timeout.
    wdt_feed(wdt, 0);
    LOG_INF("WDT: pre-setup feed done");

    struct wdt_timeout_cfg cfg{};
    cfg.window.min = 0;
    cfg.window.max = 1000;   // 1 second hardware timeout
    cfg.callback   = nullptr;
    cfg.flags      = WDT_FLAG_RESET_SOC;

    int ch = wdt_install_timeout(wdt, &cfg);
    if (ch < 0) {
        // IWDG may already be locked — fall back to channel 0 and keep feeding.
        LOG_WRN("WDT: wdt_install_timeout failed (%d) — using ch 0", ch);
        ch = 0;
    }

    int rc = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (rc < 0) {
        // -EBUSY means IWDG is already running — that's expected after a warm
        // reset. Log and continue; we still need to keep feeding it.
        LOG_WRN("WDT: wdt_setup returned %d (IWDG already running — continuing)", rc);
    }

    wdt_feed(wdt, ch);

    LOG_INF("WDT: armed (1s timeout). Feeding during 10s init grace period...");

    // Feed unconditionally for 10 seconds.
    // This covers:
    //   - plant_thread subsystem init (can take 1-2 s)
    //   - IWDG already running from a prior firmware flash (would fire within 1 s
    //     if not fed — the 50 s k_msleep approach did not feed it at all)
    uint32_t grace_end_ms = k_uptime_get_32() + 10000u;
    while ((int32_t)(k_uptime_get_32() - grace_end_ms) < 0) {
        wdt_feed(wdt, ch);
        k_msleep(500);
    }

    LOG_INF("WDT: grace period over — monitoring plant_thread via g_wdt_sem");

    // Normal operation: plant_thread must give g_wdt_sem every ≤500 ms.
    // If it stalls, we log the fault and the 1 s HW timeout fires.
    while (true) {
        int r = k_sem_take(&g_wdt_sem, K_MSEC(500));
        if (r == 0) {
            wdt_feed(wdt, ch);
        } else {
            LOG_ERR("WDT: plant_thread stall detected — hardware reset imminent");
        }
    }
}

K_THREAD_DEFINE(watchdog_tid, 2048, watchdog_thread, NULL, NULL, NULL, 2, 0, 0);
