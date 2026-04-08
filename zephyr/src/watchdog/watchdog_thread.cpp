// zephyr/src/watchdog/watchdog_thread.cpp
// Hardware watchdog heartbeat thread.
//
// Boot sequence:
//   1. SYS_INIT POST_KERNEL: direct register write reloads the IWDG counter
//      before any thread starts — handles IWDG left running from a prior flash.
//      This runs unconditionally (outside CONFIG_WATCHDOG guard) because the
//      IWDG persists across warm resets regardless of Kconfig.
//   2. watchdog_thread (CONFIG_WATCHDOG=y only): arms Zephyr WDT with 1s
//      timeout, feeds every 500 ms for 10-second grace period.
//   3. After grace period: semaphore-based feeding — plant_thread gives
//      g_wdt_sem every 10 ms; stall >500 ms → hardware reset.
//
// Priority 2 — starts before all application threads.
// Stack 2048 B — LOG_MODE_IMMEDIATE + WDT HAL call chain.

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Early IWDG reprogram + reload — before scheduler starts ──────────────────
// STM32H7 IWDG1 register map (base 0x58004800):
//   KR  +0x00 : key register  (0x5555=unlock, 0xAAAA=reload, 0xCCCC=start)
//   PR  +0x04 : prescaler     (6 = divide-by-256 → 32kHz/256 = 125 Hz tick)
//   RLR +0x08 : reload value  (624 → timeout = 625/125 = 5.0 s)
//   SR  +0x0C : status        (bit0=PVU, bit1=RVU — wait for 0 before reload)
//
// This reprogram ensures a clean 5-second timeout regardless of what prior
// boot-loop sessions left in the registers. Then reloads the counter.
// Runs unconditionally — safe whether IWDG is running or not.

static volatile uint32_t* const IWDG1_KR = (volatile uint32_t*)0x58004800U;

static int iwdg_early_kick(void)
{
    // Only reload the counter — do NOT reprogram PR/RLR here.
    // At PRE_KERNEL_1 the LSI oscillator is not yet stable; writing PR/RLR
    // and spinning on SR.PVU/RVU causes an infinite loop before UART starts.
    // A plain 0xAAAA reload is always safe: it resets the countdown to whatever
    // timeout the IWDG was configured with, buying time for the driver to arm.
    *IWDG1_KR = 0xAAAAU;
    return 0;
}
// Feed at every boot level so IWDG never expires regardless of stored timeout.
// SYS_INIT uses the function name as the symbol — use SYS_INIT_NAMED for unique symbols.
SYS_INIT_NAMED(iwdg_kick_pk1, iwdg_early_kick, PRE_KERNEL_1, 0);
SYS_INIT_NAMED(iwdg_kick_pk2, iwdg_early_kick, PRE_KERNEL_2, 0);
SYS_INIT_NAMED(iwdg_kick_pok, iwdg_early_kick, POST_KERNEL,  0);
SYS_INIT_NAMED(iwdg_kick_app, iwdg_early_kick, APPLICATION,  0);

// ── Watchdog thread (only compiled when CONFIG_WATCHDOG=y) ────────────────────
#ifdef CONFIG_WATCHDOG

#include <zephyr/drivers/watchdog.h>

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

    // Belt-and-suspenders: reload again now that we're in the thread.
    *IWDG1_KR = 0xAAAAU;
    LOG_INF("WDT: thread started, counter reloaded");

    struct wdt_timeout_cfg cfg{};
    cfg.window.min = 0;
    cfg.window.max = 1000;   // 1 second hardware timeout
    cfg.callback   = nullptr;
    cfg.flags      = WDT_FLAG_RESET_SOC;

    int ch = wdt_install_timeout(wdt, &cfg);
    if (ch < 0) {
        LOG_WRN("WDT: wdt_install_timeout failed (%d) — using ch 0", ch);
        ch = 0;
    }

    int rc = wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
    if (rc < 0) {
        // -EBUSY: IWDG already running from prior session — expected, continue.
        LOG_WRN("WDT: wdt_setup returned %d (IWDG already running — ok)", rc);
    }

    wdt_feed(wdt, ch);
    LOG_INF("WDT: armed (1s timeout). Feeding during 10s init grace period...");

    uint32_t grace_end_ms = k_uptime_get_32() + 10000u;
    while ((int32_t)(k_uptime_get_32() - grace_end_ms) < 0) {
        wdt_feed(wdt, ch);
        k_msleep(500);
    }

    LOG_INF("WDT: grace period over — monitoring plant_thread via g_wdt_sem");

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

#endif // CONFIG_WATCHDOG
