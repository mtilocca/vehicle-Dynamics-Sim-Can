// zephyr/src/led/led_task.cpp
// Cycles the 3 user LEDs (LD1 green, LD2 yellow, LD3 red) through
// pseudo-random on/off patterns every 3 seconds.
//
// Priority 10 — below plant sim (Phase 4, prio 5), above shell (prio 14).
// At least one LED is always on (patterns 0b001 – 0b111, zero excluded).

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Device tree LED specs ─────────────────────────────────────────────────────
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

// ── XorShift32 PRNG — no stdlib needed ───────────────────────────────────────
static uint32_t xorshift32(uint32_t state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

// ── LED thread ────────────────────────────────────────────────────────────────
static void led_thread(void*, void*, void*)
{
    // Configure all three pins as outputs, start off
    if (!gpio_is_ready_dt(&led0) ||
        !gpio_is_ready_dt(&led1) ||
        !gpio_is_ready_dt(&led2)) {
        LOG_ERR("LED GPIO device not ready");
        return;
    }

    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);

    // Seed from uptime so pattern differs after each reset
    uint32_t rng = k_uptime_get_32() | 1u;

    while (true) {
        rng = xorshift32(rng);

        // Map to one of 7 non-zero 3-bit patterns (1–7)
        uint8_t pattern = (rng % 7u) + 1u;

        gpio_pin_set_dt(&led0, (pattern >> 0) & 1);
        gpio_pin_set_dt(&led1, (pattern >> 1) & 1);
        gpio_pin_set_dt(&led2, (pattern >> 2) & 1);

        k_msleep(3000);
    }
}

K_THREAD_DEFINE(led_tid, 512, led_thread, NULL, NULL, NULL, 12, 0, 0);
