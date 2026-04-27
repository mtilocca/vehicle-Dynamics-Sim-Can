/* st67w61_spi.c — SPI framing layer
 *
 * Protocol derived from stm32-hotspot/ST67W61-Bare-metal-implementation,
 * Middlewares/ST/ST67W6X_Minimal_Driver/ST67W611_NCP/spi_iface.c.
 *
 * Frame format (8-byte header + payload + 0x88 alignment padding):
 *   [0xAA][0x55][LEN_L][LEN_H][0x00][0x00][0x00][0x00] | payload | pad(0x88)
 *
 * Boot sequence (CRITICAL — must happen before any AT command):
 *   1. Host releases RESETN, waits 2 s.
 *   2. Module asserts RDY HIGH and sends "\r\nready\r\n" spontaneously.
 *   3. Host reads that message via an RX-only transaction (NULL tx).
 *   4. Host waits for RDY LOW (transaction complete).
 *
 * Per-command sequence (from spi_iface_command in reference):
 *   TX: wait RDY LOW (module idle) → assert CS → wait RDY HIGH → clock frame → deassert CS → wait RDY LOW
 *   RX: wait RDY HIGH (rising edge) → assert CS → clock empty header → parse resp_len → clock payload → deassert CS → wait RDY LOW
 *
 * RX-only (tx payload == NULL, used for "ready" + all response reads):
 *   Skips TX phase entirely; goes straight to RX phase.
 *
 * SPI_RDY: active-HIGH, GPIO_PULL_DOWN.
 * RESETN:  active-LOW  (OUTPUT_ACTIVE = physical LOW = in reset).
 */

#define DT_DRV_COMPAT st_st67w61
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include "st67w61.h"

LOG_MODULE_DECLARE(st67w61, CONFIG_ST67W61_LOG_LEVEL);

/* ── RDY helpers ─────────────────────────────────────────────────────────────── */

/* Rising-edge ISR — gives semaphore so wait_rdy never misses a short pulse. */
static void rdy_rising_isr(const struct device *port,
                            struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port); ARG_UNUSED(pins);
    struct st67w61_data *dat = CONTAINER_OF(cb, struct st67w61_data, rdy_cb);
    k_sem_give(&dat->rdy_sem);
}

/* Wait for SPI_RDY HIGH using interrupt + semaphore (never misses a short pulse).
 * Caller must k_sem_reset(&dat->rdy_sem) BEFORE the action that triggers the edge,
 * so a brief pulse that arrives before this call is already captured in the semaphore. */
static int wait_rdy(const struct device *dev, k_timeout_t timeout)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;

    if (gpio_pin_get_dt(&cfg->rdy)) return 0;  /* already HIGH */
    return k_sem_take(&dat->rdy_sem, timeout);
}

/* Wait for SPI_RDY LOW. Returns 0 or -ETIMEDOUT. */
static int wait_rdy_low_ms(const struct device *dev, uint32_t timeout_ms)
{
    const struct st67w61_config *cfg = dev->config;
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (gpio_pin_get_dt(&cfg->rdy)) {
        if (k_uptime_get() > deadline) return -ETIMEDOUT;
        k_sleep(K_MSEC(1));
    }
    return 0;
}

static void wait_rdy_low(const struct device *dev)
{
    wait_rdy_low_ms(dev, 500);
}

/* ── Frame builder ───────────────────────────────────────────────────────────── */

/* Build an SPI frame into dat->tx_buf.
 * Returns total frame length (header + payload + padding), or -ENOMEM. */
static int build_frame(struct st67w61_data *dat,
                        const uint8_t *payload, uint16_t len)
{
    uint16_t pad   = (uint16_t)((4 - (len % 4)) % 4);
    uint16_t total = ST67W61_HDR_LEN + len + pad;

    if (total > ST67W61_BUF_LEN) return -ENOMEM;

    uint8_t *h = dat->tx_buf;
    h[0] = 0xAA;
    h[1] = 0x55;
    h[2] = (uint8_t)(len & 0xFF);
    h[3] = (uint8_t)(len >> 8);
    h[4] = 0;
    h[5] = 0;
    h[6] = 0;
    h[7] = 0;

    if (payload && len) {
        memcpy(h + ST67W61_HDR_LEN, payload, len);
    }
    for (uint16_t i = 0; i < pad; i++) {
        h[ST67W61_HDR_LEN + len + i] = 0x88;
    }
    return (int)total;
}

/* ── Public API ──────────────────────────────────────────────────────────────── */

int st67w61_spi_init(const struct device *dev)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI bus not ready"); return -ENODEV;
    }
    if (!gpio_is_ready_dt(&cfg->chip_en)) {
        LOG_ERR("CHIP_EN GPIO not ready"); return -ENODEV;
    }
    if (!gpio_is_ready_dt(&cfg->boot)) {
        LOG_ERR("BOOT GPIO not ready"); return -ENODEV;
    }
    if (!gpio_is_ready_dt(&cfg->rdy)) {
        LOG_ERR("RDY GPIO not ready"); return -ENODEV;
    }

    /* CHIP_EN: active-LOW in DTS → OUTPUT_ACTIVE = physical LOW = module off */
    gpio_pin_configure_dt(&cfg->chip_en, GPIO_OUTPUT_ACTIVE);

    /* BOOT: drive LOW = normal SPI AT command mode.
     * If left floating and module has internal pull-up, HIGH = firmware update mode. */
    gpio_pin_configure_dt(&cfg->boot, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&cfg->rdy, GPIO_INPUT);

    /* Interrupt-based RDY detection — catches brief pulses that 1ms polling misses */
    k_sem_init(&dat->rdy_sem, 0, 1);
    gpio_init_callback(&dat->rdy_cb, rdy_rising_isr, BIT(cfg->rdy.pin));
    gpio_add_callback(cfg->rdy.port, &dat->rdy_cb);
    gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_EDGE_RISING);

    return 0;
}

void st67w61_spi_hw_reset(const struct device *dev)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;
    char ready_buf[32] = {};

    LOG_INF("HW reset: CHIP_EN → OFF");
    gpio_pin_set_dt(&cfg->chip_en, 1);   /* physical LOW = module off */
    k_msleep(200);

    /* Pre-arm semaphore BEFORE releasing CHIP_EN so the brief RDY boot pulse
     * (fires ~100 ms after power-on, shorter than one polling interval) is
     * captured in the semaphore before wait_rdy() checks it. */
    k_sem_reset(&dat->rdy_sem);
    LOG_INF("HW reset: CHIP_EN → ON — RDY=%d", gpio_pin_get_dt(&cfg->rdy));
    gpio_pin_set_dt(&cfg->chip_en, 0);   /* physical HIGH = module on */

    /* Module sends "\r\nready\r\n" spontaneously after boot.
     * wait_rdy() inside transact will catch the interrupt-captured edge. */
    int rc = st67w61_spi_transact(dev, NULL, 0,
                                   (uint8_t *)ready_buf, sizeof(ready_buf) - 1,
                                   K_SECONDS(8));
    if (rc < 0) {
        LOG_WRN("No ready message from module (rc=%d) — continuing", rc);
    } else {
        LOG_INF("ST67W61 booted — ready msg: '%.*s'", rc, ready_buf);
    }
}

int st67w61_spi_transact(const struct device *dev,
                          const uint8_t *payload, uint16_t tx_len,
                          uint8_t *resp_buf, uint16_t resp_cap,
                          k_timeout_t timeout)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;

    if (!cfg->spi.config.cs.gpio.port) {
        LOG_ERR("No CS GPIO in SPI config");
        return -EINVAL;
    }
    const struct gpio_dt_spec *cs = &cfg->spi.config.cs.gpio;
    struct spi_config no_cs = cfg->spi.config;
    no_cs.cs.gpio.port = NULL;  /* disable auto-CS — we drive it manually */

    int rc;
    uint32_t timeout_ms = (uint32_t)k_ticks_to_ms_near64(timeout.ticks);
    if (timeout_ms == 0) timeout_ms = 10000;

    /* ── TX phase (skipped for RX-only calls: payload == NULL) ──────────── */

    if (payload != NULL && tx_len > 0) {
        int frame_len = build_frame(dat, payload, tx_len);
        if (frame_len < 0) return frame_len;

        /* Reference spi_iface_command: wait for RDY LOW before asserting CS.
         * This ensures the module finished any previous transaction. */
        wait_rdy_low_ms(dev, timeout_ms);

        /* Arm semaphore BEFORE CS so the RDY HIGH pulse after CS is captured. */
        k_sem_reset(&dat->rdy_sem);
        gpio_pin_set_dt(cs, 1);  /* assert CS */

        /* Module drives RDY HIGH after CS to signal it can accept data. */
        rc = wait_rdy(dev, timeout);
        if (rc) {
            LOG_WRN("RDY timeout after CS assert (TX)");
            gpio_pin_set_dt(cs, 0);
            return rc;
        }

        struct spi_buf     tx_spi = { .buf = dat->tx_buf, .len = (size_t)frame_len };
        struct spi_buf     rx_spi = { .buf = dat->rx_buf, .len = (size_t)frame_len };
        struct spi_buf_set tx_set = { .buffers = &tx_spi, .count = 1 };
        struct spi_buf_set rx_set = { .buffers = &rx_spi, .count = 1 };

        rc = spi_transceive(cfg->spi.bus, &no_cs, &tx_set, &rx_set);
        gpio_pin_set_dt(cs, 0);

        if (rc) {
            LOG_ERR("spi_transceive (TX) failed: %d", rc);
            return rc;
        }

        wait_rdy_low(dev);  /* wait for RDY to settle after TX */
        /* Arm for the response RDY edge now that TX is fully complete. */
        k_sem_reset(&dat->rdy_sem);
    }

    /* ── RX phase ─────────────────────────────────────────────────────────── */

    /* Wait for module to assert RDY HIGH (rising edge = response ready).
     * For RX-only (boot): sem was pre-armed in hw_reset before CHIP_EN release.
     * For post-TX: sem was re-armed above after wait_rdy_low. */
    rc = wait_rdy(dev, timeout);
    if (rc) {
        LOG_WRN("SPI_RDY timeout — no response from module");
        return rc;
    }

    /* Assert CS and clock the full frame in ONE spi_transceive call.
     * On STM32H7, splitting into header-read + payload-read causes the SPI
     * peripheral to do LL_SPI_Disable → LL_SPI_Enable mid-CS, which briefly
     * releases SCK/MOSI and can corrupt the module's SPI slave byte count,
     * resulting in all-zero MISO for the payload portion. */
    gpio_pin_set_dt(cs, 1);

    /* Reuse dat->tx_buf as dummy TX: magic header bytes, rest 0. */
    memset(dat->tx_buf, 0, ST67W61_BUF_LEN);
    dat->tx_buf[0] = 0xAA; dat->tx_buf[1] = 0x55;

    static uint8_t rx_frame[ST67W61_BUF_LEN];
    struct spi_buf     tx_b = { .buf = dat->tx_buf, .len = ST67W61_BUF_LEN };
    struct spi_buf     rx_b = { .buf = rx_frame,    .len = ST67W61_BUF_LEN };
    struct spi_buf_set txs  = { .buffers = &tx_b, .count = 1 };
    struct spi_buf_set rxs  = { .buffers = &rx_b, .count = 1 };

    rc = spi_transceive(cfg->spi.bus, &no_cs, &txs, &rxs);
    gpio_pin_set_dt(cs, 0);
    wait_rdy_low(dev);

    if (rc) return rc;

    LOG_DBG("RX hdr: %02x %02x %02x %02x %02x %02x %02x %02x",
            rx_frame[0], rx_frame[1], rx_frame[2], rx_frame[3],
            rx_frame[4], rx_frame[5], rx_frame[6], rx_frame[7]);

    uint16_t resp_len = (uint16_t)rx_frame[2] | ((uint16_t)rx_frame[3] << 8);
    if (resp_len == 0) return 0;
    if (resp_len > ST67W61_MAX_PAYLOAD) {
        LOG_WRN("resp_len=%u exceeds MAX_PAYLOAD — discarding", resp_len);
        return -EMSGSIZE;
    }

    uint16_t copy_len = (resp_len < resp_cap - 1) ? resp_len : (uint16_t)(resp_cap - 1);
    memcpy(resp_buf, rx_frame + ST67W61_HDR_LEN, copy_len);
    resp_buf[copy_len] = '\0';
    return (int)copy_len;
}
