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
 * Reset the semaphore first, then check current level — if already HIGH, return
 * immediately without waiting for the next edge. */
static int wait_rdy(const struct device *dev, k_timeout_t timeout)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;

    k_sem_reset(&dat->rdy_sem);           /* drain any stale edge from previous txn */
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
    char ready_buf[16] = {};

    LOG_INF("CHIP_EN off (PE11 → physical LOW), BOOT → LOW, RDY=%d",
            gpio_pin_get_dt(&cfg->rdy));

    gpio_pin_set_dt(&cfg->chip_en, 1);   /* active-LOW → physical LOW → module off */
    k_msleep(100);

    LOG_INF("CHIP_EN on  (PE11 → physical HIGH) — releasing module");
    gpio_pin_set_dt(&cfg->chip_en, 0);   /* active-LOW → physical HIGH → module on */

    /* Poll RDY every 100 ms during boot window so we can see when it asserts */
    for (int i = 0; i < 20; i++) {
        k_msleep(100);
        int rdy = gpio_pin_get_dt(&cfg->rdy);
        if (rdy) {
            LOG_INF("RDY asserted at T+%d ms after CHIP_EN — module booted!", (i + 1) * 100);
            break;
        }
        if ((i + 1) % 5 == 0) {
            LOG_INF("Waiting for RDY... T+%d ms, RDY=%d", (i + 1) * 100, rdy);
        }
    }
    LOG_INF("RDY state after 2 s boot window: %d", gpio_pin_get_dt(&cfg->rdy));

    /* Module sends "\r\nready\r\n" spontaneously; read it via RX-only transaction */
    int rc = st67w61_spi_transact(dev, NULL, 0,
                                   (uint8_t *)ready_buf, sizeof(ready_buf) - 1,
                                   K_MSEC(10000));
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
    }

    /* ── RX phase ─────────────────────────────────────────────────────────── */

    /* Wait for module to assert RDY HIGH (rising edge = response ready).
     * For RX-only calls this is the module's spontaneous "ready" signal. */
    rc = wait_rdy(dev, timeout);
    if (rc) {
        LOG_WRN("SPI_RDY timeout — no response from module");
        return rc;
    }

    /* Assert CS and immediately clock — do NOT wait for RDY again.
     * Reference: NCP raises RDY → host raises CS → host generates clock. */
    gpio_pin_set_dt(cs, 1);

    /* Send proper empty header (magic + len=0 + reserved) while reading
     * the module's response header.  Must use magic bytes, not all-zeros. */
    static const uint8_t empty_hdr[ST67W61_HDR_LEN] = {0xAA, 0x55, 0, 0, 0, 0, 0, 0};
    uint8_t resp_hdr[ST67W61_HDR_LEN];
    struct spi_buf     tx_hdr_spi = { .buf = (void *)empty_hdr, .len = ST67W61_HDR_LEN };
    struct spi_buf     rx_hdr_spi = { .buf = resp_hdr,          .len = ST67W61_HDR_LEN };
    struct spi_buf_set tx_hdr_set = { .buffers = &tx_hdr_spi, .count = 1 };
    struct spi_buf_set rx_hdr_set = { .buffers = &rx_hdr_spi, .count = 1 };

    rc = spi_transceive(cfg->spi.bus, &no_cs, &tx_hdr_set, &rx_hdr_set);
    if (rc) {
        gpio_pin_set_dt(cs, 0);
        return rc;
    }

    uint16_t resp_len = (uint16_t)resp_hdr[2] | ((uint16_t)resp_hdr[3] << 8);

    if (resp_len == 0) {
        gpio_pin_set_dt(cs, 0);
        wait_rdy_low(dev);
        return 0;
    }

    uint16_t pad      = (uint16_t)((4 - (resp_len % 4)) % 4);
    uint16_t to_read  = resp_len + pad;
    uint16_t copy_len = (resp_len < resp_cap - 1) ? resp_len : (uint16_t)(resp_cap - 1);

    static uint8_t scratch[ST67W61_MAX_PAYLOAD + 4];
    static uint8_t dummy_pay[ST67W61_MAX_PAYLOAD + 4];

    if (to_read > sizeof(scratch)) to_read = (uint16_t)sizeof(scratch);

    struct spi_buf     rp_spi = { .buf = scratch,   .len = to_read };
    struct spi_buf     dp_spi = { .buf = dummy_pay, .len = to_read };
    struct spi_buf_set rp_set = { .buffers = &rp_spi, .count = 1 };
    struct spi_buf_set dp_set = { .buffers = &dp_spi, .count = 1 };

    rc = spi_transceive(cfg->spi.bus, &no_cs, &dp_set, &rp_set);
    gpio_pin_set_dt(cs, 0);

    if (rc) return rc;

    wait_rdy_low(dev);

    memcpy(resp_buf, scratch, copy_len);
    resp_buf[copy_len] = '\0';
    return (int)copy_len;
}
