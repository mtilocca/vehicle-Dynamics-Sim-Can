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
#include <soc.h>  /* SPI_TypeDef, SPI_CFG2_IOSWP (bit 15) — STM32H7 CMSIS header */

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

    /* BOOT: configurable via CONFIG_ST67W61_BOOT_LEVEL_FOR_AT.
     * Most variants want LOW = normal AT mode, HIGH = FW update mode.
     * If 01-55 garbage persists across all SPI mode sweeps, flip this
     * to 1 to test inverted polarity on this HAT. */
    gpio_pin_configure_dt(&cfg->boot,
        CONFIG_ST67W61_BOOT_LEVEL_FOR_AT ? GPIO_OUTPUT_ACTIVE : GPIO_OUTPUT_INACTIVE);
    LOG_INF("BOOT pin: %s (CONFIG_ST67W61_BOOT_LEVEL_FOR_AT=%d)",
            CONFIG_ST67W61_BOOT_LEVEL_FOR_AT ? "HIGH (FW-update)" : "LOW (AT mode)",
            CONFIG_ST67W61_BOOT_LEVEL_FOR_AT);

    gpio_pin_configure_dt(&cfg->rdy, GPIO_INPUT);

    /* CS GPIO: the Zephyr SPI driver only configures the CS pin when it manages it
     * directly (non-NULL cs in spi_config).  We always pass no_cs so the driver
     * never touches PD14 — without this explicit configure the pin stays in its
     * power-on reset state (input/analog) and gpio_pin_set_dt has no effect. */
    if (cfg->spi.config.cs.gpio.port) {
        gpio_pin_configure_dt(&cfg->spi.config.cs.gpio, GPIO_OUTPUT_INACTIVE);
        LOG_INF("CS: %s pin %d — configured OUTPUT_INACTIVE (deselected)",
                cfg->spi.config.cs.gpio.port->name,
                cfg->spi.config.cs.gpio.pin);
    } else {
        LOG_ERR("CS GPIO not present in SPI config");
        return -EINVAL;
    }

    /* Interrupt-based RDY detection — catches brief pulses that 1ms polling misses */
    k_sem_init(&dat->rdy_sem, 0, 1);
    gpio_init_callback(&dat->rdy_cb, rdy_rising_isr, BIT(cfg->rdy.pin));
    gpio_add_callback(cfg->rdy.port, &dat->rdy_cb);
    gpio_pin_interrupt_configure_dt(&cfg->rdy, GPIO_INT_EDGE_RISING);

    /* Cache a no-CS copy of the SPI config with a stable pointer.
     * Zephyr SPI driver skips reconfiguration when the same pointer is passed
     * to consecutive spi_transceive calls.  A stack-allocated copy creates a new
     * address every call, forcing a full re-configure that rewrites CFG2 bits
     * (CPOL, CPHA, NSS mode) — this does not clear IOSWP (bit 15 is untouched by
     * those MODIFY_REG operations), but the pointer stability is still good practice. */
    dat->spi_no_cs_cfg = cfg->spi.config;
    dat->spi_no_cs_cfg.cs.gpio.port = NULL;

    /* Approach A — runtime mode/framing override (Kconfig-driven).  The DTS
     * has no spi-cpol/spi-cpha/spi-lsb-first properties, so the operation
     * field comes in as Mode 0 / MSB.  Apply CPOL/CPHA/LSB choices here. */
    dat->spi_no_cs_cfg.operation &= ~(SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_LSB);
    dat->spi_no_cs_cfg.operation |= SPI_TRANSFER_MSB;
    if (IS_ENABLED(CONFIG_ST67W61_SPI_CPOL)) {
        dat->spi_no_cs_cfg.operation |= SPI_MODE_CPOL;
    }
    if (IS_ENABLED(CONFIG_ST67W61_SPI_CPHA)) {
        dat->spi_no_cs_cfg.operation |= SPI_MODE_CPHA;
    }
    if (IS_ENABLED(CONFIG_ST67W61_SPI_LSB_FIRST)) {
        dat->spi_no_cs_cfg.operation &= ~SPI_TRANSFER_MSB;
        dat->spi_no_cs_cfg.operation |= SPI_TRANSFER_LSB;
    }
    LOG_INF("SPI cfg: freq=%u Hz CPOL=%d CPHA=%d %s dummy=0x%02x",
            dat->spi_no_cs_cfg.frequency,
            IS_ENABLED(CONFIG_ST67W61_SPI_CPOL),
            IS_ENABLED(CONFIG_ST67W61_SPI_CPHA),
            IS_ENABLED(CONFIG_ST67W61_SPI_LSB_FIRST) ? "LSB" : "MSB",
            (unsigned)CONFIG_ST67W61_RX_DUMMY_BYTE);

    /* IOSWP tested (H4) and confirmed NOT the issue — H4 ruled out.
     * All-zeros TX confirmed module drives MISO independently (not crosstalk).
     * With IOSWP set, MISO went all-zeros (PB5 undriven), proving D12/PA6 is
     * the module's actual MISO output (standard convention, no HAT wiring swap). */

    return 0;
}

void st67w61_spi_hw_reset(const struct device *dev)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;
    char ready_buf[64] = {};

    LOG_INF("HW reset: CS raw=%d RDY=%d — CHIP_EN → OFF (hold %u ms)",
            gpio_pin_get_raw(cfg->spi.config.cs.gpio.port, cfg->spi.config.cs.gpio.pin),
            gpio_pin_get_dt(&cfg->rdy),
            (unsigned)CONFIG_ST67W61_CHIP_EN_OFF_HOLD_MS);
    gpio_pin_set_dt(&cfg->chip_en, 1);   /* physical LOW = module off */
    k_msleep(50);   /* short delay before BASELINE diagnostic */

    /* Baseline SPI transfer with module OFF — MISO should be all-zero (pull-down).
     * If we get non-zero here, the pattern comes from SPI hw, not the module. */
    {
        const struct gpio_dt_spec *cs = &cfg->spi.config.cs.gpio;
        static uint8_t _b_tx[8] = { 0xAA, 0x55, 0, 0, 0, 0, 0, 0 };
        static uint8_t _b_rx[8] = {};
        struct spi_buf _btx = { .buf = _b_tx, .len = 8 };
        struct spi_buf _brx = { .buf = _b_rx, .len = 8 };
        struct spi_buf_set _btxs = { .buffers = &_btx, .count = 1 };
        struct spi_buf_set _brxs = { .buffers = &_brx, .count = 1 };
        gpio_pin_set_raw(cs->port, cs->pin, 1);
        k_busy_wait(CONFIG_ST67W61_CS_SETUP_US);
        spi_transceive(cfg->spi.bus, &dat->spi_no_cs_cfg, &_btxs, &_brxs);
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        LOG_INF("BASELINE (module OFF): %02x %02x %02x %02x %02x %02x %02x %02x",
                _b_rx[0], _b_rx[1], _b_rx[2], _b_rx[3],
                _b_rx[4], _b_rx[5], _b_rx[6], _b_rx[7]);
    }

    /* Remaining OFF-state hold to reach total CONFIG_ST67W61_CHIP_EN_OFF_HOLD_MS.
     * We've already consumed 50 ms + ~10 ms BASELINE; subtract that from the
     * target so the user-set value reflects total off-time. */
    if (CONFIG_ST67W61_CHIP_EN_OFF_HOLD_MS > 60) {
        k_msleep(CONFIG_ST67W61_CHIP_EN_OFF_HOLD_MS - 60);
    }

    /* Pre-arm semaphore BEFORE releasing CHIP_EN so the brief RDY boot pulse
     * (fires ~640 ms after power-on) is captured before wait_rdy() checks it. */
    k_sem_reset(&dat->rdy_sem);
    LOG_INF("HW reset: CHIP_EN → ON — RDY=%d", gpio_pin_get_dt(&cfg->rdy));
    gpio_pin_set_dt(&cfg->chip_en, 0);   /* physical HIGH = module on */

    /* Drain all spontaneous boot frames until RDY is stably LOW.
     * First iteration uses 4 s (module asserts RDY ~640 ms after CHIP_EN).
     * Subsequent iterations use 2 s for any follow-up frames. */
    for (int i = 0; i < 8; i++) {
        k_timeout_t t = (i == 0) ? K_SECONDS(4) : K_MSEC(2000);
        int rc = st67w61_spi_transact(dev, NULL, 0,
                                       (uint8_t *)ready_buf, sizeof(ready_buf) - 1,
                                       t);
        if (rc < 0) {
            LOG_INF("Boot drain[%d]: timeout/err (rc=%d) — done", i, rc);
            break;
        }
        LOG_INF("Boot drain[%d]: len=%d '%.*s' (raw[0]=0x%02x)",
                i, rc, rc, ready_buf, (uint8_t)ready_buf[0]);

        /* Wait up to 5 s to see if RDY goes HIGH again (second boot frame).
         * The module may send capabilities frame first, then \r\nready\r\n. */
        bool rdy_again = false;
        int64_t deadline = k_uptime_get() + 5000;
        while (k_uptime_get() < deadline) {
            if (gpio_pin_get_dt(&cfg->rdy)) { rdy_again = true; break; }
            k_msleep(10);
        }
        LOG_INF("Boot drain[%d]: RDY after 300ms hold-off = %d", i, (int)rdy_again);

        if (!rdy_again) {
            LOG_INF("RDY stable LOW after frame %d — module ready", i);
            break;
        }
        /* RDY went HIGH again — another frame pending; arm semaphore */
        k_sem_reset(&dat->rdy_sem);
    }

    /* Allow the module firmware 500 ms to finish any internal post-boot work
     * before we start sending AT commands. */
    LOG_INF("Boot drain complete — settling 500 ms");
    k_msleep(500);
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
    /* Use the cached no-CS config (stable pointer — set once in spi_init).
     * A stack copy would create a new address each call, causing the Zephyr
     * driver to re-run spi_stm32_configure on every TX transfer. */
    struct spi_config *no_cs = &dat->spi_no_cs_cfg;

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
        LOG_DBG("TX: pre-CS RDY=%d", gpio_pin_get_dt(&cfg->rdy));

        /* Arm semaphore BEFORE CS so the RDY HIGH pulse after CS is captured. */
        k_sem_reset(&dat->rdy_sem);

        /* Use gpio_pin_set_raw to drive CS physically HIGH regardless of any
         * polarity flag ambiguity. Module selects on CS=physical HIGH. */
        gpio_pin_set_raw(cs->port, cs->pin, 1);
        /* H7 software CS has zero CS-to-SCK setup; without this delay the
         * module's MISO driver is still in hi-Z when the first SCK fires
         * and the first byte reads back as a floating-line sample. */
        k_busy_wait(CONFIG_ST67W61_CS_SETUP_US);
        LOG_DBG("TX: CS pin raw=%d RDY=%d",
                gpio_pin_get_raw(cs->port, cs->pin),
                gpio_pin_get_dt(&cfg->rdy));

        /* Module drives RDY HIGH to signal it can accept data.
         * Wait up to 500 ms (module may need time after CS assertion). */
        rc = wait_rdy(dev, K_MSEC(500));
        if (rc) {
            LOG_WRN("TX: no RDY HIGH after CS in 500 ms (RDY=%d) — blind send",
                    gpio_pin_get_dt(&cfg->rdy));
            k_msleep(1);
        }

        struct spi_buf     tx_spi = { .buf = dat->tx_buf, .len = (size_t)frame_len };
        struct spi_buf     rx_spi = { .buf = dat->rx_buf, .len = (size_t)frame_len };
        struct spi_buf_set tx_set = { .buffers = &tx_spi, .count = 1 };
        struct spi_buf_set rx_set = { .buffers = &rx_spi, .count = 1 };

        rc = spi_transceive(cfg->spi.bus, no_cs, &tx_set, &rx_set);
        gpio_pin_set_raw(cs->port, cs->pin, 0);  /* CS physically LOW = deselect */

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
        if (IS_ENABLED(CONFIG_ST67W61_FORCE_RX_ON_NORDY)) {
            LOG_WRN("SPI_RDY timeout — force-reading 32 bytes anyway (diagnostic)");
            /* fall through to RX clocking */
        } else {
            LOG_WRN("SPI_RDY timeout — no response from module");
            return rc;
        }
    }

    LOG_DBG("RX: RDY HIGH — asserting CS for two-phase transfer");

    /* Two-phase transfer matching reference spi_iface_txRx():
     *   Phase 1: clock 32 bytes (8 header + 24 payload lookahead for diagnostics)
     *   Parse resp_len from bytes [2:3]; payload bytes [8..31] already captured
     *   Phase 2: only if resp_len > 24 — clock remaining payload bytes
     * CS is manually asserted/deasserted around both phases combined.
     * Using no_cs config for all calls so the driver never auto-toggles CS. */

    /* TX dummy for RX-only: filled with CONFIG_ST67W61_RX_DUMMY_BYTE so we
     * can sweep the matrix (0xFF canonical idle, 0x00 quiet, 0xAA/0x55 to
     * probe MOSI→MISO coupling). One-time init; refilled if Kconfig changed
     * since the previous call (cheap: a single memset of 1036 B). */
    static uint8_t tx_dummy[ST67W61_BUF_LEN];
    static uint8_t tx_dummy_fill;
    static uint8_t rx_hdr[32];
    static uint8_t rx_payload[ST67W61_MAX_PAYLOAD + 4];

    if (tx_dummy_fill != (uint8_t)CONFIG_ST67W61_RX_DUMMY_BYTE) {
        memset(tx_dummy, CONFIG_ST67W61_RX_DUMMY_BYTE, sizeof(tx_dummy));
        tx_dummy_fill = (uint8_t)CONFIG_ST67W61_RX_DUMMY_BYTE;
    }

    /* Assert CS manually — reference does set_cs(1) before HAL_SPI_TransmitReceive.
     * gpio_pin_set_raw avoids any GPIO_ACTIVE_HIGH polarity translation. */
    gpio_pin_set_raw(cs->port, cs->pin, 1);
    k_busy_wait(CONFIG_ST67W61_CS_SETUP_US);  /* see TX phase note */
    LOG_DBG("RX: CS pin raw=1 RDY=%d", gpio_pin_get_dt(&cfg->rdy));

    /* Phase 1: exchange 32 bytes — captures full boot preamble for diagnostics. */
    memset(rx_hdr, 0, sizeof(rx_hdr));
    struct spi_buf     h_tx_b = { .buf = tx_dummy,  .len = sizeof(rx_hdr) };
    struct spi_buf     h_rx_b = { .buf = rx_hdr,    .len = sizeof(rx_hdr) };
    struct spi_buf_set h_txs  = { .buffers = &h_tx_b, .count = 1 };
    struct spi_buf_set h_rxs  = { .buffers = &h_rx_b, .count = 1 };

    rc = spi_transceive(cfg->spi.bus, no_cs, &h_txs, &h_rxs);
    if (rc) {
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        LOG_ERR("spi_transceive (RX hdr) failed: %d", rc);
        return rc;
    }

    /* Always log first 16 bytes at INF so they appear regardless of log level. */
    LOG_INF("RX[0..7]:  %02x %02x %02x %02x %02x %02x %02x %02x",
            rx_hdr[0], rx_hdr[1], rx_hdr[2], rx_hdr[3],
            rx_hdr[4], rx_hdr[5], rx_hdr[6], rx_hdr[7]);
    LOG_INF("RX[8..15]: %02x %02x %02x %02x %02x %02x %02x %02x",
            rx_hdr[8], rx_hdr[9], rx_hdr[10], rx_hdr[11],
            rx_hdr[12], rx_hdr[13], rx_hdr[14], rx_hdr[15]);
    LOG_INF("RX[16..23]:%02x %02x %02x %02x %02x %02x %02x %02x",
            rx_hdr[16], rx_hdr[17], rx_hdr[18], rx_hdr[19],
            rx_hdr[20], rx_hdr[21], rx_hdr[22], rx_hdr[23]);
    LOG_INF("RX[24..31]:%02x %02x %02x %02x %02x %02x %02x %02x",
            rx_hdr[24], rx_hdr[25], rx_hdr[26], rx_hdr[27],
            rx_hdr[28], rx_hdr[29], rx_hdr[30], rx_hdr[31]);

    /* Parse header from first 8 bytes. */
    uint16_t resp_len = (uint16_t)rx_hdr[2] | ((uint16_t)rx_hdr[3] << 8);

    if (rx_hdr[0] != 0xAA || rx_hdr[1] != 0x55) {
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        wait_rdy_low(dev);
        LOG_WRN("RX: bad magic %02x %02x resp_len=%u — treating as empty",
                rx_hdr[0], rx_hdr[1], resp_len);
        return 0;
    }

    if (resp_len == 0) {
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        wait_rdy_low(dev);
        return 0;
    }
    if (resp_len > ST67W61_MAX_PAYLOAD) {
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        wait_rdy_low(dev);
        LOG_WRN("resp_len=%u OOB — discarding", resp_len);
        return -EMSGSIZE;
    }

    /* Phase 1 already clocked (sizeof(rx_hdr) - ST67W61_HDR_LEN) = 24 extra bytes.
     * If the full payload fits in those 24 bytes, skip Phase 2. */
    uint16_t already = (uint16_t)(sizeof(rx_hdr) - ST67W61_HDR_LEN);  /* 24 */

    if (resp_len <= already) {
        gpio_pin_set_raw(cs->port, cs->pin, 0);
        wait_rdy_low(dev);
        uint16_t copy_len = (resp_len < resp_cap - 1) ? resp_len : (uint16_t)(resp_cap - 1);
        memcpy(resp_buf, rx_hdr + ST67W61_HDR_LEN, copy_len);
        resp_buf[copy_len] = '\0';
        LOG_DBG("RX payload (from hdr buf, %u bytes): '%.*s'", copy_len, copy_len, resp_buf);
        return (int)copy_len;
    }

    /* Phase 2: payload doesn't fit in Phase 1 window — clock the remaining bytes. */
    uint16_t pad     = (uint16_t)((4 - (resp_len % 4)) % 4);
    uint16_t total   = resp_len + pad;
    uint16_t pay_len = (uint16_t)(total - already);

    struct spi_buf     p_tx_b = { .buf = tx_dummy + sizeof(rx_hdr), .len = pay_len };
    struct spi_buf     p_rx_b = { .buf = rx_payload,                .len = pay_len };
    struct spi_buf_set p_txs  = { .buffers = &p_tx_b, .count = 1 };
    struct spi_buf_set p_rxs  = { .buffers = &p_rx_b, .count = 1 };

    rc = spi_transceive(cfg->spi.bus, no_cs, &p_txs, &p_rxs);
    gpio_pin_set_raw(cs->port, cs->pin, 0);
    wait_rdy_low(dev);

    if (rc) {
        LOG_ERR("spi_transceive (RX payload) failed: %d", rc);
        return rc;
    }

    /* Assemble response: first 'already' bytes from rx_hdr, rest from rx_payload */
    uint16_t copy_len = (resp_len < resp_cap - 1) ? resp_len : (uint16_t)(resp_cap - 1);
    uint16_t from_hdr = (already < copy_len) ? already : copy_len;
    memcpy(resp_buf, rx_hdr + ST67W61_HDR_LEN, from_hdr);
    if (copy_len > from_hdr) {
        memcpy(resp_buf + from_hdr, rx_payload, copy_len - from_hdr);
    }
    resp_buf[copy_len] = '\0';
    return (int)copy_len;
}
