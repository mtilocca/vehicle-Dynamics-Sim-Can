/* st67w61_spi.c — SPI framing layer
 *
 * Adapted from stm32-hotspot/ST67W61-Bare-metal-implementation.
 * Original uses STM32 HAL SPI; this version uses Zephyr's spi_transceive_dt().
 *
 * Frame format (little-endian):
 *   [SYNC:2][SEQ:1][LEN:2][TYPE:1][RSVD:2][PAYLOAD:LEN]
 *
 * Before every TX the driver waits for RDY GPIO to be asserted by the module.
 * After TX the driver polls for the response frame the same way.
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

/* Wait for SPI_RDY GPIO (active-high) within timeout. */
static int wait_rdy(const struct device *dev, k_timeout_t timeout)
{
    const struct st67w61_config *cfg = dev->config;
    int64_t deadline = k_uptime_ticks() + k_ms_to_ticks_ceil64(
        k_ticks_to_ms_near64(timeout.ticks));

    while (!gpio_pin_get_dt(&cfg->rdy)) {
        if (k_uptime_ticks() > deadline) return -ETIMEDOUT;
        k_sleep(K_MSEC(1));
    }
    return 0;
}

static void build_frame(struct st67w61_data *data,
                         const uint8_t *payload, uint16_t len)
{
    uint8_t *h = data->tx_buf;
    /* SYNC word */
    h[0] = (uint8_t)(ST67W61_SYNC_WORD & 0xFF);
    h[1] = (uint8_t)(ST67W61_SYNC_WORD >> 8);
    /* Sequence number */
    h[2] = (uint8_t)(data->tx_seq++ & 0xFF);
    /* Payload length */
    h[3] = (uint8_t)(len & 0xFF);
    h[4] = (uint8_t)(len >> 8);
    /* Frame type: command */
    h[5] = ST67W61_TYPE_CMD;
    /* Reserved */
    h[6] = 0;
    h[7] = 0;
    if (payload && len) {
        memcpy(h + ST67W61_HDR_LEN, payload, len);
    }
}

int st67w61_spi_init(const struct device *dev)
{
    const struct st67w61_config *cfg = dev->config;

    if (!spi_is_ready_dt(&cfg->spi)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&cfg->resetn)) {
        LOG_ERR("RESETN GPIO not ready");
        return -ENODEV;
    }
    if (!gpio_is_ready_dt(&cfg->rdy)) {
        LOG_ERR("RDY GPIO not ready");
        return -ENODEV;
    }

    gpio_pin_configure_dt(&cfg->resetn, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&cfg->rdy,    GPIO_INPUT);

    /* Hard-reset the module: assert RESETN low ≥1 ms, then release */
    gpio_pin_set_dt(&cfg->resetn, 0);
    k_msleep(10);
    gpio_pin_set_dt(&cfg->resetn, 1);
    k_msleep(500);  /* module boot-up time after reset */

    LOG_INF("ST67W61 SPI transport ready");
    return 0;
}

int st67w61_spi_transact(const struct device *dev,
                          const uint8_t *payload, uint16_t tx_len,
                          uint8_t *resp_buf, uint16_t resp_cap,
                          k_timeout_t timeout)
{
    const struct st67w61_config *cfg = dev->config;
    struct st67w61_data         *dat = dev->data;
    int rc;

    /* Wait for module ready before sending */
    rc = wait_rdy(dev, timeout);
    if (rc) {
        LOG_ERR("SPI_RDY timeout before TX");
        return rc;
    }

    build_frame(dat, payload, tx_len);
    uint16_t frame_len = ST67W61_HDR_LEN + tx_len;

    struct spi_buf tx_spi = { .buf = dat->tx_buf, .len = frame_len };
    struct spi_buf rx_spi = { .buf = dat->rx_buf, .len = frame_len };
    struct spi_buf_set tx_set = { .buffers = &tx_spi, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_spi, .count = 1 };

    rc = spi_transceive_dt(&cfg->spi, &tx_set, &rx_set);
    if (rc) {
        LOG_ERR("spi_transceive_dt failed: %d", rc);
        return rc;
    }

    /* Wait for response */
    rc = wait_rdy(dev, timeout);
    if (rc) {
        LOG_WRN("SPI_RDY timeout waiting for response");
        return rc;
    }

    /* Read response header to get payload length */
    uint8_t resp_hdr[ST67W61_HDR_LEN] = {};
    struct spi_buf rh_spi = { .buf = resp_hdr, .len = ST67W61_HDR_LEN };
    struct spi_buf_set rh_set = { .buffers = &rh_spi, .count = 1 };

    /* Dummy TX while reading response */
    uint8_t dummy[ST67W61_HDR_LEN] = {};
    struct spi_buf dt_spi = { .buf = dummy, .len = ST67W61_HDR_LEN };
    struct spi_buf_set dt_set = { .buffers = &dt_spi, .count = 1 };

    rc = spi_transceive_dt(&cfg->spi, &dt_set, &rh_set);
    if (rc) return rc;

    uint16_t resp_len = (uint16_t)resp_hdr[3] | ((uint16_t)resp_hdr[4] << 8);
    if (resp_len == 0) return 0;
    if (resp_len > resp_cap - 1) resp_len = (uint16_t)(resp_cap - 1);

    struct spi_buf rp_spi = { .buf = resp_buf, .len = resp_len };
    struct spi_buf_set rp_set = { .buffers = &rp_spi, .count = 1 };
    uint8_t dummy2[ST67W61_MAX_PAYLOAD] = {};
    struct spi_buf dp_spi = { .buf = dummy2, .len = resp_len };
    struct spi_buf_set dp_set = { .buffers = &dp_spi, .count = 1 };

    rc = spi_transceive_dt(&cfg->spi, &dp_set, &rp_set);
    if (rc) return rc;

    resp_buf[resp_len] = '\0';
    return (int)resp_len;
}
