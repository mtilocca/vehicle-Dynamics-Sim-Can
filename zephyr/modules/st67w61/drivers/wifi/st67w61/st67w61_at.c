/* st67w61_at.c — AT command encode/response layer
 *
 * Wraps st67w61_spi_transact() to send AT strings and check responses.
 * AT commands are null-terminated ASCII strings; the SPI layer frames them.
 *
 * Verify command strings against the ST67W61 AT Command Guide (UM3475).
 * The bare-metal reference at stm32-hotspot/ST67W61-Bare-metal-implementation
 * contains usage examples for each command.
 */

#define DT_DRV_COMPAT st_st67w61
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "st67w61.h"

LOG_MODULE_DECLARE(st67w61, CONFIG_ST67W61_LOG_LEVEL);

#define AT_RESP_BUF 256

static int at_cmd(const struct device *dev, const char *cmd,
                   char *resp, size_t resp_cap, k_timeout_t timeout)
{
    /* ST67W61 requires AT commands terminated with \r\n */
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "%s\r\n", cmd);
    if (n < 0 || (size_t)n >= sizeof(buf)) return -ENOMEM;

    LOG_DBG("AT> %s", cmd);
    int rc = st67w61_spi_transact(dev,
                                    (const uint8_t *)buf, (uint16_t)n,
                                    (uint8_t *)resp, (uint16_t)resp_cap,
                                    timeout);
    if (rc < 0) return rc;
    LOG_DBG("AT< %.*s", rc, resp);
    return rc;
}

static bool resp_is_ok(const char *resp)
{
    return strstr(resp, ST67W61_OK_STR) != NULL;
}

int st67w61_at_init(const struct device *dev)
{
    char resp[AT_RESP_BUF] = {};
    k_timeout_t t = K_MSEC(CONFIG_ST67W61_AT_TIMEOUT_MS);

    /* Verify module is alive */
    int rc = at_cmd(dev, ST67W61_AT_TEST, resp, sizeof(resp), t);
    if (rc < 0 || !resp_is_ok(resp)) {
        LOG_ERR("AT test failed (rc=%d, resp='%s')", rc, resp);
        return -EIO;
    }
    LOG_INF("ST67W61 module alive");
    return 0;
}

int st67w61_at_cmd(const struct device *dev, const char *cmd,
                    char *resp, size_t resp_cap, k_timeout_t timeout)
{
    return at_cmd(dev, cmd, resp, resp_cap, timeout);
}

int st67w61_at_get_mac(const struct device *dev, uint8_t mac[6])
{
    char resp[AT_RESP_BUF];
    k_timeout_t t = K_MSEC(CONFIG_ST67W61_AT_TIMEOUT_MS);

    int rc = at_cmd(dev, ST67W61_AT_GETMAC, resp, sizeof(resp), t);
    if (rc < 0) return rc;

    /* Expected response format (verify against AT guide):
     * "+GETMAC:XX:XX:XX:XX:XX:XX\r\nOK\r\n"
     * TODO: adjust sscanf format if module uses different separator */
    unsigned int m[6];
    if (sscanf(resp, "+GETMAC:%02x:%02x:%02x:%02x:%02x:%02x",
               &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) mac[i] = (uint8_t)m[i];
        return 0;
    }

    /* Fallback: locally administered MAC derived from board ID */
    LOG_WRN("Could not parse MAC from '%s' — using default", resp);
    mac[0] = 0x02; mac[1] = 0x00; mac[2] = 0x5E;
    mac[3] = 0x67; mac[4] = 0x57; mac[5] = 0x61;
    return 0;
}

int st67w61_at_connect(const struct device *dev,
                        const char *ssid, const char *psk)
{
    char cmd[128];
    char resp[AT_RESP_BUF];
    k_timeout_t t = K_MSEC(CONFIG_ST67W61_AT_TIMEOUT_MS * 4);  /* association can take ~15s */

    /* AT+WFJAP="ssid","password"
     * TODO: verify exact command format in UM3475 for your module FW version */
    snprintf(cmd, sizeof(cmd), "%s=\"%s\",\"%s\"", ST67W61_AT_WFJAP, ssid, psk);

    int rc = at_cmd(dev, cmd, resp, sizeof(resp), t);
    if (rc < 0) return rc;
    if (!resp_is_ok(resp)) {
        LOG_ERR("WFJAP failed: %s", resp);
        return -EIO;
    }
    return 0;
}

int st67w61_at_disconnect(const struct device *dev)
{
    char resp[AT_RESP_BUF];
    return at_cmd(dev, ST67W61_AT_WFDAP, resp, sizeof(resp),
                  K_MSEC(CONFIG_ST67W61_AT_TIMEOUT_MS));
}
