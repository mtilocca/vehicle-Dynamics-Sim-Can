/* st67w61_wifi.c — Zephyr WiFi management + net_if registration
 *
 * Registers the ST67W61 as a Zephyr WiFi offload device using the same
 * NET_DEVICE_DT_INST_OFFLOAD_DEFINE + CONNECTIVITY_WIFI_MGMT_BIND pattern
 * as the eswifi driver (drivers/wifi/eswifi/eswifi_core.c).
 *
 * The driver handles Wi-Fi L2 (scan/connect/disconnect).
 * TCP socket offload (open/send/recv) is implemented in st67w61_sock.c
 * (TODO: add when the AT socket command set is confirmed from UM3475).
 */

#define DT_DRV_COMPAT st_st67w61
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include "st67w61.h"

LOG_MODULE_REGISTER(st67w61, CONFIG_ST67W61_LOG_LEVEL);

/* ── Driver config / data instances ─────────────────────────────────────────── */

static const struct st67w61_config st67w61_cfg_0 = {
    .spi    = SPI_DT_SPEC_INST_GET(0,
                SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),
    .resetn = GPIO_DT_SPEC_INST_GET(0, resetn_gpios),
    .rdy    = GPIO_DT_SPEC_INST_GET(0, rdy_gpios),
};

static struct st67w61_data st67w61_data_0;

/* ── Wi-Fi connection work handler ───────────────────────────────────────────── */

static void connect_work_handler(struct k_work *work)
{
    struct st67w61_data *dat = CONTAINER_OF(work, struct st67w61_data, connect_work);
    const struct device *dev = net_if_get_device(dat->iface);
    int rc;

    rc = st67w61_at_connect(dev, dat->ssid, dat->psk);
    if (rc) {
        LOG_ERR("Wi-Fi connect failed: %d", rc);
        wifi_mgmt_raise_connect_result_event(dat->iface, rc);
        return;
    }

    dat->connected = true;
    LOG_INF("Wi-Fi connected to \"%s\"", dat->ssid);
    wifi_mgmt_raise_connect_result_event(dat->iface, 0);
}

/* ── WiFi management ops ─────────────────────────────────────────────────────── */

static int st67w61_mgmt_connect(const struct device *dev,
                                 struct wifi_connect_req_params *params)
{
    struct st67w61_data *dat = dev->data;

    if (params->ssid_length > WIFI_SSID_MAX_LEN) return -EINVAL;
    if (params->psk_length  > 64)                return -EINVAL;

    k_mutex_lock(&dat->mutex, K_FOREVER);
    memcpy(dat->ssid, params->ssid, params->ssid_length);
    dat->ssid[params->ssid_length] = '\0';
    if (params->psk && params->psk_length) {
        memcpy(dat->psk, params->psk, params->psk_length);
        dat->psk[params->psk_length] = '\0';
    } else {
        dat->psk[0] = '\0';
    }
    dat->security = params->security;
    k_mutex_unlock(&dat->mutex);

    k_work_submit(&dat->connect_work);
    return 0;
}

static int st67w61_mgmt_disconnect(const struct device *dev)
{
    struct st67w61_data *dat = dev->data;
    int rc = st67w61_at_disconnect(dev);
    if (rc == 0) {
        dat->connected = false;
        wifi_mgmt_raise_disconnect_result_event(dat->iface, 0);
    }
    return rc;
}

static int st67w61_mgmt_scan(const struct device *dev,
                               struct wifi_scan_params *params,
                               scan_result_cb_t cb)
{
    /* TODO: implement AT+WFSCAN and parse results */
    ARG_UNUSED(dev); ARG_UNUSED(params); ARG_UNUSED(cb);
    LOG_WRN("Wi-Fi scan not yet implemented");
    return -ENOTSUP;
}

static int st67w61_mgmt_iface_status(const struct device *dev,
                                      struct wifi_iface_status *status)
{
    struct st67w61_data *dat = dev->data;
    memset(status, 0, sizeof(*status));
    if (dat->connected) {
        status->state = WIFI_STATE_COMPLETED;
        strncpy((char *)status->ssid, dat->ssid, sizeof(status->ssid) - 1);
        status->ssid_len = (uint8_t)strlen(dat->ssid);
        status->rssi     = -70;  /* TODO: query via AT+WFSTAT */
    } else {
        status->state = WIFI_STATE_DISCONNECTED;
    }
    return 0;
}

static const struct wifi_mgmt_ops st67w61_mgmt_api = {
    .scan         = st67w61_mgmt_scan,
    .connect      = st67w61_mgmt_connect,
    .disconnect   = st67w61_mgmt_disconnect,
    .iface_status = st67w61_mgmt_iface_status,
};

/* ── net_if callbacks ────────────────────────────────────────────────────────── */

static void st67w61_iface_init(struct net_if *iface)
{
    const struct device  *dev = net_if_get_device(iface);
    struct st67w61_data  *dat = dev->data;

    dat->iface = iface;
    net_if_set_link_addr(iface, dat->mac, sizeof(dat->mac), NET_LINK_ETHERNET);

    LOG_INF("ST67W61 net_if init — MAC %02X:%02X:%02X:%02X:%02X:%02X",
            dat->mac[0], dat->mac[1], dat->mac[2],
            dat->mac[3], dat->mac[4], dat->mac[5]);
}

/* ── Driver init ─────────────────────────────────────────────────────────────── */

static int st67w61_init(const struct device *dev)
{
    struct st67w61_data *dat = dev->data;
    int rc;

    k_mutex_init(&dat->mutex);
    k_work_init(&dat->connect_work, connect_work_handler);

    /* Bring up SPI + hard-reset module */
    rc = st67w61_spi_init(dev);
    if (rc) return rc;

    /* Verify AT comms and read MAC */
    rc = st67w61_at_init(dev);
    if (rc) return rc;

    rc = st67w61_at_get_mac(dev, dat->mac);
    if (rc) return rc;

    /* Auto-connect on boot using Kconfig credentials */
    if (strlen(CONFIG_ST67W61_SSID) > 0) {
        strncpy(dat->ssid, CONFIG_ST67W61_SSID, sizeof(dat->ssid) - 1);
        strncpy(dat->psk,  CONFIG_ST67W61_PASSWORD, sizeof(dat->psk) - 1);
        dat->security = WIFI_SECURITY_TYPE_PSK;
        k_work_submit(&dat->connect_work);
    }

    return 0;
}

/* ── Zephyr device + net_if registration ─────────────────────────────────────── */

static const struct net_wifi_mgmt_offload st67w61_offload_api = {
    .wifi_iface.iface_api.init = st67w61_iface_init,
    .wifi_mgmt_api             = &st67w61_mgmt_api,
};

NET_DEVICE_DT_INST_OFFLOAD_DEFINE(0,
    st67w61_init, NULL,
    &st67w61_data_0, &st67w61_cfg_0,
    CONFIG_WIFI_INIT_PRIORITY,
    &st67w61_offload_api,
    1500);

CONNECTIVITY_WIFI_MGMT_BIND(Z_DEVICE_DT_DEV_ID(DT_DRV_INST(0)));
