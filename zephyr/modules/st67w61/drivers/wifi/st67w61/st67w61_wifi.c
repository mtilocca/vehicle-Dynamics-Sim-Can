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
#include <zephyr/net/net_offload.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/conn_mgr/connectivity_wifi_mgmt.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <errno.h>

#include "st67w61.h"

LOG_MODULE_REGISTER(st67w61, CONFIG_ST67W61_LOG_LEVEL);

/* Dedicated 4 KB work queue — sysworkq default (~1 KB) is too small for the
 * combined stack depth of hw_init + at_init + spi_transact. */
K_THREAD_STACK_DEFINE(st67w61_wq_stack, 4096);
static struct k_work_q st67w61_wq;

/* ── Driver config / data instances ─────────────────────────────────────────── */

static const struct st67w61_config st67w61_cfg_0 = {
    .spi     = SPI_DT_SPEC_INST_GET(0,
                 SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),
    .chip_en = GPIO_DT_SPEC_INST_GET(0, chip_en_gpios),
    .boot    = GPIO_DT_SPEC_INST_GET(0, boot_gpios),
    .rdy     = GPIO_DT_SPEC_INST_GET(0, rdy_gpios),
};

static struct st67w61_data st67w61_data_0;

/* ── Deferred hardware init (runs in system work queue after threads start) ───── */

static void hw_init_work_handler(struct k_work *work)
{
    struct st67w61_data *dat = CONTAINER_OF(work, struct st67w61_data, hw_init_work);
    const struct device *dev = net_if_get_device(dat->iface);
    int rc;

    LOG_INF("HW init: starting SPI reset + boot drain");
    int64_t t0 = k_uptime_get();
    st67w61_spi_hw_reset(dev);
    LOG_INF("HW init: reset+drain done (%lld ms)", k_uptime_get() - t0);

    LOG_INF("HW init: running AT init");
    int64_t t_at = k_uptime_get();
    rc = st67w61_at_init(dev);
    LOG_INF("HW init: AT init %s (%lld ms)", rc ? "FAILED" : "OK", k_uptime_get() - t_at);
    if (rc) {
        LOG_ERR("ST67W61 AT init failed: %d", rc);
        return;
    }

    LOG_INF("HW init: reading MAC");
    rc = st67w61_at_get_mac(dev, dat->mac);
    if (rc) {
        LOG_ERR("ST67W61 MAC read failed: %d", rc);
        return;
    }

    /* Update link address with real MAC now that we have it */
    net_if_set_link_addr(dat->iface, dat->mac, sizeof(dat->mac), NET_LINK_ETHERNET);
    LOG_INF("ST67W61 ready — MAC %02X:%02X:%02X:%02X:%02X:%02X (total init %lld ms)",
            dat->mac[0], dat->mac[1], dat->mac[2],
            dat->mac[3], dat->mac[4], dat->mac[5],
            k_uptime_get() - t0);

    dat->hw_ready = true;

    /* Auto-connect on boot if SSID is configured */
    if (strlen(CONFIG_ST67W61_SSID) > 0) {
        LOG_INF("HW init: auto-connect to SSID '%s'", CONFIG_ST67W61_SSID);
        strncpy(dat->ssid, CONFIG_ST67W61_SSID, sizeof(dat->ssid) - 1);
        strncpy(dat->psk,  CONFIG_ST67W61_PASSWORD, sizeof(dat->psk) - 1);
        dat->security = WIFI_SECURITY_TYPE_PSK;
        k_work_submit_to_queue(&st67w61_wq, &dat->connect_work);
    } else {
        LOG_WRN("HW init: no SSID configured — skipping auto-connect");
    }
}

/* ── Wi-Fi connection work handler ───────────────────────────────────────────── */

static void connect_work_handler(struct k_work *work)
{
    struct st67w61_data *dat = CONTAINER_OF(work, struct st67w61_data, connect_work);
    const struct device *dev = net_if_get_device(dat->iface);
    int rc;

    if (!dat->hw_ready) {
        LOG_WRN("Wi-Fi connect: hardware not ready — aborting");
        return;
    }

    LOG_INF("Wi-Fi connect: SSID='%s' security=%d psk_len=%zu",
            dat->ssid, dat->security, strlen(dat->psk));
    int64_t t0 = k_uptime_get();

    rc = st67w61_at_connect(dev, dat->ssid, dat->psk);
    int64_t elapsed = k_uptime_get() - t0;
    if (rc) {
        LOG_ERR("Wi-Fi connect FAILED: rc=%d (%lld ms)", rc, elapsed);
        wifi_mgmt_raise_connect_result_event(dat->iface, rc);
        return;
    }

    dat->connected = true;
    LOG_INF("Wi-Fi connected: SSID='%s' in %lld ms — carrier ON", dat->ssid, elapsed);
    net_if_carrier_on(dat->iface);
    wifi_mgmt_raise_connect_result_event(dat->iface, 0);
}

/* ── net_offload stubs ───────────────────────────────────────────────────────── */
/* Setting iface->if_dev->offload to a non-NULL pointer makes net_if_is_offloaded()
 * return true, which prevents notify_iface_up() from calling iface_ipv6_start() /
 * iface_ipv4_start() — those would queue packets through the NULL OFFLOADED_NETDEV
 * L2 send pointer and crash.  Full TCP socket offload (AT+NSTCP etc.) is TODO. */

static int off_get(sa_family_t family, enum net_sock_type type,
                   enum net_ip_protocol ip_proto, struct net_context **context)
{
    ARG_UNUSED(family); ARG_UNUSED(type);
    ARG_UNUSED(ip_proto); ARG_UNUSED(context);
    return -ENOTSUP;
}

static int off_bind(struct net_context *context,
                    const struct sockaddr *addr, socklen_t addrlen)
{
    ARG_UNUSED(context); ARG_UNUSED(addr); ARG_UNUSED(addrlen);
    return -ENOTSUP;
}

static int off_listen(struct net_context *context, int backlog)
{
    ARG_UNUSED(context); ARG_UNUSED(backlog);
    return -ENOTSUP;
}

static int off_connect(struct net_context *context,
                       const struct sockaddr *addr, socklen_t addrlen,
                       net_context_connect_cb_t cb, int32_t timeout,
                       void *user_data)
{
    ARG_UNUSED(context); ARG_UNUSED(addr); ARG_UNUSED(addrlen);
    ARG_UNUSED(cb); ARG_UNUSED(timeout); ARG_UNUSED(user_data);
    return -ENOTSUP;
}

static int off_accept(struct net_context *context,
                      net_tcp_accept_cb_t cb, int32_t timeout, void *user_data)
{
    ARG_UNUSED(context); ARG_UNUSED(cb);
    ARG_UNUSED(timeout); ARG_UNUSED(user_data);
    return -ENOTSUP;
}

static int off_send(struct net_pkt *pkt, net_context_send_cb_t cb,
                    int32_t timeout, void *user_data)
{
    ARG_UNUSED(pkt); ARG_UNUSED(cb);
    ARG_UNUSED(timeout); ARG_UNUSED(user_data);
    return -ENOTSUP;
}

static int off_sendto(struct net_pkt *pkt,
                      const struct sockaddr *dst_addr, socklen_t addrlen,
                      net_context_send_cb_t cb, int32_t timeout,
                      void *user_data)
{
    ARG_UNUSED(pkt); ARG_UNUSED(dst_addr); ARG_UNUSED(addrlen);
    ARG_UNUSED(cb); ARG_UNUSED(timeout); ARG_UNUSED(user_data);
    return -ENOTSUP;
}

static int off_recv(struct net_context *context, net_context_recv_cb_t cb,
                    int32_t timeout, void *user_data)
{
    ARG_UNUSED(context); ARG_UNUSED(cb);
    ARG_UNUSED(timeout); ARG_UNUSED(user_data);
    return -ENOTSUP;
}

static int off_put(struct net_context *context)
{
    ARG_UNUSED(context);
    return -ENOTSUP;
}

static struct net_offload st67w61_net_offload = {
    .get     = off_get,
    .bind    = off_bind,
    .listen  = off_listen,
    .connect = off_connect,
    .accept  = off_accept,
    .send    = off_send,
    .sendto  = off_sendto,
    .recv    = off_recv,
    .put     = off_put,
};

/* ── WiFi management ops ─────────────────────────────────────────────────────── */

static int st67w61_mgmt_connect(const struct device *dev,
                                 struct wifi_connect_req_params *params)
{
    struct st67w61_data *dat = dev->data;

    if (params->ssid_length > WIFI_SSID_MAX_LEN) return -EINVAL;
    if (params->psk_length  > 64)                return -EINVAL;

    LOG_INF("mgmt_connect: SSID='%.*s' security=%d hw_ready=%d",
            params->ssid_length, params->ssid, params->security, (int)dat->hw_ready);

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

    k_work_submit_to_queue(&st67w61_wq, &dat->connect_work);
    return 0;
}

static int st67w61_mgmt_disconnect(const struct device *dev)
{
    struct st67w61_data *dat = dev->data;
    LOG_INF("mgmt_disconnect: was_connected=%d SSID='%s'",
            (int)dat->connected, dat->ssid);
    int rc = st67w61_at_disconnect(dev);
    if (rc == 0) {
        dat->connected = false;
        net_if_carrier_off(dat->iface);
        wifi_mgmt_raise_disconnect_result_event(dat->iface, 0);
        LOG_INF("mgmt_disconnect: carrier OFF");
    } else {
        LOG_ERR("mgmt_disconnect: AT+WFDAP failed rc=%d", rc);
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

    /* NET_IF_OFFLOAD_INIT creates NET_IF_MAX_CONFIGS entries sharing one if_dev;
     * net_if_init() calls api->init for each entry.  Only initialize on first call. */
    if (dat->iface != NULL) {
        return;
    }

    dat->iface = iface;
    /* MAC is zeros until hw_init_work reads it; placeholder keeps net_if happy */
    net_if_set_link_addr(iface, dat->mac, sizeof(dat->mac), NET_LINK_ETHERNET);

    /* Register offload ops so net_if_is_offloaded() returns true.
     * Without this, notify_iface_up() calls iface_ipv6_start() which queues
     * an RS packet — net_if_tx then calls the NULL OFFLOADED_NETDEV L2 send
     * and crashes (PC=0x00000000, USAGE FAULT). */
    iface->if_dev->offload = &st67w61_net_offload;

    /* Carrier is off until Wi-Fi associates.  NET_IF_OFFLOAD_INIT sets
     * NET_IF_LOWER_UP by default; clear it now so no TX is attempted
     * before the link is actually up. */
    net_if_carrier_off(iface);

    /* Kick off the blocking hardware init now that iface is wired up */
    k_work_submit_to_queue(&st67w61_wq, &dat->hw_init_work);
}

/* ── Driver init (POST_KERNEL — must be non-blocking, no sleeps) ─────────────── */

static int st67w61_init(const struct device *dev)
{
    struct st67w61_data *dat = dev->data;
    int rc;

    k_mutex_init(&dat->mutex);
    k_work_init(&dat->connect_work, connect_work_handler);
    k_work_init(&dat->hw_init_work,  hw_init_work_handler);

    k_work_queue_start(&st67w61_wq, st67w61_wq_stack,
                       K_THREAD_STACK_SIZEOF(st67w61_wq_stack),
                       8, NULL);  /* preemptible prio 8 — between CAN RX (6) and HTTP (10) */

    /* Configure SPI bus and GPIO directions (fast — no delays) */
    rc = st67w61_spi_init(dev);
    if (rc) return rc;

    /* hw_init_work is submitted from st67w61_iface_init once iface is wired up */
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
