// net_mgr.cpp — Wi-Fi L4 connection manager
//
// Registers a NET_EVENT_L4_CONNECTED callback; signals s_net_ready semaphore
// when the network stack reports the interface has a routable IP.
// The ST67W61 driver auto-connects using Kconfig SSID/password; this file
// only waits for the result and signals other threads.

#include <zephyr/kernel.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/logging/log.h>

#include "net_mgr.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static K_SEM_DEFINE(s_net_ready, 0, 1);
static struct net_mgmt_event_callback s_l4_cb;

static void on_l4_event(struct net_mgmt_event_callback* /*cb*/,
                         uint32_t event, struct net_if* /*iface*/)
{
    if (event == NET_EVENT_L4_CONNECTED) {
        LOG_INF("NET: L4 connected — IP ready");
        k_sem_give(&s_net_ready);
    } else if (event == NET_EVENT_L4_DISCONNECTED) {
        LOG_WRN("NET: L4 disconnected");
    }
}

void net_mgr_init()
{
    net_mgmt_init_event_callback(&s_l4_cb, on_l4_event,
        NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
    net_mgmt_add_event_callback(&s_l4_cb);
    LOG_INF("NET: waiting for Wi-Fi association...");
}

int net_mgr_wait_connected(k_timeout_t timeout)
{
    int rc = k_sem_take(&s_net_ready, timeout);
    if (rc == 0) return 0;
    LOG_WRN("NET: timed out waiting for L4 — continuing anyway");
    return -EAGAIN;
}
