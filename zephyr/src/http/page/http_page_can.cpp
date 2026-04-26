// zephyr/src/http/page/http_page_can.cpp
// Dashboard card: CAN Stats.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_can_card(int fd, const sim::ActuatorCmd& c)
{
    char buf[320];

    send_str(fd, "<div class='card'><h2>CAN Stats</h2><table>");

    snprintf(buf, sizeof(buf),
        "<tr><td>TX frames</td><td id='can-tx'>%u</td></tr>"
        "<tr><td>RX frames</td><td id='can-rx'>%u</td></tr>"
        "<tr><td>Timeouts</td>"
        "<td id='can-to'><span class='%s'>%u</span></td></tr>"
        "<tr><td>Last RX</td><td id='can-lrx'>%.3f s</td></tr>",
        (uint32_t)atomic_get(&g_ctrl_bus.can_tx_count),
        (uint32_t)atomic_get(&g_ctrl_bus.can_rx_count),
        atomic_get(&g_ctrl_bus.can_timeout_count) ? "val-warn" : "",
        (uint32_t)atomic_get(&g_ctrl_bus.can_timeout_count),
        c.last_update_t_s);
    send_str(fd, buf);

    send_str(fd, "</table></div>");
}
