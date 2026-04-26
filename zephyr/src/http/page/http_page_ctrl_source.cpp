// zephyr/src/http/page/http_page_ctrl_source.cpp
// Dashboard card: Control Source toggle (CAN / MQTT / HTTP).

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_ctrl_source_card(int fd)
{
    char buf[400];
    int src = (int)atomic_get(&g_ctrl_bus.ctrl_source);
    const char* ac = "btn btn-active";
    const char* in = "btn";

    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>Control Source</h2>"
        "<div class='ctrl-row'>"
        "<a class='%s' href='/dash?ctrl=can' >CAN</a>"
        "<a class='%s' href='/dash?ctrl=mqtt'>MQTT</a>"
        "<a class='%s' href='/dash?ctrl=http'>HTTP</a>"
        "</div>"
        "<p style='margin:6px 0 0;font-size:12px;color:#8b949e'>"
        "Active source writes g_cmd. Others decode but discard.</p>"
        "</div>",
        (src == CTRL_CAN)  ? ac : in,
        (src == CTRL_MQTT) ? ac : in,
        (src == CTRL_HTTP) ? ac : in);
    send_str(fd, buf);
}
