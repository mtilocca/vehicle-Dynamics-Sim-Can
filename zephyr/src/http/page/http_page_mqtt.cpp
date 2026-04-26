// zephyr/src/http/page/http_page_mqtt.cpp
// Dashboard card: MQTT broker address form + connection status.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_mqtt_card(int fd)
{
    char buf[512];

    const auto& bcfg = mqtt::broker_config();
    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>MQTT Broker</h2>"
        "<form method='get' action='/dash'>"
        "<div class='ctrl-row'>"
        "<label>IP&nbsp;"
        "<input type='text' name='broker_addr' value='%s'"
        " style='width:140px'></label>"
        "<label>Port&nbsp;"
        "<input type='number' name='broker_port' value='%d' min='1' max='65535'"
        " style='width:70px'></label>"
        "<button class='btn' type='submit'>Apply</button>"
        "</div></form>",
        bcfg.addr, bcfg.port);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<p style='margin:6px 0 0;font-size:12px'>"
        "Status:&nbsp;<span id='mqtt-st'><span class='%s'>%s</span></span>"
        "&nbsp;&nbsp;MQTT&nbsp;RX:&nbsp;<span id='mqtt-rx'>%u</span></p>"
        "</div>",
        g_mqtt_connected ? "val-hi" : "val-warn",
        g_mqtt_connected ? "connected" : "disconnected",
        (uint32_t)atomic_get(&g_ctrl_bus.mqtt_rx_count));
    send_str(fd, buf);
}
