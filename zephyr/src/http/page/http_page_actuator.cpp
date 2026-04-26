// zephyr/src/http/page/http_page_actuator.cpp
// Dashboard card: Actuator Command.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_actuator_card(int fd, const sim::ActuatorCmd& c)
{
    char buf[320];

    send_str(fd, "<div class='card'><h2>Actuator Command</h2><table>");

    snprintf(buf, sizeof(buf),
        "<tr><td>Enable</td>"
        "<td id='c-en'><span class='%s'>%s</span></td></tr>"
        "<tr><td>Gear</td><td id='c-gear'>%s</td></tr>"
        "<tr><td>Torque</td><td id='c-torq'>%.1f Nm</td></tr>"
        "<tr><td>Brake</td><td id='c-brk'>%.2f %%</td></tr>"
        "<tr><td>Steer</td><td id='c-steer'>%.2f &deg;</td></tr>",
        c.system_enable ? "val-hi" : "val-warn",
        c.system_enable ? "ON" : "OFF",
        c.gear_position == sim::GearPosition::FORWARD ? "FORWARD" :
        c.gear_position == sim::GearPosition::REVERSE ? "REVERSE" : "NEUTRAL",
        c.drive_torque_cmd_nm, c.brake_cmd_pct, c.steer_cmd_deg);
    send_str(fd, buf);

    send_str(fd, "</table></div>");
}
