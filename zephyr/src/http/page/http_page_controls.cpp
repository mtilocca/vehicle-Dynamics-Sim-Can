// zephyr/src/http/page/http_page_controls.cpp
// Dashboard card: quick-action buttons + manual inject form.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_controls_card(int fd, const sim::ActuatorCmd& c)
{
    char buf[512];

    double sp5 = c.steer_cmd_deg + 5.0;
    double sm5 = c.steer_cmd_deg - 5.0;
    if (sp5 >  45.0) sp5 =  45.0;
    if (sm5 < -45.0) sm5 = -45.0;

    send_str(fd, "<div class='card'><h2>Controls</h2>");
    send_str(fd, "<div class='ctrl-row'>");
    send_str(fd,
        "<a class='btn btn-stop' href='/dash?enable=1&gear=N&torque=0&brake=100&steer=0'>"
        "&#9632;&nbsp;STOP</a>");
    send_str(fd,
        "<a class='btn btn-fwd' href='/dash?enable=1&gear=F&torque=50000&brake=0&steer=0'>"
        "&#9654;&nbsp;Drive FWD</a>");
    send_str(fd,
        "<a class='btn btn-rev' href='/dash?enable=1&gear=R&torque=50000&brake=0&steer=0'>"
        "&#9664;&nbsp;Drive REV</a>");
    snprintf(buf, sizeof(buf),
        "<a class='btn btn-steer' href='/dash?steer=%.0f'>&#8592;&nbsp;%.0f&deg;</a>"
        "<a class='btn btn-steer' href='/dash?steer=%.0f'>%.0f&deg;&nbsp;&#8594;</a>",
        sm5, sm5, sp5, sp5);
    send_str(fd, buf);
    send_str(fd, "</div>");

    // Manual inject form
    const char* sel_f = (c.gear_position == sim::GearPosition::FORWARD) ? " selected" : "";
    const char* sel_n = (c.gear_position == sim::GearPosition::NEUTRAL) ? " selected" : "";
    const char* sel_r = (c.gear_position == sim::GearPosition::REVERSE) ? " selected" : "";

    send_str(fd, "<form method='get' action='/dash' style='margin-top:10px'>");
    send_str(fd, "<div class='ctrl-row'>");

    snprintf(buf, sizeof(buf),
        "<label>Steer&nbsp;&deg;</label>"
        "<input type='number' name='steer' min='-45' max='45' step='1' value='%.0f'>",
        c.steer_cmd_deg);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<label>Torque&nbsp;Nm</label>"
        "<input type='number' name='torque' min='0' max='145000' step='5000' value='%.0f'>",
        c.drive_torque_cmd_nm);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<label>Brake&nbsp;%%</label>"
        "<input type='number' name='brake' min='0' max='100' step='5' value='%.0f'>",
        c.brake_cmd_pct);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<label>Gear</label>"
        "<select name='gear'>"
        "<option value='F'%s>FORWARD</option>"
        "<option value='N'%s>NEUTRAL</option>"
        "<option value='R'%s>REVERSE</option>"
        "</select>",
        sel_f, sel_n, sel_r);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<label><input type='checkbox' name='enable' value='1'%s>Enable</label>",
        c.system_enable ? " checked" : "");
    send_str(fd, buf);

    send_str(fd, "</div>");
    send_str(fd, "<div class='ctrl-row' style='margin-top:6px'>");
    send_str(fd,
        "<button class='btn btn-inject' type='submit'>Inject Command &#8594;</button>"
        "<span class='meta'>&nbsp;watchdog: resend within 500&nbsp;ms to hold</span>");
    send_str(fd, "</div></form></div>");
}
