// zephyr/src/http/page/http_page_plant.cpp
// Dashboard card: Plant State.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_plant_card(int fd, const plant::PlantState& s)
{
    char buf[512];

    send_str(fd, "<div class='card'><h2>Plant State</h2><table>");

    snprintf(buf, sizeof(buf),
        "<tr><td>vx</td><td id='s-vx'>%.3f m/s</td></tr>"
        "<tr><td>vy</td><td id='s-vy'>%.3f m/s</td></tr>"
        "<tr><td>yaw</td><td id='s-yaw'>%.2f &deg;</td></tr>"
        "<tr><td>yaw rate</td><td id='s-yawr'>%.2f &deg;/s</td></tr>"
        "<tr><td>x</td><td id='s-x'>%.2f m</td></tr>"
        "<tr><td>y</td><td id='s-y'>%.2f m</td></tr>",
        s.v_mps, s.vy_mps,
        s.yaw_rad * 57.2958, s.yaw_rate_radps * 57.2958,
        s.x_m, s.y_m);
    send_str(fd, buf);

    snprintf(buf, sizeof(buf),
        "<tr><td>steer</td><td id='s-steer'>%.2f &deg;</td></tr>"
        "<tr><td>SOC</td><td id='s-soc'><span class='val-hi'>%.1f %%</span></td></tr>"
        "<tr><td>&omega; FL</td><td id='s-ofl'>%.2f rad/s</td></tr>"
        "<tr><td>&omega; FR</td><td id='s-ofr'>%.2f rad/s</td></tr>"
        "<tr><td>&omega; RL</td><td id='s-orl'>%.2f rad/s</td></tr>"
        "<tr><td>&omega; RR</td><td id='s-orr'>%.2f rad/s</td></tr>"
        "<tr><td>surface &mu;</td><td id='s-mu'>%.2f</td></tr>",
        s.steer_virtual_rad * 57.2958,
        s.batt_soc_pct,
        s.omega_fl_radps, s.omega_fr_radps,
        s.omega_rl_radps, s.omega_rr_radps,
        s.surface_mu);
    send_str(fd, buf);

    send_str(fd, "</table></div>");
}
