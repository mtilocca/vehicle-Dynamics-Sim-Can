// zephyr/src/http/page/http_page_root.cpp
// Dashboard page assembler — snapshots state once, delegates to card builders.

#include <zephyr/logging/log.h>
#include "http_html.hpp"
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// CSS embedded from dashboard.css at build time via generate_inc_file_for_target()
static const char kDashboardCss[] = {
#include "dashboard.css.inc"
    '\0'
};

// ── Low-level send helper ─────────────────────────────────────────────────────

void send_str(int fd, const char* s)
{
    int len  = (int)strlen(s);
    int sent = 0;
    while (sent < len) {
        int r = zsock_send(fd, s + sent, len - sent, 0);
        if (r <= 0) return;
        sent += r;
    }
}

// ── Dashboard page ────────────────────────────────────────────────────────────

namespace http {

void send_page(int fd)
{
    char buf[512];

    send_str(fd, kHtmlHead);
    send_str(fd, kDashboardCss);
    send_str(fd, kHtmlStyleClose);

    // Snapshot shared state once — eliminates per-card mutex lock/unlock.
    plant::PlantState s{};
    sim::ActuatorCmd  c{};
    { hdv::MutexGuard g(g_sim_plant_mtx); s = g_sim_bus.plant; }
    { hdv::MutexGuard g(g_sim_cmd_mtx);  c = g_sim_bus.cmd;  }

    // Header bar
    uint32_t ms     = k_uptime_get_32();
    uint32_t sec_up = ms / 1000;
    uint32_t h      = sec_up / 3600;
    uint32_t m      = (sec_up % 3600) / 60;
    uint32_t sc     = sec_up % 60;

    send_str(fd,
        "<div class='header'>"
        "<h1>Heavy-Duty Electric Vehicle &mdash; Simulator Dashboard</h1>"
        "<span class='badge'>&#9679; ONLINE</span>");
    snprintf(buf, sizeof(buf),
        "<span class='meta'>Uptime&nbsp;<span id='up'>%02u:%02u:%02u</span></span>",
        h, m, sc);
    send_str(fd, buf);
    send_str(fd,
        "<span class='meta'>IP&nbsp;192.168.1.80</span>"
        "<span class='meta'>MAC&nbsp;02:00:5E:00:53:01</span>"
        "<div style='margin-left:auto;display:flex;gap:8px;align-items:center'>"
        "<a href='/ota' style='color:#8b949e;font-size:12px;"
        "text-decoration:none;border:1px solid #30363d;padding:2px 8px;border-radius:4px;'>"
        "OTA</a>"
        "<a href='/logout' style='color:#8b949e;font-size:12px;"
        "text-decoration:none;border:1px solid #30363d;padding:2px 8px;border-radius:4px;'>"
        "Logout</a>"
        "</div>"
        "</div>");

    // Two-column grid: plant state (left) + actuator/CAN (right)
    send_str(fd, "<div class='grid'>");
    send_plant_card(fd, s);
    send_str(fd, "<div style='display:flex;flex-direction:column;gap:12px'>");
    send_actuator_card(fd, c);
    send_can_card(fd, c);
    send_str(fd, "</div>");  // right column
    send_str(fd, "</div>");  // grid

    send_controls_card(fd, c);
    send_ctrl_source_card(fd);
    send_mqtt_card(fd);
    send_resources_card(fd);
    send_threads_card(fd);   // must precede send_memory_card
    send_memory_card(fd);

    send_str(fd, kVehicleCard);
    send_str(fd, kHtmlFoot);
}

} // namespace http
