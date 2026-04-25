// zephyr/src/http/http_page.cpp
// HTML dashboard page builder.
// Assembles the full HTTP response (header + HTML) and writes it to the
// client socket via send_str(). Also provides the kernel-threads card.

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>   /* snprintf */
#include <string.h>  /* strlen */

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "http_html.hpp"
#include "stats/sys_stats.hpp"
#include "mqtt/mqtt_client.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// CSS embedded from dashboard.css at build time via generate_inc_file_for_target()
static const char kDashboardCss[] = {
#include "dashboard.css.inc"
    '\0'
};

// ── Shared globals (defined in main.cpp) ─────────────────────────────────────
extern plant::PlantState    g_state;
extern sim::ActuatorCmd     g_cmd;
extern struct k_mutex       g_state_mutex;
extern struct k_mutex       g_cmd_mutex;
extern atomic_t             g_can_tx_count;
extern atomic_t             g_can_rx_count;
extern atomic_t             g_can_timeout_count;
extern double               g_surface_mu;
extern SysStats             g_sys_stats;
extern struct k_mutex       g_stats_mutex;
extern atomic_t             g_ctrl_source;
extern atomic_t             g_mqtt_rx_count;

// ── Low-level send helper ─────────────────────────────────────────────────────

static void send_str(int fd, const char* s)
{
    int len  = (int)strlen(s);
    int sent = 0;
    while (sent < len) {
        int r = zsock_send(fd, s + sent, len - sent, 0);
        if (r <= 0) return;
        sent += r;
    }
}

// ── System resources card ─────────────────────────────────────────────────────

static void send_resources_card(int fd)
{
    SysStats st{};
    k_mutex_lock(&g_stats_mutex, K_FOREVER);
    st = g_sys_stats;
    k_mutex_unlock(&g_stats_mutex);

    const size_t heap_total = CONFIG_HEAP_MEM_POOL_SIZE;
    int heap_pct = (heap_total > 0 && st.heap_used > 0)
                   ? (int)(100u * st.heap_used / heap_total) : 0;
    const char* heap_cls = (heap_pct >= 80) ? "val-warn"
                         : (heap_pct >= 60) ? ""
                         : "val-hi";

    // plant_loop_us_max in tenths of ms for one-decimal display
    int loop_tenths = (int)(st.plant_loop_us_max / 100);
    const char* loop_cls = (st.plant_loop_us_max > 10000) ? "val-warn" : "val-hi";

    char buf[512];
    snprintf(buf, sizeof(buf),
        "<div class='card'><h2>System Resources</h2><table>"
        "<tr><td>Heap used</td>"
        "<td id='res-hu'><span class='%s'>%zu&nbsp;B / %zu&nbsp;B&nbsp;(%d%%)</span></td></tr>"
        "<tr><td>Heap free</td><td id='res-hf'>%zu&nbsp;B</td></tr>"
        "<tr><td>Sim loop&nbsp;max</td>"
        "<td id='res-loop'><span class='%s'>%d.%d&nbsp;ms</span>"
        "&nbsp;<span style='color:#8b949e'>(budget&nbsp;10&nbsp;ms)</span></td></tr>"
        "</table></div>",
        heap_cls, st.heap_used, heap_total, heap_pct,
        st.heap_free,
        loop_cls, loop_tenths / 10, loop_tenths % 10);
    send_str(fd, buf);
}

// ── Kernel threads card ───────────────────────────────────────────────────────

struct ThreadInfo {
    char   name[24];
    int    priority;
    size_t stack_used;
    size_t stack_total;
};

static ThreadInfo s_threads[20];
static int        s_thread_count;

static void collect_thread_cb(const struct k_thread* t, void*)
{
    if (s_thread_count >= 20) return;
    ThreadInfo& info = s_threads[s_thread_count++];

    const char* n = k_thread_name_get((k_tid_t)t);
    strncpy(info.name, (n && n[0]) ? n : "?", sizeof(info.name) - 1);
    info.name[sizeof(info.name) - 1] = '\0';

    info.priority    = k_thread_priority_get((k_tid_t)t);
    info.stack_total = t->stack_info.size;

    size_t unused = 0;
    k_thread_stack_space_get(t, &unused);
    info.stack_used = (info.stack_total > unused) ? (info.stack_total - unused) : 0;
}

static void send_threads_card(int fd)
{
    char buf[1024];

    s_thread_count = 0;
    k_thread_foreach(collect_thread_cb, nullptr);

    send_str(fd,
        "<div class='card'>"
        "<h2>Kernel Threads</h2>"
        "<table>"
        "<tr style='color:#8b949e;font-size:12px'>"
        "<td>Name</td><td>Prio</td><td style='width:40%'>Stack used / total</td><td>%</td>"
        "</tr>"
    );

    for (int i = 0; i < s_thread_count; ++i) {
        const ThreadInfo& ti = s_threads[i];
        int pct = (ti.stack_total > 0)
                  ? (int)(100u * ti.stack_used / ti.stack_total) : 0;
        const char* cls = (pct >= 80) ? "val-warn" : (pct >= 60) ? "" : "val-hi";

        snprintf(buf, sizeof(buf),
            "<tr><td>%.23s</td><td>%d</td>"
            "<td><span class='%.8s'>%zu</span> / %zu B</td>"
            "<td class='%.8s'>%d%%</td></tr>",
            ti.name, ti.priority,
            cls, ti.stack_used, ti.stack_total,
            cls, pct);
        send_str(fd, buf);
    }

    send_str(fd, "</table></div>");
}

// ── /api/state — JSON telemetry snapshot ─────────────────────────────────────

void send_api_state(int fd)
{
    plant::PlantState s{};
    sim::ActuatorCmd  c{};
    SysStats st{};

    k_mutex_lock(&g_state_mutex, K_FOREVER);  s = g_state;        k_mutex_unlock(&g_state_mutex);
    k_mutex_lock(&g_cmd_mutex,   K_FOREVER);  c = g_cmd;          k_mutex_unlock(&g_cmd_mutex);
    k_mutex_lock(&g_stats_mutex, K_FOREVER);  st = g_sys_stats;   k_mutex_unlock(&g_stats_mutex);

    uint32_t uptime_s = k_uptime_get_32() / 1000;
    int src = (int)atomic_get(&g_ctrl_source);
    const char* gear_str =
        (c.gear_position == sim::GearPosition::FORWARD) ? "F" :
        (c.gear_position == sim::GearPosition::REVERSE) ? "R" : "N";

    static char body[640];
    int n = snprintf(body, sizeof(body),
        "{\"uptime\":%u,"
        "\"vx\":%.3f,\"vy\":%.3f,"
        "\"yaw\":%.2f,\"yawr\":%.2f,"
        "\"x\":%.2f,\"y\":%.2f,"
        "\"steer\":%.2f,\"soc\":%.1f,"
        "\"ofl\":%.2f,\"ofr\":%.2f,\"orl\":%.2f,\"orr\":%.2f,"
        "\"mu\":%.2f,"
        "\"en\":%d,\"gear\":\"%s\","
        "\"torq\":%.1f,\"brk\":%.2f,\"csteer\":%.2f,"
        "\"cantx\":%u,\"canrx\":%u,\"canto\":%u,\"lrx\":%.3f,"
        "\"hu\":%zu,\"hf\":%zu,\"htot\":%u,\"loop\":%u,"
        "\"mqc\":%d,\"mqrx\":%u,\"src\":%d}",
        uptime_s,
        s.v_mps, s.vy_mps,
        s.yaw_rad * 57.2958, s.yaw_rate_radps * 57.2958,
        s.x_m, s.y_m,
        s.steer_virtual_rad * 57.2958, s.batt_soc_pct,
        s.omega_fl_radps, s.omega_fr_radps, s.omega_rl_radps, s.omega_rr_radps,
        s.surface_mu,
        c.system_enable ? 1 : 0, gear_str,
        c.drive_torque_cmd_nm, c.brake_cmd_pct, c.steer_cmd_deg,
        (uint32_t)atomic_get(&g_can_tx_count),
        (uint32_t)atomic_get(&g_can_rx_count),
        (uint32_t)atomic_get(&g_can_timeout_count),
        c.last_update_t_s,
        st.heap_used, st.heap_free,
        (uint32_t)CONFIG_HEAP_MEM_POOL_SIZE,
        st.plant_loop_us_max,
        g_mqtt_connected ? 1 : 0,
        (uint32_t)atomic_get(&g_mqtt_rx_count),
        src);

    if (n <= 0 || n >= (int)sizeof(body)) return;

    char hdr[128];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n", n);
    zsock_send(fd, hdr, hn, 0);
    zsock_send(fd, body, n, 0);
}

// ── Login page ────────────────────────────────────────────────────────────────

void send_login_page(int fd, bool bad_token)
{
    static const char hdr[] =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>HDV Sim &mdash; Login</title>"
        "<style>"
        "body{background:#0d1117;color:#c9d1d9;font-family:monospace;"
        "display:flex;align-items:center;justify-content:center;"
        "min-height:100vh;margin:0;}"
        ".card{background:#161b22;border:1px solid #30363d;border-radius:8px;"
        "padding:32px 40px;width:420px;}"
        "h1{color:#58a6ff;font-size:18px;margin:0 0 4px;}"
        ".sub{color:#8b949e;font-size:12px;margin:0 0 24px;}"
        "label{display:block;color:#8b949e;font-size:12px;margin-bottom:6px;}"
        "input[type=text]{"
        "width:100%;box-sizing:border-box;"
        "background:#0d1117;border:1px solid #30363d;border-radius:6px;"
        "color:#c9d1d9;font-family:monospace;font-size:13px;"
        "padding:12px 14px;margin-bottom:20px;}"
        "input[type=text]:focus{outline:none;border-color:#58a6ff;}"
        "button{width:100%;background:#238636;border:none;border-radius:6px;"
        "color:#fff;font-family:monospace;font-size:14px;padding:10px;cursor:pointer;}"
        "button:hover{background:#2ea043;}"
        ".err{color:#f85149;font-size:13px;margin-bottom:16px;"
        "padding:8px;border:1px solid #f8514944;border-radius:6px;background:#f8514911;}"
        "</style></head><body>"
        "<div class='card'>"
        "<h1>&#9651;&nbsp;HDV Simulator</h1>"
        "<p class='sub'>Heavy-Duty Electric Vehicle &mdash; Secure Dashboard</p>";

    zsock_send(fd, hdr, sizeof(hdr) - 1, 0);

    if (bad_token) {
        static const char err[] =
            "<p class='err'>&#10007;&nbsp;Invalid token &mdash; try again.</p>";
        zsock_send(fd, err, sizeof(err) - 1, 0);
    }

    static const char form[] =
        "<form method='post' action='/login'>"
        "<label for='tok'>API Token</label>"
        "<input id='tok' type='text' name='token'"
        " placeholder='paste token here' autocomplete='off' autofocus spellcheck='false'>"
        "<button type='submit'>Sign In &rarr;</button>"
        "</form>"
        "</div></body></html>";
    zsock_send(fd, form, sizeof(form) - 1, 0);
}

// ── HTML dashboard page builder ───────────────────────────────────────────────

void send_page(int fd)
{
    char buf[512];

    // HTTP header + <head> + <style> open tag
    send_str(fd, kHtmlHead);
    // Embedded CSS from dashboard.css (built via generate_inc_file_for_target)
    send_str(fd, kDashboardCss);
    // </style></head><body>
    send_str(fd, kHtmlStyleClose);

    // Snapshot shared state
    plant::PlantState s{};
    sim::ActuatorCmd  c{};

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    s = g_state;
    k_mutex_unlock(&g_state_mutex);

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    c = g_cmd;
    k_mutex_unlock(&g_cmd_mutex);

    uint32_t ms      = k_uptime_get_32();
    uint32_t sec_up  = ms / 1000;
    uint32_t h       = sec_up / 3600;
    uint32_t m       = (sec_up % 3600) / 60;
    uint32_t sc      = sec_up % 60;

    // Header bar — static parts via send_str, only uptime needs snprintf
    send_str(fd,
        "<div class='header'>"
        "<h1>Heavy-Duty Electric Vehicle &mdash; Simulator Dashboard</h1>"
        "<span class='badge'>&#9679; ONLINE</span>");
    snprintf(buf, sizeof(buf),
        "<span class='meta'>Uptime&nbsp;<span id='up'>%02u:%02u:%02u</span></span>", h, m, sc);
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

    // Two-column grid
    send_str(fd, "<div class='grid'>");

    // Left card — Plant State
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

    // Right column — two stacked cards
    send_str(fd, "<div style='display:flex;flex-direction:column;gap:12px'>");

    // Actuator command card
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

    // CAN stats card
    send_str(fd, "<div class='card'><h2>CAN Stats</h2><table>");
    snprintf(buf, sizeof(buf),
        "<tr><td>TX frames</td><td id='can-tx'>%u</td></tr>"
        "<tr><td>RX frames</td><td id='can-rx'>%u</td></tr>"
        "<tr><td>Timeouts</td>"
        "<td id='can-to'><span class='%s'>%u</span></td></tr>"
        "<tr><td>Last RX</td><td id='can-lrx'>%.3f s</td></tr>",
        (uint32_t)atomic_get(&g_can_tx_count),
        (uint32_t)atomic_get(&g_can_rx_count),
        atomic_get(&g_can_timeout_count) ? "val-warn" : "",
        (uint32_t)atomic_get(&g_can_timeout_count),
        c.last_update_t_s);
    send_str(fd, buf);
    send_str(fd, "</table></div>");

    send_str(fd, "</div>"); // right column
    send_str(fd, "</div>"); // grid

    // Controls card
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

    // Control source card — toggle which input writes g_cmd
    {
        int src = (int)atomic_get(&g_ctrl_source);
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

    // MQTT broker card — runtime broker address + connection status
    // Split into two snprintf calls: form part (~350 B) + status part (~180 B)
    // to stay within buf[512].
    {
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
            g_mqtt_broker_addr, g_mqtt_broker_port);
        send_str(fd, buf);

        snprintf(buf, sizeof(buf),
            "<p style='margin:6px 0 0;font-size:12px'>"
            "Status:&nbsp;<span id='mqtt-st'><span class='%s'>%s</span></span>"
            "&nbsp;&nbsp;MQTT&nbsp;RX:&nbsp;<span id='mqtt-rx'>%u</span></p>"
            "</div>",
            g_mqtt_connected ? "val-hi" : "val-warn",
            g_mqtt_connected ? "connected" : "disconnected",
            (uint32_t)atomic_get(&g_mqtt_rx_count));
        send_str(fd, buf);
    }

    // System resources card (heap + worst-case sim loop)
    send_resources_card(fd);

    // Kernel threads card
    send_threads_card(fd);

    // Vehicle info card (static — defined in http_html.hpp)
    send_str(fd, kVehicleCard);
    send_str(fd, kHtmlFoot);
}
