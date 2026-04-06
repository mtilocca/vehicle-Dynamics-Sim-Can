// zephyr/src/http_server.cpp
// Minimal HTTP server — serves a live dashboard on port 80.
// Single-threaded: accept → parse request line → apply command if query present
//                 → build HTML → send → close.
// Uses Zephyr native socket API (zsock_*) — no CONFIG_POSIX_API needed.
//
// Priority 10: below sim (Phase 4, prio 5), above LED (prio 12) and shell (prio 14).

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <stdio.h>    /* snprintf */
#include <stdlib.h>   /* atof */
#include <string.h>   /* strncmp, strlen */

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

// ── Shared state — defined in main.cpp ───────────────────────────────────────
extern plant::PlantState g_state;
extern sim::ActuatorCmd  g_cmd;
extern struct k_mutex    g_state_mutex;
extern struct k_mutex    g_cmd_mutex;

extern volatile uint32_t g_can_tx_count;
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_timeout_count;
extern volatile double   g_last_rx_t;
extern double            g_surface_mu;

// ── Helpers ───────────────────────────────────────────────────────────────────

static void send_str(int fd, const char* s)
{
    int len = (int)strlen(s);
    int sent = 0;
    while (sent < len) {
        int r = zsock_send(fd, s + sent, len - sent, 0);
        if (r <= 0) return;
        sent += r;
    }
}

// Read the HTTP request line ("GET /path?query HTTP/1.1\r\n") byte-by-byte.
// Copies everything after '?' and before the next ' ' into query_out.
// query_out[0] == '\0' if there is no query string.
static void read_request_line(int fd, char* query_out, int qlen)
{
    query_out[0] = '\0';

    char line[192];
    int  li = 0;

    while (li < (int)sizeof(line) - 1) {
        char c;
        if (zsock_recv(fd, &c, 1, 0) <= 0) break;
        if (c == '\n') break;
        line[li++] = c;
    }
    line[li] = '\0';

    // Find '?'
    char* q = line;
    while (*q && *q != '?') ++q;
    if (!*q) return; // no query string

    ++q; // skip '?'
    int i = 0;
    while (*q && *q != ' ' && *q != '\r' && i < qlen - 1)
        query_out[i++] = *q++;
    query_out[i] = '\0';
}

// Drain remaining HTTP headers until the blank line (\r\n\r\n).
static void drain_headers(int fd)
{
    int state = 0;
    while (state < 4) {
        char c;
        if (zsock_recv(fd, &c, 1, 0) <= 0) break;
        if      (state == 0 && c == '\r') state = 1;
        else if (state == 1 && c == '\n') state = 2;
        else if (state == 2 && c == '\r') state = 3;
        else if (state == 3 && c == '\n') state = 4;
        else                              state = (c == '\r') ? 1 : 0;
    }
}

// ── Query string parser ───────────────────────────────────────────────────────

// Returns pointer to start of the value for 'key' in query string 'qs',
// or NULL if not found.  Value ends at '&', ' ', '\0'.
static const char* find_param(const char* qs, const char* key)
{
    int klen = (int)strlen(key);
    const char* p = qs;
    while (*p) {
        if (strncmp(p, key, klen) == 0 && p[klen] == '=')
            return p + klen + 1;
        while (*p && *p != '&') ++p;
        if (*p == '&') ++p;
    }
    return nullptr;
}

// Parse query string and apply found params to g_cmd (partial update).
static void apply_web_cmd(const char* qs)
{
    sim::ActuatorCmd cmd;
    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    cmd = g_cmd;
    k_mutex_unlock(&g_cmd_mutex);

    const char* v;
    if ((v = find_param(qs, "steer")))  cmd.steer_cmd_deg       = atof(v);
    if ((v = find_param(qs, "torque"))) cmd.drive_torque_cmd_nm = atof(v);
    if ((v = find_param(qs, "brake")))  cmd.brake_cmd_pct       = atof(v);
    if ((v = find_param(qs, "gear"))) {
        cmd.gear_position = (v[0] == 'F') ? sim::GearPosition::FORWARD :
                            (v[0] == 'R') ? sim::GearPosition::REVERSE :
                                            sim::GearPosition::NEUTRAL;
    }
    if ((v = find_param(qs, "enable"))) cmd.system_enable = (v[0] == '1');

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    g_cmd = cmd;
    k_mutex_unlock(&g_cmd_mutex);

    // Poke watchdog so plant_thread keeps this command alive for 500 ms
    g_last_rx_t = (double)k_uptime_get_32() / 1000.0;
}

// ── HTML page builder ─────────────────────────────────────────────────────────

static void send_page(int fd)
{
    char buf[512];

    // ── HTTP header ───────────────────────────────────────────────────────────
    send_str(fd,
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Connection: close\r\n"
        "\r\n"
    );

    // ── HTML head + global styles ─────────────────────────────────────────────
    send_str(fd,
        "<!DOCTYPE html><html lang='en'><head>"
        "<meta charset='utf-8'>"
        "<meta http-equiv='refresh' content='2'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>XCMG XDE320 &mdash; Simulator</title>"
        "<style>"
        "*{box-sizing:border-box;margin:0;padding:0}"
        "body{background:#161b22;color:#e6edf3;font-family:monospace;font-size:14px;padding:16px}"
        "h1{font-size:18px;font-weight:600;color:#58a6ff}"
        "h2{font-size:13px;font-weight:600;color:#58a6ff;margin-bottom:8px;text-transform:uppercase;letter-spacing:.05em}"
        ".header{display:flex;align-items:center;gap:16px;padding:12px 16px;background:#21262d;border-radius:8px;margin-bottom:16px}"
        ".badge{background:#1a7f37;color:#fff;font-size:11px;padding:2px 8px;border-radius:12px}"
        ".meta{color:#8b949e;font-size:12px}"
        ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px}"
        ".card{background:#21262d;border-radius:8px;padding:14px;margin-bottom:12px}"
        "table{width:100%;border-collapse:collapse}"
        "td{padding:3px 0;vertical-align:top}"
        "td:first-child{color:#8b949e;width:52%;padding-right:8px}"
        "td:last-child{color:#e6edf3;font-weight:500}"
        ".val-hi{color:#3fb950}"
        ".val-warn{color:#d29922}"
    );
    send_str(fd,
        ".btn{display:inline-block;padding:6px 14px;border-radius:6px;font-family:monospace;"
        "font-size:13px;font-weight:600;text-decoration:none;border:none;cursor:pointer;margin:3px 2px}"
        ".btn-stop{background:#da3633;color:#fff}"
        ".btn-fwd{background:#238636;color:#fff}"
        ".btn-rev{background:#1f6feb;color:#fff}"
        ".btn-steer{background:#30363d;color:#e6edf3;border:1px solid #444}"
        ".btn-inject{background:#1f6feb;color:#fff;padding:7px 20px}"
        "label{color:#8b949e;font-size:12px;margin-right:4px}"
        "input[type=number],select{"
        "background:#0d1117;color:#e6edf3;border:1px solid #30363d;"
        "border-radius:4px;padding:4px 6px;font-family:monospace;font-size:13px;"
        "width:100px;margin-right:12px}"
        "select{width:auto}"
        "input[type=checkbox]{margin-right:4px;vertical-align:middle}"
        ".ctrl-row{display:flex;flex-wrap:wrap;align-items:center;gap:4px;margin-top:8px}"
        "</style></head><body>"
    );

    // ── Snapshot shared state ─────────────────────────────────────────────────
    plant::PlantState s{};
    sim::ActuatorCmd  c{};

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    s = g_state;
    k_mutex_unlock(&g_state_mutex);

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    c = g_cmd;
    k_mutex_unlock(&g_cmd_mutex);

    uint32_t ms  = k_uptime_get_32();
    uint32_t sec_up = ms / 1000;
    uint32_t h   = sec_up / 3600;
    uint32_t m   = (sec_up % 3600) / 60;
    uint32_t sc  = sec_up % 60;

    // ── Header bar ────────────────────────────────────────────────────────────
    snprintf(buf, sizeof(buf),
        "<div class='header'>"
        "<h1>XCMG XDE320 &mdash; Simulator Dashboard</h1>"
        "<span class='badge'>&#9679; ONLINE</span>"
        "<span class='meta'>Uptime&nbsp;%02u:%02u:%02u</span>"
        "<span class='meta'>IP&nbsp;192.168.1.80</span>"
        "<span class='meta'>MAC&nbsp;02:00:5E:00:53:01</span>"
        "</div>",
        h, m, sc);
    send_str(fd, buf);

    // ── Two-column grid ───────────────────────────────────────────────────────
    send_str(fd, "<div class='grid'>");

    // Left card — Plant State
    send_str(fd, "<div class='card'><h2>Plant State</h2><table>");
    snprintf(buf, sizeof(buf),
        "<tr><td>vx</td><td>%.3f m/s</td></tr>"
        "<tr><td>vy</td><td>%.3f m/s</td></tr>"
        "<tr><td>yaw</td><td>%.2f &deg;</td></tr>"
        "<tr><td>yaw rate</td><td>%.2f &deg;/s</td></tr>"
        "<tr><td>x</td><td>%.2f m</td></tr>"
        "<tr><td>y</td><td>%.2f m</td></tr>",
        s.v_mps, s.vy_mps,
        s.yaw_rad * 57.2958, s.yaw_rate_radps * 57.2958,
        s.x_m, s.y_m);
    send_str(fd, buf);
    snprintf(buf, sizeof(buf),
        "<tr><td>steer</td><td>%.2f &deg;</td></tr>"
        "<tr><td>SOC</td><td class='val-hi'>%.1f %%</td></tr>"
        "<tr><td>&omega; FL</td><td>%.2f rad/s</td></tr>"
        "<tr><td>&omega; FR</td><td>%.2f rad/s</td></tr>"
        "<tr><td>&omega; RL</td><td>%.2f rad/s</td></tr>"
        "<tr><td>&omega; RR</td><td>%.2f rad/s</td></tr>"
        "<tr><td>surface &mu;</td><td>%.2f</td></tr>",
        s.steer_virtual_rad * 57.2958,
        s.batt_soc_pct,
        s.omega_fl_radps, s.omega_fr_radps,
        s.omega_rl_radps, s.omega_rr_radps,
        s.surface_mu);
    send_str(fd, buf);
    send_str(fd, "</table></div>");

    // Right column — two stacked cards
    send_str(fd, "<div style='display:flex;flex-direction:column;gap:12px'>");

    // Actuator cmd card
    send_str(fd, "<div class='card'><h2>Actuator Command</h2><table>");
    snprintf(buf, sizeof(buf),
        "<tr><td>Enable</td><td class='%s'>%s</td></tr>"
        "<tr><td>Gear</td><td>%s</td></tr>"
        "<tr><td>Torque</td><td>%.1f Nm</td></tr>"
        "<tr><td>Brake</td><td>%.2f %%</td></tr>"
        "<tr><td>Steer</td><td>%.2f &deg;</td></tr>",
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
        "<tr><td>TX frames</td><td>%u</td></tr>"
        "<tr><td>RX frames</td><td>%u</td></tr>"
        "<tr><td>Timeouts</td><td class='%s'>%u</td></tr>"
        "<tr><td>Last RX</td><td>%.3f s</td></tr>",
        g_can_tx_count, g_can_rx_count,
        g_can_timeout_count ? "val-warn" : "",
        g_can_timeout_count,
        g_last_rx_t);
    send_str(fd, buf);
    send_str(fd, "</table></div>");

    send_str(fd, "</div>"); // right column
    send_str(fd, "</div>"); // grid

    // ── Controls card ─────────────────────────────────────────────────────────
    double sp5 = c.steer_cmd_deg + 5.0;
    double sm5 = c.steer_cmd_deg - 5.0;
    if (sp5 >  45.0) sp5 =  45.0;
    if (sm5 < -45.0) sm5 = -45.0;

    send_str(fd, "<div class='card'><h2>Controls</h2>");

    // Quick-action buttons
    send_str(fd, "<div class='ctrl-row'>");
    send_str(fd,
        "<a class='btn btn-stop' href='/?enable=1&gear=N&torque=0&brake=100&steer=0'>"
        "&#9632;&nbsp;STOP</a>");
    send_str(fd,
        "<a class='btn btn-fwd' href='/?enable=1&gear=F&torque=50000&brake=0&steer=0'>"
        "&#9654;&nbsp;Drive FWD</a>");
    send_str(fd,
        "<a class='btn btn-rev' href='/?enable=1&gear=R&torque=50000&brake=0&steer=0'>"
        "&#9664;&nbsp;Drive REV</a>");

    snprintf(buf, sizeof(buf),
        "<a class='btn btn-steer' href='/?steer=%.0f'>&#8592;&nbsp;%.0f&deg;</a>"
        "<a class='btn btn-steer' href='/?steer=%.0f'>%.0f&deg;&nbsp;&#8594;</a>",
        sm5, sm5, sp5, sp5);
    send_str(fd, buf);
    send_str(fd, "</div>");

    // Manual inject form
    const char* sel_f = (c.gear_position == sim::GearPosition::FORWARD) ? " selected" : "";
    const char* sel_n = (c.gear_position == sim::GearPosition::NEUTRAL) ? " selected" : "";
    const char* sel_r = (c.gear_position == sim::GearPosition::REVERSE) ? " selected" : "";

    send_str(fd, "<form method='get' action='/' style='margin-top:10px'>");
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

    // ── Full-width vehicle card ───────────────────────────────────────────────
    send_str(fd,
        "<div class='card'>"
        "<h2>Vehicle &mdash; XCMG XDE320 Electric</h2>"
        "<table><tr>"
        "<td>Mass</td><td>218 000 kg (218 t)</td>"
        "<td>Motor power</td><td>2 013 kW</td>"
        "<td>Motor torque</td><td>145 000 Nm</td>"
        "</tr><tr>"
        "<td>Battery</td><td>1 650 kWh</td>"
        "<td>Max speed</td><td>17.8 m/s (64 km/h)</td>"
        "<td>Gear ratio</td><td>28.0</td>"
        "</tr></table>"
        "</div>"
    );

    send_str(fd, "</body></html>");
}

// ── HTTP server thread ────────────────────────────────────────────────────────

static void http_server_thread(void*, void*, void*)
{
    k_msleep(2000);

    int srv = zsock_socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (srv < 0) {
        LOG_ERR("HTTP: socket() failed: %d", srv);
        return;
    }

    int opt = 1;
    zsock_setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(80);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(srv, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERR("HTTP: bind() failed");
        zsock_close(srv);
        return;
    }

    if (zsock_listen(srv, 3) < 0) {
        LOG_ERR("HTTP: listen() failed");
        zsock_close(srv);
        return;
    }

    LOG_INF("HTTP server listening on 192.168.1.80:80");

    while (true) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client = zsock_accept(srv, (struct sockaddr*)&client_addr, &client_len);
        if (client < 0) {
            k_msleep(10);
            continue;
        }

        char query[128] = {};
        read_request_line(client, query, sizeof(query));
        drain_headers(client);

        if (query[0] != '\0') {
            apply_web_cmd(query);
        }

        send_page(client);
        zsock_close(client);
    }
}

K_THREAD_DEFINE(http_tid, 8192, http_server_thread, NULL, NULL, NULL, 10, 0, 0);
