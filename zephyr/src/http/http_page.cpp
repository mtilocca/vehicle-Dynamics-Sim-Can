// zephyr/src/http/http_page.cpp
// Web UI delivery + JSON APIs.
// Serves the Vite-built UI (webui/dist, embedded at build time), the login
// page, and the JSON endpoints /api/state and /api/system.

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>   /* snprintf */
#include <string.h>  /* strlen */

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "http_internal.hpp"
#include "stats/sys_stats.hpp"
#include "mqtt/mqtt_client.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// Web UI embedded from webui/dist at build time via generate_inc_file_for_target()
static const char kIndexHtml[] = {
#include "webui_index.inc"
};
static const char kAppJs[] = {
#include "webui_app_js.inc"
};
static const char kAppCss[] = {
#include "webui_app_css.inc"
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

// ── Static assets ─────────────────────────────────────────────────────────────

static void send_static(int fd, const char* data, size_t len, const char* mime)
{
    char hdr[160];
    int hn = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %u\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n", mime, (unsigned)len);
    zsock_send(fd, hdr, hn, 0);

    size_t sent = 0;
    while (sent < len) {
        size_t chunk = (len - sent > 2048) ? 2048 : (len - sent);
        int r = zsock_send(fd, data + sent, chunk, 0);
        if (r <= 0) return;
        sent += (size_t)r;
    }
}

void send_shell(int fd)   { send_static(fd, kIndexHtml, sizeof(kIndexHtml), "text/html; charset=utf-8"); }
void send_app_js(int fd)  { send_static(fd, kAppJs,     sizeof(kAppJs),     "text/javascript; charset=utf-8"); }
void send_app_css(int fd) { send_static(fd, kAppCss,    sizeof(kAppCss),    "text/css; charset=utf-8"); }

void send_204(int fd)
{
    static const char r[] =
        "HTTP/1.1 204 No Content\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n";
    zsock_send(fd, r, sizeof(r) - 1, 0);
}

// ── /api/system — threads, memory map, MQTT broker config ───────────────────

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

void send_api_system(int fd)
{
    // Flash footprint: read the MCUboot image header at slot0 base.
    //   +0x08 ih_hdr_size (u16), +0x0A ih_protect_tlv_size (u16), +0x0C ih_img_size (u32)
    extern char __rom_region_start[];
    extern char _image_ram_start[];
    extern char _image_ram_end[];

    const uint8_t* hdr = (const uint8_t*)__rom_region_start;
    uint16_t ih_hdr_size, ih_tlv_size;
    uint32_t ih_img_size;
    memcpy(&ih_hdr_size, hdr + 8,  sizeof(ih_hdr_size));
    memcpy(&ih_tlv_size, hdr + 10, sizeof(ih_tlv_size));
    memcpy(&ih_img_size, hdr + 12, sizeof(ih_img_size));
    size_t flash_used = (size_t)ih_hdr_size + ih_img_size + ih_tlv_size;
    size_t ram_image  = (size_t)(_image_ram_end - _image_ram_start);

    s_thread_count = 0;
    k_thread_foreach(collect_thread_cb, nullptr);

    size_t stacks_sz = 0;
    for (int i = 0; i < s_thread_count; ++i) stacks_sz += s_threads[i].stack_total;

    static char body[2304];
    int n = snprintf(body, sizeof(body),
        "{\"broker_addr\":\"%.31s\",\"broker_port\":%d,"
        "\"flash_used\":%zu,\"flash_total\":%u,"
        "\"ram_image\":%zu,\"ram_total\":%u,"
        "\"heap_gen\":%u,\"heap_tls\":%u,\"stacks\":%zu,\"threads\":[",
        g_mqtt_broker_addr, g_mqtt_broker_port,
        flash_used, 768u * 1024u,
        ram_image, 512u * 1024u,
        (unsigned)CONFIG_HEAP_MEM_POOL_SIZE, (unsigned)CONFIG_MBEDTLS_HEAP_SIZE, stacks_sz);

    for (int i = 0; i < s_thread_count && n > 0 && n < (int)sizeof(body) - 96; ++i) {
        n += snprintf(body + n, sizeof(body) - n,
            "%s{\"n\":\"%.23s\",\"p\":%d,\"u\":%zu,\"t\":%zu}",
            i ? "," : "", s_threads[i].name, s_threads[i].priority,
            s_threads[i].stack_used, s_threads[i].stack_total);
    }
    if (n <= 0 || n >= (int)sizeof(body) - 4) return;
    n += snprintf(body + n, sizeof(body) - n, "]}");

    char h[128];
    int hn = snprintf(h, sizeof(h),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Cache-Control: no-cache\r\n"
        "Connection: close\r\n"
        "\r\n", n);
    zsock_send(fd, h, hn, 0);
    zsock_send(fd, body, n, 0);
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
