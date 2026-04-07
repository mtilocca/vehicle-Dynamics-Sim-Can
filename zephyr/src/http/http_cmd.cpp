// zephyr/src/http/http_cmd.cpp
// HTTP request parsing and actuator command injection.
// Reads the request line, extracts the query string, and writes
// the parsed values into the shared g_cmd / g_last_rx_t globals.

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>  /* atof */
#include <string.h>  /* strncmp, strlen */

#include "sim/actuator_cmd.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

// ── Shared globals (defined in main.cpp) ─────────────────────────────────────
extern sim::ActuatorCmd g_cmd;
extern struct k_mutex   g_cmd_mutex;
extern volatile double  g_last_rx_t;

// ── Request line reader ───────────────────────────────────────────────────────

// Reads the HTTP request line byte-by-byte and copies everything after '?'
// and before the next ' ' into query_out. query_out[0] == '\0' if no query.
void read_request_line(int fd, char* query_out, int qlen)
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

    char* q = line;
    while (*q && *q != '?') ++q;
    if (!*q) return;

    ++q;
    int i = 0;
    while (*q && *q != ' ' && *q != '\r' && i < qlen - 1)
        query_out[i++] = *q++;
    query_out[i] = '\0';
}

// ── Header drainer ────────────────────────────────────────────────────────────

// Consumes remaining HTTP headers until the blank line (\r\n\r\n).
void drain_headers(int fd)
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

// Returns pointer to the value for 'key' in query string 'qs', or NULL.
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

// ── Command application ───────────────────────────────────────────────────────

// Parses query string and applies found params to g_cmd (partial update).
void apply_web_cmd(const char* qs)
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

    // Poke watchdog so plant_thread keeps this command alive for 500 ms.
    g_last_rx_t = (double)k_uptime_get_32() / 1000.0;
}
