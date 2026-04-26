// zephyr/src/http/page/http_page_api.cpp
// GET /api/state — JSON telemetry snapshot for JS polling.

#include <zephyr/logging/log.h>
#include "http_page_internal.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

void send_api_state(int fd)
{
    plant::PlantState s{};
    sim::ActuatorCmd  c{};
    SysStats          st{};

    { hdv::MutexGuard g(g_sim_plant_mtx); s  = g_sim_bus.plant; }
    { hdv::MutexGuard g(g_sim_cmd_mtx);  c  = g_sim_bus.cmd;  }
    { hdv::MutexGuard g(g_health_mtx);   st = g_health_bus.stats; }

    uint32_t uptime_s = k_uptime_get_32() / 1000;
    int src = (int)atomic_get(&g_ctrl_bus.ctrl_source);
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
        (uint32_t)atomic_get(&g_ctrl_bus.can_tx_count),
        (uint32_t)atomic_get(&g_ctrl_bus.can_rx_count),
        (uint32_t)atomic_get(&g_ctrl_bus.can_timeout_count),
        c.last_update_t_s,
        st.heap_used, st.heap_free,
        (uint32_t)CONFIG_HEAP_MEM_POOL_SIZE,
        st.plant_loop_us_max,
        g_mqtt_connected ? 1 : 0,
        (uint32_t)atomic_get(&g_ctrl_bus.mqtt_rx_count),
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
