// zephyr/src/shell/debug_cmds.cpp
// Zephyr shell commands for runtime inspection and control.
//
// Security: destructive commands (plant reset, plant inject, can tx_test)
// require 'login <token>' first. Token matches HDV_API_TOKEN from http_auth.hpp.
//
// Commands:
//   login <token>         — unlock destructive commands
//   plant state           — dump all PlantState fields
//   plant mu <val>        — change surface friction coefficient
//   plant reset           — zero plant state (requires login)
//   plant inject ...      — inject actuator cmd (requires login)
//   can stats             — TX/RX frame counts
//   can rx_frame          — last decoded ACTUATOR_CMD_1
//   can map               — dump static CAN frame table
//   can tx_test ...       — send test frame (requires login)
//   can timeout <ms>      — set CAN RX watchdog timeout
//   stats                 — system health snapshot
//   mem                   — heap usage
//   threads               — all threads with stack info
//   vehicle info          — vehicle parameter summary
//   network mac           — MAC address and IP
//   system uptime         — uptime HH:MM:SS

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/sys/atomic.h>
#include <stdlib.h>
#include <string.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "can/can_map_static.hpp"
#include "http_auth.hpp"   // HDV_API_TOKEN

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// ── Shared state — defined in main.cpp ───────────────────────────────────────
extern plant::PlantState  g_state;
extern sim::ActuatorCmd   g_cmd;
extern struct k_mutex     g_state_mutex;
extern struct k_mutex     g_cmd_mutex;
extern atomic_t           g_can_tx_count;
extern atomic_t           g_can_rx_count;
extern atomic_t           g_can_timeout_count;
extern double             g_surface_mu;

struct SysStats {
    uint32_t plant_loop_us_max;
    uint32_t can_rx_total;
    uint32_t can_timeout_total;
    size_t   heap_used;
    size_t   heap_free;
};
extern SysStats       g_sys_stats;
extern struct k_mutex g_stats_mutex;

// ── Shell auth state ──────────────────────────────────────────────────────────
static bool g_shell_unlocked = false;

static bool require_login(const struct shell* sh) {
    if (!g_shell_unlocked) {
        shell_error(sh, "Locked — run 'login <token>' first");
        return false;
    }
    return true;
}

// ── login ─────────────────────────────────────────────────────────────────────
static int cmd_login(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: login <token>");
        return -EINVAL;
    }
    if (strncmp(argv[1], HDV_API_TOKEN, strlen(HDV_API_TOKEN)) == 0 &&
        strlen(argv[1]) == strlen(HDV_API_TOKEN)) {
        g_shell_unlocked = true;
        shell_print(sh, "Shell unlocked — destructive commands enabled");
    } else {
        shell_error(sh, "Invalid token");
        g_shell_unlocked = false;
        return -EACCES;
    }
    return 0;
}
SHELL_CMD_REGISTER(login, NULL, "Unlock destructive commands", cmd_login);

// ── plant state ───────────────────────────────────────────────────────────────
static int cmd_plant_state(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    plant::PlantState s = g_state;
    k_mutex_unlock(&g_state_mutex);

    shell_print(sh, "--- Plant State ---");
    shell_print(sh, "  t_s      = %.3f s",    s.t_s);
    shell_print(sh, "  vx       = %.3f m/s",  s.v_mps);
    shell_print(sh, "  vy       = %.3f m/s",  s.vy_mps);
    shell_print(sh, "  yaw      = %.3f deg",  s.yaw_rad * 57.2958);
    shell_print(sh, "  yaw_rate = %.3f deg/s", s.yaw_rate_radps * 57.2958);
    shell_print(sh, "  x        = %.2f m",    s.x_m);
    shell_print(sh, "  y        = %.2f m",    s.y_m);
    shell_print(sh, "  steer    = %.2f deg",  s.steer_virtual_rad * 57.2958);
    shell_print(sh, "  soc      = %.1f %%",   s.batt_soc_pct);
    shell_print(sh, "  omega_fl = %.2f rad/s", s.omega_fl_radps);
    shell_print(sh, "  omega_fr = %.2f rad/s", s.omega_fr_radps);
    shell_print(sh, "  omega_rl = %.2f rad/s", s.omega_rl_radps);
    shell_print(sh, "  omega_rr = %.2f rad/s", s.omega_rr_radps);
    shell_print(sh, "  Fx_total = %.0f N",
                s.Fx_fl + s.Fx_fr + s.Fx_rl + s.Fx_rr);
    shell_print(sh, "  Fy_total = %.0f N",
                s.Fy_fl + s.Fy_fr + s.Fy_rl + s.Fy_rr);
    shell_print(sh, "  surface_mu = %.2f", s.surface_mu);
    return 0;
}

static int cmd_plant_mu(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: plant mu <value>  (0.1 .. 1.0)");
        return -EINVAL;
    }
    float mu = strtof(argv[1], nullptr);
    if (mu < 0.1f || mu > 1.0f) {
        shell_error(sh, "mu must be between 0.1 and 1.0");
        return -EINVAL;
    }
    g_surface_mu = static_cast<double>(mu);
    shell_print(sh, "Surface mu set to %.2f", (double)mu);
    return 0;
}

static int cmd_plant_reset(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    if (!require_login(sh)) return -EACCES;

    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state = plant::PlantState{};
    k_mutex_unlock(&g_state_mutex);
    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    g_cmd = sim::ActuatorCmd{};
    k_mutex_unlock(&g_cmd_mutex);
    shell_print(sh, "Plant state and command reset to zero");
    return 0;
}

static int cmd_plant_inject(const struct shell* sh, size_t argc, char** argv)
{
    if (!require_login(sh)) return -EACCES;

    if (argc < 4) {
        shell_print(sh, "Usage: plant inject <steer_deg> <torque_nm> <brake_pct> [enable=1]");
        shell_print(sh, "  steer_deg  : -45 .. +45");
        shell_print(sh, "  torque_nm  : 0 .. 145000");
        shell_print(sh, "  brake_pct  : 0 .. 100");
        return -EINVAL;
    }

    auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };

    double steer  = clamp(strtod(argv[1], nullptr), -45.0,     45.0);
    double torque = clamp(strtod(argv[2], nullptr),   0.0, 145000.0);
    double brake  = clamp(strtod(argv[3], nullptr),   0.0,    100.0);
    bool   enable = (argc > 4) ? (strtod(argv[4], nullptr) != 0.0) : true;

    sim::ActuatorCmd cmd;
    cmd.steer_cmd_deg       = steer;
    cmd.drive_torque_cmd_nm = torque;
    cmd.brake_cmd_pct       = brake;
    cmd.system_enable       = enable;
    cmd.gear_position       = (torque >= 0.0) ? sim::GearPosition::FORWARD
                                              : sim::GearPosition::REVERSE;
    cmd.last_update_t_s     = (double)k_uptime_get_32() / 1000.0;

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    g_cmd = cmd;
    k_mutex_unlock(&g_cmd_mutex);

    shell_print(sh, "Injected: steer=%.1f deg  torque=%.0f Nm  brake=%.2f  enable=%d",
                steer, torque, brake, (int)enable);
    shell_print(sh, "  (watchdog: resend within 500 ms to maintain)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(plant_cmds,
    SHELL_CMD(state,  NULL, "Dump plant state",                    cmd_plant_state),
    SHELL_CMD(mu,     NULL, "Set surface mu",                      cmd_plant_mu),
    SHELL_CMD(reset,  NULL, "Reset plant state (requires login)",  cmd_plant_reset),
    SHELL_CMD(inject, NULL, "Inject actuator cmd (requires login)", cmd_plant_inject),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(plant, &plant_cmds, "Plant model commands", NULL);

// ── can stats / rx_frame / map / tx_test / timeout ───────────────────────────

// Runtime-adjustable CAN RX watchdog timeout (seconds)
double g_can_rx_timeout_s = 0.5;

static int cmd_can_stats(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;

    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    double last_rx = g_cmd.last_update_t_s;
    k_mutex_unlock(&g_cmd_mutex);

    shell_print(sh, "--- CAN Stats ---");
    shell_print(sh, "  TX frames  : %u", (uint32_t)atomic_get(&g_can_tx_count));
    shell_print(sh, "  RX frames  : %u", (uint32_t)atomic_get(&g_can_rx_count));
    shell_print(sh, "  RX timeouts: %u", (uint32_t)atomic_get(&g_can_timeout_count));
    shell_print(sh, "  Last RX    : %.3f s", last_rx);
    shell_print(sh, "  RX timeout : %.3f s", g_can_rx_timeout_s);
    return 0;
}

static int cmd_can_rx_frame(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    k_mutex_lock(&g_cmd_mutex, K_FOREVER);
    sim::ActuatorCmd c = g_cmd;
    k_mutex_unlock(&g_cmd_mutex);

    shell_print(sh, "--- Last ACTUATOR_CMD_1 ---");
    shell_print(sh, "  system_enable   = %d",   (int)c.system_enable);
    shell_print(sh, "  gear_position   = %d",   (int)c.gear_position);
    shell_print(sh, "  drive_torque_nm = %.1f", c.drive_torque_cmd_nm);
    shell_print(sh, "  brake_pct       = %.2f", c.brake_cmd_pct);
    shell_print(sh, "  steer_deg       = %.2f", c.steer_cmd_deg);
    shell_print(sh, "  last_update_t_s = %.3f", c.last_update_t_s);
    return 0;
}

extern "C" int can_tx_test_frame(double steer_deg, double torque_nm,
                                  double brake_pct, bool enable);

static int cmd_can_tx_test(const struct shell* sh, size_t argc, char** argv)
{
    if (!require_login(sh)) return -EACCES;

    double steer  = (argc > 1) ? strtod(argv[1], nullptr) : 10.0;
    double torque = (argc > 2) ? strtod(argv[2], nullptr) : 50000.0;
    double brake  = (argc > 3) ? strtod(argv[3], nullptr) : 0.0;

    shell_print(sh, "Sending ACTUATOR_CMD_1: steer=%.1f  torque=%.0f Nm  brake=%.1f %%",
                steer, torque, brake);
    int ret = can_tx_test_frame(steer, torque, brake, true);
    if (ret < 0) {
        shell_error(sh, "can_send failed: %d", ret);
        return ret;
    }
    shell_print(sh, "Sent OK.");
    return 0;
}

static int cmd_can_map(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Static CAN Map (%u frames, %u RX, %u TX) ---",
                (unsigned)can_static::k_frame_count,
                (unsigned)can_static::k_rx_count,
                (unsigned)can_static::k_tx_count);
    for (uint8_t i = 0; i < can_static::k_frame_count; ++i) {
        const can_static::FrameDef& f = can_static::k_frames[i];
        shell_print(sh, "  [%s] 0x%08X  %-22s  %2u sig(s)  %3u ms",
                    f.is_rx ? "RX" : "TX",
                    (unsigned)f.id, f.name,
                    (unsigned)f.sig_count, (unsigned)f.cycle_ms);
    }
    return 0;
}

static int cmd_can_timeout(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_print(sh, "Current CAN RX watchdog timeout: %.0f ms",
                    g_can_rx_timeout_s * 1000.0);
        return 0;
    }
    double ms = strtod(argv[1], nullptr);
    if (ms < 100.0 || ms > 5000.0) {
        shell_error(sh, "Timeout must be 100..5000 ms");
        return -EINVAL;
    }
    g_can_rx_timeout_s = ms / 1000.0;
    shell_print(sh, "CAN RX watchdog timeout set to %.0f ms", ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(can_cmds,
    SHELL_CMD(stats,    NULL, "CAN TX/RX counters",                  cmd_can_stats),
    SHELL_CMD(rx_frame, NULL, "Last decoded actuator cmd",            cmd_can_rx_frame),
    SHELL_CMD(map,      NULL, "Dump static CAN frame map",            cmd_can_map),
    SHELL_CMD(tx_test,  NULL, "Send test frame (requires login)",     cmd_can_tx_test),
    SHELL_CMD(timeout,  NULL, "Get/set CAN RX watchdog timeout (ms)", cmd_can_timeout),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can, &can_cmds, "CAN bus commands", NULL);

// ── stats ─────────────────────────────────────────────────────────────────────

static int cmd_stats(const struct shell* sh, size_t argc, char** argv)
{
    if (argc > 1 && strncmp(argv[1], "reset", 5) == 0) {
        k_mutex_lock(&g_stats_mutex, K_FOREVER);
        g_sys_stats.plant_loop_us_max = 0;
        k_mutex_unlock(&g_stats_mutex);
        shell_print(sh, "Stats counters reset");
        return 0;
    }

    SysStats s;
    k_mutex_lock(&g_stats_mutex, K_FOREVER);
    s = g_sys_stats;
    k_mutex_unlock(&g_stats_mutex);

    shell_print(sh, "--- System Stats ---");
    shell_print(sh, "  Plant loop max : %u us", s.plant_loop_us_max);
    shell_print(sh, "  CAN RX total   : %u",    s.can_rx_total);
    shell_print(sh, "  CAN timeouts   : %u",    s.can_timeout_total);
    shell_print(sh, "  Heap used      : %zu B",  s.heap_used);
    shell_print(sh, "  Heap free      : %zu B",  s.heap_free);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(stats_sub,
    SHELL_CMD(reset, NULL, "Reset stats counters", cmd_stats),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(stats, &stats_sub, "System health stats [reset]", cmd_stats);

// ── mem ───────────────────────────────────────────────────────────────────────

static int cmd_mem(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    SysStats s;
    k_mutex_lock(&g_stats_mutex, K_FOREVER);
    s = g_sys_stats;
    k_mutex_unlock(&g_stats_mutex);

    shell_print(sh, "--- Heap Memory ---");
    shell_print(sh, "  Used : %zu B", s.heap_used);
    shell_print(sh, "  Free : %zu B", s.heap_free);
    if (s.heap_used + s.heap_free > 0) {
        uint32_t pct = (uint32_t)(s.heap_used * 100 /
                                  (s.heap_used + s.heap_free));
        shell_print(sh, "  Usage: %u %%", pct);
    }
    return 0;
}
SHELL_CMD_REGISTER(mem, NULL, "Heap memory usage", cmd_mem);

// ── threads ───────────────────────────────────────────────────────────────────

static void print_thread_cb(const struct k_thread* t, void* user_data)
{
    const struct shell* sh = (const struct shell*)user_data;
    const char* name = k_thread_name_get((struct k_thread*)t);
    if (!name || name[0] == '\0') name = "(unnamed)";

    size_t unused = 0;
#if defined(CONFIG_THREAD_STACK_INFO)
    k_thread_stack_space_get(t, &unused);
#endif

    shell_print(sh, "  %-16s prio=%-3d  stack_free=%zu B  state=0x%x",
                name, t->base.prio, unused, t->base.thread_state);
}

static int cmd_threads(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Threads ---");
    k_thread_foreach(print_thread_cb, (void*)sh);
    return 0;
}
SHELL_CMD_REGISTER(threads, NULL, "List all threads with stack info", cmd_threads);

// ── vehicle info ──────────────────────────────────────────────────────────────

static int cmd_vehicle_info(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Vehicle: Heavy-Duty Electric Vehicle ---");
    shell_print(sh, "  Mass          : 218000 kg (218 t)");
    shell_print(sh, "  Wheelbase     : 6.30 m");
    shell_print(sh, "  Track width   : 7.20 m");
    shell_print(sh, "  Wheel radius  : 1.930 m");
    shell_print(sh, "  Motor torque  : 145000 Nm");
    shell_print(sh, "  Motor power   : 2013000 W (2013 kW)");
    shell_print(sh, "  Gear ratio    : 28.0");
    shell_print(sh, "  Battery       : 1650 kWh");
    shell_print(sh, "  Vmax          : 17.8 m/s (64 km/h)");
    shell_print(sh, "  Surface mu    : %.2f  (live)", g_surface_mu);
    return 0;
}
SHELL_CMD_REGISTER(vehicle, NULL, "Vehicle info", cmd_vehicle_info);

// ── network ───────────────────────────────────────────────────────────────────

static int cmd_network_mac(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    struct net_if* iface = net_if_get_default();
    if (!iface) {
        shell_error(sh, "No network interface found");
        return -ENODEV;
    }
    struct net_linkaddr* ll = net_if_get_link_addr(iface);
    shell_print(sh, "MAC : %02X:%02X:%02X:%02X:%02X:%02X",
                ll->addr[0], ll->addr[1], ll->addr[2],
                ll->addr[3], ll->addr[4], ll->addr[5]);
    shell_print(sh, "IP  : 192.168.1.80");
    shell_print(sh, "Port: 443  (https://192.168.1.80)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(network_cmds,
    SHELL_CMD(mac, NULL, "Print MAC and IP address", cmd_network_mac),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(network, &network_cmds, "Network commands", NULL);

// ── system ────────────────────────────────────────────────────────────────────

static int cmd_system_uptime(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    uint32_t ms  = k_uptime_get_32();
    uint32_t s   = ms / 1000;
    uint32_t h   = s / 3600;
    uint32_t m   = (s % 3600) / 60;
    uint32_t sec = s % 60;
    shell_print(sh, "Uptime: %02u:%02u:%02u  (%u ms)", h, m, sec, ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(system_cmds,
    SHELL_CMD(uptime, NULL, "Print system uptime", cmd_system_uptime),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(system, &system_cmds, "System commands", NULL);

// ── mqtt ──────────────────────────────────────────────────────────────────────

#include "mqtt/mqtt_client.hpp"

extern atomic_t g_ctrl_source;
extern atomic_t g_mqtt_rx_count;

static int cmd_mqtt_status(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    const char* src_str =
        (atomic_get(&g_ctrl_source) == CTRL_MQTT) ? "MQTT" :
        (atomic_get(&g_ctrl_source) == CTRL_HTTP) ? "HTTP" : "CAN";
    shell_print(sh, "Broker     : %s:%d", g_mqtt_broker_addr, g_mqtt_broker_port);
    shell_print(sh, "Connected  : %s", g_mqtt_connected ? "yes" : "no");
    shell_print(sh, "MQTT RX    : %u", (uint32_t)atomic_get(&g_mqtt_rx_count));
    shell_print(sh, "Ctrl source: %s", src_str);
    return 0;
}

static int cmd_mqtt_reconnect(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    g_mqtt_reconnect_req = true;
    shell_print(sh, "MQTT: reconnect requested → %s:%d",
                g_mqtt_broker_addr, g_mqtt_broker_port);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mqtt_cmds,
    SHELL_CMD(status,    NULL, "MQTT broker status and RX count", cmd_mqtt_status),
    SHELL_CMD(reconnect, NULL, "Force MQTT reconnect",            cmd_mqtt_reconnect),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(mqtt, &mqtt_cmds, "MQTT client commands", NULL);
