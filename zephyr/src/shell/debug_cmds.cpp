// zephyr/src/shell/debug_cmds.cpp
// Zephyr shell commands for runtime inspection.
// Register with CONFIG_SHELL=y in prj.conf.
//
// Usage over USART3 (screen / picocom at 115200):
//   plant state      — dump all PlantState fields
//   plant mu <val>   — change surface friction coefficient
//   plant reset      — zero plant state
//   can stats        — TX/RX frame counts
//   can rx_frame     — last decoded ACTUATOR_CMD_1
//   vehicle info     — XCMG parameter summary

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <stdlib.h>   /* strtof — use C header, not <cstdlib> (-nostdinc++) */

#include "plant/plant_main/plant_state.hpp"   // pulls in sim/actuator_cmd.hpp
#include "sim/actuator_cmd.hpp"
#include "can/can_map_static.hpp"

LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);

// ── Shared state — defined in main.cpp ───────────────────────────────────────
extern plant::PlantState g_state;
extern sim::ActuatorCmd  g_cmd;
extern struct k_mutex    g_state_mutex;
extern struct k_mutex    g_cmd_mutex;

// ── CAN counters — updated in Phase 3 by zephyr_can_iface.cpp ────────────────
extern volatile uint32_t g_can_tx_count;
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_timeout_count;
extern volatile double   g_last_rx_t;

// ── Surface friction — read by plant thread in Phase 4 ───────────────────────
extern double g_surface_mu;

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
    shell_print(sh, "  surface_mu = %.2f",    s.surface_mu);
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
    shell_print(sh, "Surface mu set to %.2f (takes effect on next plant step)", (double)mu);
    return 0;
}

static int cmd_plant_reset(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    g_state = plant::PlantState{};
    k_mutex_unlock(&g_state_mutex);
    shell_print(sh, "Plant state reset to zero");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(plant_cmds,
    SHELL_CMD(state, NULL, "Dump plant state",   cmd_plant_state),
    SHELL_CMD(mu,    NULL, "Set surface mu",      cmd_plant_mu),
    SHELL_CMD(reset, NULL, "Reset plant state",   cmd_plant_reset),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(plant, &plant_cmds, "Plant model commands", NULL);

// ── can stats / rx_frame ──────────────────────────────────────────────────────

static int cmd_can_stats(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- CAN Stats ---");
    shell_print(sh, "  TX frames  : %u", g_can_tx_count);
    shell_print(sh, "  RX frames  : %u", g_can_rx_count);
    shell_print(sh, "  RX timeouts: %u", g_can_timeout_count);
    shell_print(sh, "  Last RX    : %.3f s", g_last_rx_t);
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
                    (unsigned)f.id,
                    f.name,
                    (unsigned)f.sig_count,
                    (unsigned)f.cycle_ms);
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(can_cmds,
    SHELL_CMD(stats,    NULL, "CAN TX/RX counters",        cmd_can_stats),
    SHELL_CMD(rx_frame, NULL, "Last decoded actuator cmd",  cmd_can_rx_frame),
    SHELL_CMD(map,      NULL, "Dump static CAN frame map",  cmd_can_map),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can, &can_cmds, "CAN bus commands", NULL);

// ── vehicle info ──────────────────────────────────────────────────────────────
// Hardcoded XCMG XDE320 summary for Phase 1.
// Phase 4 will replace this with VehicleConfig::get_default().

static int cmd_vehicle_info(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Vehicle: XCMG XDE320 Electric ---");
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
    shell_print(sh, "IP  : 192.168.1.100");
    shell_print(sh, "Port: 80  (http://192.168.1.100)");
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
    uint32_t ms = k_uptime_get_32();
    uint32_t s  = ms / 1000;
    uint32_t h  = s / 3600;
    uint32_t m  = (s % 3600) / 60;
    uint32_t sec = s % 60;
    shell_print(sh, "Uptime: %02u:%02u:%02u  (%u ms)", h, m, sec, ms);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(system_cmds,
    SHELL_CMD(uptime, NULL, "Print system uptime", cmd_system_uptime),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(system, &system_cmds, "System commands", NULL);
