// zephyr/src/shell/debug_cmds.cpp
// Zephyr shell commands for runtime inspection.
// Register with CONFIG_SHELL=y in prj.conf.
//
// Usage over USART3 (minicom / picocom at 115200):
//   plant state      — dump all PlantState fields
//   plant mu <val>   — change surface friction coefficient
//   plant reset      — zero plant state
//   can stats        — TX/RX frame counts
//   can rx_frame     — last decoded ACTUATOR_CMD_1
//   vehicle info     — XCMG parameter summary

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "config/vehicle_config.hpp"

LOG_MODULE_DECLARE(xcmg_sim);

// ── Shared state — declared in main.cpp ──────────────────────────────────────
extern plant::PlantState g_state;
extern sim::ActuatorCmd  g_cmd;
extern struct k_mutex    g_state_mutex;
extern struct k_mutex    g_cmd_mutex;

// ── Counters (incremented in zephyr_can_iface.cpp) ───────────────────────────
extern volatile uint32_t g_can_tx_count;
extern volatile uint32_t g_can_rx_count;
extern volatile uint32_t g_can_timeout_count;
extern volatile double   g_last_rx_t;

// ── plant state ──────────────────────────────────────────────────────────────

static int cmd_plant_state(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    k_mutex_lock(&g_state_mutex, K_FOREVER);
    plant::PlantState s = g_state;
    k_mutex_unlock(&g_state_mutex);

    shell_print(sh, "--- Plant State ---");
    shell_print(sh, "  t_s      = %.3f s",  s.t_s);
    shell_print(sh, "  vx       = %.3f m/s", s.vx_mps);
    shell_print(sh, "  vy       = %.3f m/s", s.vy_mps);
    shell_print(sh, "  yaw      = %.3f deg", s.yaw_rad * 57.2958);
    shell_print(sh, "  yaw_rate = %.3f deg/s", s.yaw_rate_rps * 57.2958);
    shell_print(sh, "  x        = %.2f m",  s.x_m);
    shell_print(sh, "  y        = %.2f m",  s.y_m);
    shell_print(sh, "  steer    = %.2f deg", s.steer_deg);
    shell_print(sh, "  soc      = %.1f %%", s.soc_pct);
    shell_print(sh, "  omega_fl = %.2f rad/s", s.omega_fl_rps);
    shell_print(sh, "  omega_fr = %.2f rad/s", s.omega_fr_rps);
    shell_print(sh, "  omega_rl = %.2f rad/s", s.omega_rl_rps);
    shell_print(sh, "  omega_rr = %.2f rad/s", s.omega_rr_rps);
    shell_print(sh, "  Fx       = %.0f N",  s.Fx_N);
    shell_print(sh, "  Fy       = %.0f N",  s.Fy_N);
    return 0;
}

static int cmd_plant_mu(const struct shell* sh, size_t argc, char** argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: plant mu <value>");
        return -EINVAL;
    }
    float mu = strtof(argv[1], nullptr);
    if (mu < 0.1f || mu > 1.0f) {
        shell_error(sh, "mu must be between 0.1 and 1.0");
        return -EINVAL;
    }
    // Surface friction is set on the plant model's tyre params.
    // Access via a global pointer (set in main.cpp).
    extern plant::PlantModel* g_plant;
    if (g_plant) {
        g_plant->set_surface_mu(static_cast<double>(mu));
        shell_print(sh, "Surface mu set to %.2f", mu);
    }
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
    SHELL_CMD(state, nullptr, "Dump plant state",  cmd_plant_state),
    SHELL_CMD(mu,    nullptr, "Set surface mu",     cmd_plant_mu),
    SHELL_CMD(reset, nullptr, "Reset plant state",  cmd_plant_reset),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(plant, &plant_cmds, "Plant model commands", nullptr);

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
    shell_print(sh, "  system_enable     = %d",   c.system_enable);
    shell_print(sh, "  gear_position     = %d",   c.gear_position);
    shell_print(sh, "  drive_torque_nm   = %.1f", c.drive_torque_cmd_nm);
    shell_print(sh, "  brake_pct         = %.2f", c.brake_cmd_pct);
    shell_print(sh, "  steer_deg         = %.2f", c.steer_cmd_deg);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(can_cmds,
    SHELL_CMD(stats,    nullptr, "CAN TX/RX counters",       cmd_can_stats),
    SHELL_CMD(rx_frame, nullptr, "Last decoded actuator cmd", cmd_can_rx_frame),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(can, &can_cmds, "CAN bus commands", nullptr);

// ── vehicle info ──────────────────────────────────────────────────────────────

static int cmd_vehicle_info(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    auto v = config::VehicleConfig::get_default();
    shell_print(sh, "--- Vehicle: %s ---", v.name.c_str());
    shell_print(sh, "  Mass          : %.0f kg",   v.params.drive.mass_kg);
    shell_print(sh, "  Wheelbase     : %.2f m",    v.params.wheelbase_m);
    shell_print(sh, "  Track width   : %.2f m",    v.params.track_width_m);
    shell_print(sh, "  Wheel radius  : %.3f m",    v.params.drive.wheel_radius_m);
    shell_print(sh, "  Motor torque  : %.0f Nm",   v.params.drive.motor_torque_max_nm);
    shell_print(sh, "  Motor power   : %.0f W",    v.params.drive.motor_power_max_w);
    shell_print(sh, "  Gear ratio    : %.1f",      v.params.drive.gear_ratio);
    shell_print(sh, "  Battery       : %.0f kWh",  v.params.battery_params.capacity_kWh);
    shell_print(sh, "  Vmax          : %.1f m/s",  v.params.drive.v_max_mps);
    shell_print(sh, "  Surface mu    : %.2f",      v.tire_params.surface.mu_peak);
    return 0;
}

SHELL_CMD_REGISTER(vehicle, nullptr, "Vehicle info", cmd_vehicle_info);
