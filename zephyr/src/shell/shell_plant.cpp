// zephyr/src/shell/shell_plant.cpp
// Shell: plant state, mu, reset, inject subcommands.

#include <zephyr/logging/log.h>
#include <stdlib.h>

#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

static int cmd_plant_state(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    plant::PlantState s;
    { hdv::MutexGuard g(g_sim_plant_mtx); s = g_sim_bus.plant; }

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
    g_ctrl_bus.surface_mu = static_cast<double>(mu);
    shell_print(sh, "Surface mu set to %.2f", (double)mu);
    return 0;
}

static int cmd_plant_reset(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    if (!shell_cmds::require_login(sh)) return -EACCES;

    { hdv::MutexGuard g(g_sim_plant_mtx); g_sim_bus.plant = plant::PlantState{}; }
    { hdv::MutexGuard g(g_sim_cmd_mtx);  g_sim_bus.cmd   = sim::ActuatorCmd{};  }
    shell_print(sh, "Plant state and command reset to zero");
    return 0;
}

static int cmd_plant_inject(const struct shell* sh, size_t argc, char** argv)
{
    if (!shell_cmds::require_login(sh)) return -EACCES;

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

    { hdv::MutexGuard g(g_sim_cmd_mtx); g_sim_bus.cmd = cmd; }

    shell_print(sh, "Injected: steer=%.1f deg  torque=%.0f Nm  brake=%.2f  enable=%d",
                steer, torque, brake, (int)enable);
    shell_print(sh, "  (watchdog: resend within 500 ms to maintain)");
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(plant_cmds,
    SHELL_CMD(state,  NULL, "Dump plant state",                     cmd_plant_state),
    SHELL_CMD(mu,     NULL, "Set surface mu",                       cmd_plant_mu),
    SHELL_CMD(reset,  NULL, "Reset plant state (requires login)",   cmd_plant_reset),
    SHELL_CMD(inject, NULL, "Inject actuator cmd (requires login)", cmd_plant_inject),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(plant, &plant_cmds, "Plant model commands", NULL);
