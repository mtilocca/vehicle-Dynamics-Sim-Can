// zephyr/src/shell/shell_vehicle.cpp
// Shell: vehicle info command.

#include <zephyr/logging/log.h>

#include "config/vehicle_config_zephyr.hpp"
#include "shell_shared.hpp"

LOG_MODULE_DECLARE(hdv_sim, LOG_LEVEL_INF);

// File-scope to avoid __cxa_guard_acquire (Zephyr newlib limitation).
static const plant::PlantModelParams s_hdv = config::hdv_default_params();

static int cmd_vehicle_info(const struct shell* sh, size_t argc, char** argv)
{
    (void)argc; (void)argv;
    shell_print(sh, "--- Vehicle: Heavy-Duty Electric Vehicle ---");
    shell_print(sh, "  Mass          : %.0f kg (%.0f t)", s_hdv.drive.mass_kg, s_hdv.drive.mass_kg / 1000.0);
    shell_print(sh, "  Wheelbase     : %.2f m",            s_hdv.wheelbase_m);
    shell_print(sh, "  Track width   : %.2f m",            s_hdv.track_width_m);
    shell_print(sh, "  Wheel radius  : %.3f m",            s_hdv.drive.wheel_radius_m);
    shell_print(sh, "  Motor torque  : %.0f Nm",           s_hdv.drive.motor_torque_max_nm);
    shell_print(sh, "  Motor power   : %.0f W (%.0f kW)",  s_hdv.drive.motor_power_max_w,
                                                            s_hdv.drive.motor_power_max_w / 1000.0);
    shell_print(sh, "  Gear ratio    : %.1f",              s_hdv.drive.gear_ratio);
    shell_print(sh, "  Battery       : %.0f kWh",          s_hdv.battery_params.capacity_kWh);
    shell_print(sh, "  Vmax          : %.1f m/s (%.0f km/h)", s_hdv.drive.v_max_mps,
                                                              s_hdv.drive.v_max_mps * 3.6);
    shell_print(sh, "  Surface mu    : %.2f  (live)", g_ctrl_bus.surface_mu);
    return 0;
}
SHELL_CMD_REGISTER(vehicle, NULL, "Vehicle info", cmd_vehicle_info);
