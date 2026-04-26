// zephyr/src/config/vehicle_config_zephyr.hpp
// Compile-time Heavy-Duty Electric Vehicle parameters for the Zephyr build.
// No std::string, no heap, no STL — mirrors plant/config/vehicle_config.cpp.
//
// To change vehicle parameters: edit this file and rebuild.
// Single source of truth for all Zephyr modules (plant_thread, can_tx, shell).
#pragma once

#include "plant/plant_model.hpp"

namespace config {

// Default surface friction coefficient used at boot.
static constexpr double kDefaultSurfaceMu = 0.72;

// Returns a fully populated PlantModelParams for the HDV.
// Inline so the compiler folds all numeric references to constants.
inline plant::PlantModelParams hdv_default_params()
{
    plant::PlantModelParams p;

    p.wheelbase_m                   = 6.30;
    p.track_width_m                 = 7.20;
    p.geometry.cg_height_m          = 3.20;
    p.geometry.cg_to_front_m        = 2.52;
    p.geometry.cg_to_rear_m         = 3.78;
    p.geometry.yaw_inertia_kgm2     = 8'500'000.0;

    p.drive.mass_kg                  = 218'000.0;
    p.drive.wheel_radius_m           = 1.93;
    p.drive.motor_torque_max_nm      = 145'000.0;
    p.drive.motor_power_max_w        = 2'013'000.0;
    p.drive.gear_ratio               = 28.0;
    p.drive.drivetrain_eff           = 0.92;
    p.drive.brake_torque_max_nm      = 2'500'000.0;
    p.drive.regen_eff_active         = 0.65;
    p.drive.regen_eff_coast          = 0.02;
    p.drive.drag_c                   = 1.85;
    p.drive.roll_c                   = 9'500.0;
    p.drive.v_max_mps                = 17.78;
    p.drive.v_stop_eps               = 0.5;

    p.motor_params.max_power_kW      = 2013.0;
    p.motor_params.max_torque_nm     = 145'000.0;
    p.motor_params.efficiency        = 0.92;

    p.battery_params.capacity_kWh            = 1650.0;
    p.battery_params.nominal_voltage_v       = 1200.0;
    p.battery_params.max_charge_power_kW     = 600.0;
    p.battery_params.max_discharge_power_kW  = 2400.0;
    p.battery_params.efficiency_charge       = 0.91;
    p.battery_params.efficiency_discharge    = 0.93;
    p.battery_params.min_soc                 = 0.18;
    p.battery_params.max_soc                 = 0.88;

    p.dynamic_config.enabled    = true;
    p.dynamic_config.surface_mu = kDefaultSurfaceMu;

    return p;
}

} // namespace config
