// src/config/vehicle_config.cpp
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <stdexcept>

namespace config {

VehicleConfig VehicleConfig::load(const std::string& /*yaml_path*/) {
    LOG_WARN("[VehicleConfig] YAML loading disabled — using hardcoded XCMG XDE320 defaults");
    return get_default();
}

VehicleConfig VehicleConfig::get_default() {
    VehicleConfig vehicle;

    // XCMG XDE320 Electric Dump Truck
    vehicle.name = "XCMG XDE320 Electric";
    vehicle.description = "220-ton electric mining dump truck";
    vehicle.manufacturer = "XCMG";
    vehicle.year = 2025;

    // ========================================================================
    // Geometry
    // ========================================================================
    vehicle.params.drive.mass_kg             = 218000.0;
    vehicle.params.wheelbase_m               = 6.30;
    vehicle.params.track_width_m             = 7.20;
    vehicle.params.drive.wheel_radius_m      = 1.93;

    vehicle.params.geometry.cg_height_m      = 3.20;
    vehicle.params.geometry.cg_to_front_m    = 2.52;   // 40% of wheelbase
    vehicle.params.geometry.cg_to_rear_m     = 3.78;   // 60% of wheelbase
    vehicle.params.geometry.yaw_inertia_kgm2 = 8500000.0;

    // ========================================================================
    // Drivetrain
    // ========================================================================
    vehicle.params.drive.motor_torque_max_nm  = 145000.0;
    vehicle.params.drive.motor_power_max_w    = 2013000.0;
    vehicle.params.drive.gear_ratio           = 28.0;
    vehicle.params.drive.drivetrain_eff       = 0.92;

    vehicle.params.motor_params.max_power_kW  = 2013.0;
    vehicle.params.motor_params.max_torque_nm = 145000.0;
    vehicle.params.motor_params.efficiency    = 0.92;

    // ========================================================================
    // Brakes
    // ========================================================================
    vehicle.params.drive.brake_torque_max_nm  = 180000.0;
    vehicle.params.drive.regen_eff_active     = 0.65;
    vehicle.params.drive.regen_eff_coast      = 0.02;

    // ========================================================================
    // Battery
    // ========================================================================
    vehicle.params.battery_params.capacity_kWh            = 1650.0;
    vehicle.params.battery_params.nominal_voltage_v        = 1200.0;
    vehicle.params.battery_params.max_charge_power_kW      = 600.0;
    vehicle.params.battery_params.max_discharge_power_kW   = 2400.0;
    vehicle.params.battery_params.efficiency_charge        = 0.91;
    vehicle.params.battery_params.efficiency_discharge     = 0.93;
    vehicle.params.battery_params.min_soc                  = 0.18;
    vehicle.params.battery_params.max_soc                  = 0.88;

    // ========================================================================
    // Resistance
    // ========================================================================
    vehicle.params.drive.drag_c   = 1.85;
    vehicle.params.drive.roll_c   = 9500.0;

    // ========================================================================
    // Limits
    // ========================================================================
    vehicle.params.drive.v_max_mps   = 17.78;   // 64 km/h
    vehicle.params.drive.v_stop_eps  = 0.5;

    // ========================================================================
    // Wheel rotational dynamics
    // ========================================================================
    vehicle.wheel_params.inertia_kgm2 = 1000.0;

    // ========================================================================
    // Tires — Dugoff model, gravel compact (Pilbara haul roads)
    // ========================================================================
    vehicle.tire_params.model              = "dugoff";
    vehicle.tire_params.radius_m           = 1.93;
    vehicle.tire_params.width_m            = 1.016;
    vehicle.tire_params.Cx_base            = 280000.0;
    vehicle.tire_params.Cy_base            = 220000.0;
    vehicle.tire_params.Fz_ref             = 800000.0;
    vehicle.tire_params.load_exponent      = 0.50;
    vehicle.tire_params.surface.name       = "gravel_compact";
    vehicle.tire_params.surface.mu_peak    = 0.72;
    vehicle.tire_params.surface.mu_slide   = 0.65;
    vehicle.tire_params.sigma_x_max        = 0.95;
    vehicle.tire_params.sigma_y_max        = 0.50;
    vehicle.tire_params.v_min_for_slip_calc = 0.5;

    // ========================================================================
    // Dynamic model
    // ========================================================================
    vehicle.params.dynamic_config.enabled    = true;
    vehicle.params.dynamic_config.surface_mu = 0.72;

    return vehicle;
}

void VehicleConfig::validate() const {
    if (params.drive.mass_kg <= 0.0)
        throw std::runtime_error("Invalid mass_kg: must be > 0");
    if (params.wheelbase_m <= 0.0)
        throw std::runtime_error("Invalid wheelbase_m: must be > 0");
    if (params.drive.wheel_radius_m <= 0.0)
        throw std::runtime_error("Invalid wheel_radius_m: must be > 0");
    if (wheel_params.inertia_kgm2 <= 0.0)
        throw std::runtime_error("Invalid wheel inertia_kgm2: must be > 0");
    if (params.drive.motor_torque_max_nm <= 0.0)
        throw std::runtime_error("Invalid motor_torque_max_nm: must be > 0");
    if (params.drive.motor_power_max_w <= 0.0)
        throw std::runtime_error("Invalid motor_power_max_w: must be > 0");
    if (params.drive.gear_ratio <= 0.0)
        throw std::runtime_error("Invalid gear_ratio: must be > 0");
    if (params.drive.drivetrain_eff <= 0.0 || params.drive.drivetrain_eff > 1.0)
        throw std::runtime_error("Invalid drivetrain_eff: must be 0 < eff <= 1");
    if (params.battery_params.capacity_kWh <= 0.0)
        throw std::runtime_error("Invalid battery capacity_kWh: must be > 0");
    if (params.battery_params.min_soc < 0.0 || params.battery_params.min_soc >= params.battery_params.max_soc)
        throw std::runtime_error("Invalid SOC range: 0 <= min_soc < max_soc <= 1");
    if (params.battery_params.max_soc > 1.0)
        throw std::runtime_error("Invalid max_soc: must be <= 1.0");
    if (params.drive.v_max_mps <= 0.0)
        throw std::runtime_error("Invalid v_max_mps: must be > 0");
    if (tire_params.radius_m <= 0.0)
        throw std::runtime_error("Invalid tire radius_m: must be > 0");
    if (tire_params.Cx_base <= 0.0 || tire_params.Cy_base <= 0.0)
        throw std::runtime_error("Invalid slip stiffness: Cx, Cy must be > 0");
    if (tire_params.surface.mu_peak <= 0.0 || tire_params.surface.mu_peak > 2.0)
        throw std::runtime_error("Invalid mu_peak: must be 0 < mu <= 2.0");
    if (tire_params.surface.mu_slide <= 0.0 || tire_params.surface.mu_slide > tire_params.surface.mu_peak)
        throw std::runtime_error("Invalid mu_slide: must be 0 < mu_slide <= mu_peak");

    LOG_DEBUG("[VehicleConfig] Validation passed");
}

void VehicleConfig::print_summary() const {
    LOG_INFO("========================================");
    LOG_INFO("Vehicle Configuration Summary");
    LOG_INFO("========================================");
    LOG_INFO("Name: %s", name.c_str());
    if (!description.empty())
        LOG_INFO("Description: %s", description.c_str());
    LOG_INFO("----------------------------------------");
    LOG_INFO("Mass: %.0f kg (%.0f tons)", params.drive.mass_kg, params.drive.mass_kg / 1000.0);
    LOG_INFO("Motor Power: %.0f kW (%.0f hp)",
             params.drive.motor_power_max_w / 1000.0,
             params.drive.motor_power_max_w / 745.7);
    LOG_INFO("Motor Torque: %.0f Nm", params.drive.motor_torque_max_nm);
    LOG_INFO("Battery: %.1f kWh", params.battery_params.capacity_kWh);
    LOG_INFO("Max Speed: %.1f m/s (%.0f km/h)",
             params.drive.v_max_mps, params.drive.v_max_mps * 3.6);
    LOG_INFO("CoG: h=%.2f m, front=%.2f m, rear=%.2f m",
             params.geometry.cg_height_m,
             params.geometry.cg_to_front_m,
             params.geometry.cg_to_rear_m);
    LOG_INFO("Wheel inertia: %.1f kg*m^2", wheel_params.inertia_kgm2);
    LOG_INFO("----------------------------------------");
    LOG_INFO("Tire Model: %s", tire_params.model.c_str());
    LOG_INFO("Surface: %s (mu_peak=%.2f, mu_slide=%.2f)",
             tire_params.surface.name.c_str(),
             tire_params.surface.mu_peak,
             tire_params.surface.mu_slide);
    LOG_INFO("========================================");
}

} // namespace config
