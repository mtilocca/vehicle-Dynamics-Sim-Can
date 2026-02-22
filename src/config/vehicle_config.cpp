// src/config/vehicle_config.cpp
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace config {

VehicleConfig VehicleConfig::load(const std::string& yaml_path) {
    std::ifstream file_check(yaml_path);
    if (!file_check.good()) {
        LOG_WARN("[VehicleConfig] File not found: %s — using defaults", yaml_path.c_str());
        return get_default();
    }
    file_check.close();

    LOG_INFO("[VehicleConfig] Loading: %s", yaml_path.c_str());

    try {
        YAML::Node cfg = YAML::LoadFile(yaml_path);
        VehicleConfig vehicle;

        // ------------------------------------------------------------------
        // Metadata
        // ------------------------------------------------------------------
        if (cfg["vehicle"]) {
            auto v = cfg["vehicle"];
            vehicle.name         = v["name"].as<std::string>("Unknown Vehicle");
            vehicle.description  = v["description"].as<std::string>("");
            vehicle.manufacturer = v["manufacturer"].as<std::string>("");
            vehicle.year         = v["year"].as<int>(2025);
        }

        // ------------------------------------------------------------------
        // Geometry
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["geometry"]) {
            auto geo = cfg["vehicle"]["geometry"];
            vehicle.params.drive.mass_kg       = geo["mass_kg"].as<double>(218000.0);
            vehicle.params.wheelbase_m         = geo["wheelbase_m"].as<double>(6.30);
            vehicle.params.track_width_m       = geo["track_width_m"].as<double>(7.20);
            vehicle.params.drive.wheel_radius_m = geo["wheel_radius_m"].as<double>(1.93);

            vehicle.params.geometry.cg_height_m      = geo["cg_height_m"].as<double>(3.20);
            vehicle.params.geometry.cg_to_front_m    = geo["cg_to_front_axle_m"].as<double>(2.52);
            vehicle.params.geometry.cg_to_rear_m     = geo["cg_to_rear_axle_m"].as<double>(3.78);
            vehicle.params.geometry.yaw_inertia_kgm2 = geo["yaw_inertia_kgm2"].as<double>(8500000.0);
        }

        // ------------------------------------------------------------------
        // Drivetrain
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["drivetrain"]) {
            auto dt = cfg["vehicle"]["drivetrain"];
            vehicle.params.drive.motor_torque_max_nm = dt["motor_torque_max_nm"].as<double>(9500.0);
            vehicle.params.drive.motor_power_max_w   = dt["motor_power_max_w"].as<double>(2800000.0);
            vehicle.params.drive.gear_ratio          = dt["gear_ratio"].as<double>(25.0);
            vehicle.params.drive.drivetrain_eff      = dt["efficiency"].as<double>(0.92);
        }

        // ------------------------------------------------------------------
        // Brakes
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["brakes"]) {
            auto br = cfg["vehicle"]["brakes"];
            vehicle.params.drive.brake_torque_max_nm = br["brake_torque_max_nm"].as<double>(80000.0);
            vehicle.params.drive.brake_bias_front    = br["brake_bias_front"].as<double>(0.40);
        }

        // ------------------------------------------------------------------
        // Resistance
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["resistance"]) {
            auto res = cfg["vehicle"]["resistance"];
            vehicle.params.drive.drag_c = res["drag_coefficient"].as<double>(2.5);
            vehicle.params.drive.roll_c = res["rolling_resistance"].as<double>(1500.0);
        }

        // ------------------------------------------------------------------
        // Limits
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["limits"]) {
            auto lim = cfg["vehicle"]["limits"];
            vehicle.params.drive.v_max_mps  = lim["v_max_mps"].as<double>(17.78);
            vehicle.params.drive.v_stop_eps = lim["v_stop_eps"].as<double>(0.5);
            vehicle.params.drive.v_kinematic_blend_mps = lim["v_kinematic_blend_mps"].as<double>(3.0);
        }

        // ------------------------------------------------------------------
        // Linear tyre dynamics
        // ------------------------------------------------------------------
        if (cfg["vehicle"] && cfg["vehicle"]["dynamics"]) {
            auto dyn = cfg["vehicle"]["dynamics"];
            vehicle.params.drive.mu_surface   = dyn["mu_surface"].as<double>(0.72);
            vehicle.params.drive.Cy_front_Npm = dyn["Cy_front_Npm"].as<double>(2500000.0);
            vehicle.params.drive.Cy_rear_Npm  = dyn["Cy_rear_Npm"].as<double>(2000000.0);
            vehicle.params.drive.tire_relax_tau_s = dyn["tire_relax_tau_s"].as<double>(0.3);
        }

        vehicle.validate();
        LOG_INFO("[VehicleConfig] Loaded: %s", vehicle.name.c_str());
        return vehicle;

    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("[VehicleConfig] YAML error: ") + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("[VehicleConfig] Load error: ") + e.what());
    }
}

VehicleConfig VehicleConfig::get_default() {
    VehicleConfig vehicle;

    vehicle.name         = "XCMG XDE320 (defaults)";
    vehicle.description  = "Built-in heavy truck defaults";
    vehicle.manufacturer = "XCMG";
    vehicle.year         = 2025;

    vehicle.params.drive.mass_kg            = 218000.0;
    vehicle.params.wheelbase_m              = 6.30;
    vehicle.params.track_width_m            = 7.20;
    vehicle.params.drive.wheel_radius_m     = 1.93;

    vehicle.params.geometry.cg_height_m      = 3.20;
    vehicle.params.geometry.cg_to_front_m    = 2.52;
    vehicle.params.geometry.cg_to_rear_m     = 3.78;
    vehicle.params.geometry.yaw_inertia_kgm2 = 8500000.0;

    vehicle.params.drive.motor_torque_max_nm = 9500.0;
    vehicle.params.drive.motor_power_max_w   = 2800000.0;
    vehicle.params.drive.gear_ratio          = 25.0;
    vehicle.params.drive.drivetrain_eff      = 0.92;

    vehicle.params.drive.brake_torque_max_nm = 80000.0;
    vehicle.params.drive.brake_bias_front    = 0.40;

    vehicle.params.drive.drag_c = 2.5;
    vehicle.params.drive.roll_c = 1500.0;

    vehicle.params.drive.v_max_mps  = 16.667;  // 60 km/h
    vehicle.params.drive.v_stop_eps = 0.5;
    vehicle.params.drive.v_kinematic_blend_mps = 3.0;

    vehicle.params.drive.mu_surface   = 0.72;
    vehicle.params.drive.Cy_front_Npm = 2500000.0;
    vehicle.params.drive.Cy_rear_Npm  = 2000000.0;
    vehicle.params.drive.tire_relax_tau_s = 0.3;

    return vehicle;
}

void VehicleConfig::validate() const {
    if (params.drive.mass_kg <= 0.0)
        throw std::runtime_error("mass_kg must be > 0");
    if (params.wheelbase_m <= 0.0)
        throw std::runtime_error("wheelbase_m must be > 0");
    if (params.drive.wheel_radius_m <= 0.0)
        throw std::runtime_error("wheel_radius_m must be > 0");
    if (params.drive.motor_torque_max_nm <= 0.0)
        throw std::runtime_error("motor_torque_max_nm must be > 0");
    if (params.drive.motor_power_max_w <= 0.0)
        throw std::runtime_error("motor_power_max_w must be > 0");
    if (params.drive.gear_ratio <= 0.0)
        throw std::runtime_error("gear_ratio must be > 0");
    if (params.drive.drivetrain_eff <= 0.0 || params.drive.drivetrain_eff > 1.0)
        throw std::runtime_error("drivetrain_eff must be (0, 1]");
    if (params.drive.v_max_mps <= 0.0)
        throw std::runtime_error("v_max_mps must be > 0");
    if (params.drive.mu_surface <= 0.0 || params.drive.mu_surface > 2.0)
        throw std::runtime_error("mu_surface must be (0, 2]");
    if (params.drive.Cy_front_Npm <= 0.0 || params.drive.Cy_rear_Npm <= 0.0)
        throw std::runtime_error("Cy_front/rear_Npm must be > 0");
}

void VehicleConfig::print_summary() const {
    LOG_INFO("========================================");
    LOG_INFO("Vehicle: %s", name.c_str());
    LOG_INFO("  Mass:       %.0f kg", params.drive.mass_kg);
    LOG_INFO("  Wheelbase:  %.2f m  Track: %.2f m", params.wheelbase_m, params.track_width_m);
    LOG_INFO("  Power:      %.0f kW  Torque: %.0f Nm",
             params.drive.motor_power_max_w / 1000.0, params.drive.motor_torque_max_nm);
    LOG_INFO("  Gear ratio: %.1f   Eff: %.2f", params.drive.gear_ratio, params.drive.drivetrain_eff);
    LOG_INFO("  v_max:      %.1f m/s (%.0f km/h)",
             params.drive.v_max_mps, params.drive.v_max_mps * 3.6);
    LOG_INFO("  mu_surface: %.2f   Cy_f: %.0f N/rad  Cy_r: %.0f N/rad",
             params.drive.mu_surface, params.drive.Cy_front_Npm, params.drive.Cy_rear_Npm);
    LOG_INFO("  CoG:        h=%.2f m  lf=%.2f m  lr=%.2f m",
             params.geometry.cg_height_m,
             params.geometry.cg_to_front_m,
             params.geometry.cg_to_rear_m);
    LOG_INFO("========================================");
}

} // namespace config
