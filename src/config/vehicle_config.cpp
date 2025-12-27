// src/config/vehicle_config.cpp
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

namespace config {

VehicleConfig VehicleConfig::load(const std::string& yaml_path) {
    // Check if file exists
    std::ifstream file_check(yaml_path);
    if (!file_check.good()) {
        LOG_WARN("[VehicleConfig] File not found: %s", yaml_path.c_str());
        LOG_WARN("[VehicleConfig] Using default configuration");
        return get_default();
    }
    file_check.close();
    
    LOG_INFO("[VehicleConfig] Loading vehicle config from: %s", yaml_path.c_str());
    
    try {
        YAML::Node config = YAML::LoadFile(yaml_path);
        
        VehicleConfig vehicle;
        
        // ====================================================================
        // Parse vehicle metadata
        // ====================================================================
        if (config["vehicle"]) {
            auto v = config["vehicle"];
            vehicle.name = v["name"].as<std::string>("Unknown Vehicle");
            vehicle.description = v["description"].as<std::string>("");
            vehicle.manufacturer = v["manufacturer"].as<std::string>("");
            vehicle.year = v["year"].as<int>(2024);
        }
        
        // ====================================================================
        // Parse geometry
        // ====================================================================
        if (config["vehicle"]["geometry"]) {
            auto geo = config["vehicle"]["geometry"];
            vehicle.params.drive.mass_kg = geo["mass_kg"].as<double>(1800.0);
            vehicle.params.wheelbase_m = geo["wheelbase_m"].as<double>(2.8);
            vehicle.params.track_width_m = geo["track_width_m"].as<double>(1.6);
            vehicle.params.drive.wheel_radius_m = geo["wheel_radius_m"].as<double>(0.33);
        }
        
        // ====================================================================
        // Parse drivetrain
        // ====================================================================
        if (config["vehicle"]["drivetrain"]) {
            auto dt = config["vehicle"]["drivetrain"];
            vehicle.params.drive.motor_torque_max_nm = dt["motor_torque_max_nm"].as<double>(4000.0);
            vehicle.params.drive.motor_power_max_w = dt["motor_power_max_w"].as<double>(300000.0);
            vehicle.params.drive.gear_ratio = dt["gear_ratio"].as<double>(9.0);
            vehicle.params.drive.drivetrain_eff = dt["efficiency"].as<double>(0.92);
            
            // Also populate motor params
            vehicle.params.motor_params.max_power_kW = dt["motor_power_max_w"].as<double>(300000.0) / 1000.0;
            vehicle.params.motor_params.max_torque_nm = dt["motor_torque_max_nm"].as<double>(4000.0);
            vehicle.params.motor_params.efficiency = dt["efficiency"].as<double>(0.92);
        }
        
        // ====================================================================
        // Parse brakes
        // ====================================================================
        if (config["vehicle"]["brakes"]) {
            auto brakes = config["vehicle"]["brakes"];
            vehicle.params.drive.brake_torque_max_nm = brakes["brake_torque_max_nm"].as<double>(4000.0);
            // Note: regen efficiencies are in DrivePlant, not exposed in params yet
            // Can extend PlantModelParams if needed
        }
        
        // ====================================================================
        // Parse battery
        // ====================================================================
        if (config["vehicle"]["battery"]) {
            auto bat = config["vehicle"]["battery"];
            vehicle.params.battery_params.capacity_kWh = bat["capacity_kwh"].as<double>(60.0);
            vehicle.params.battery_params.max_charge_power_kW = bat["max_charge_power_kw"].as<double>(50.0);
            vehicle.params.battery_params.max_discharge_power_kW = bat["max_discharge_power_kw"].as<double>(150.0);
            vehicle.params.battery_params.efficiency_charge = bat["efficiency_charge"].as<double>(0.95);
            vehicle.params.battery_params.efficiency_discharge = bat["efficiency_discharge"].as<double>(0.95);
            vehicle.params.battery_params.min_soc = bat["min_soc"].as<double>(0.05);
            vehicle.params.battery_params.max_soc = bat["max_soc"].as<double>(0.95);
        }
        
        // ====================================================================
        // Parse resistance
        // ====================================================================
        if (config["vehicle"]["resistance"]) {
            auto res = config["vehicle"]["resistance"];
            vehicle.params.drive.drag_c = res["drag_coefficient"].as<double>(0.35);
            vehicle.params.drive.roll_c = res["rolling_resistance"].as<double>(40.0);
        }
        
        // ====================================================================
        // Parse limits
        // ====================================================================
        if (config["vehicle"]["limits"]) {
            auto lim = config["vehicle"]["limits"];
            vehicle.params.drive.v_max_mps = lim["v_max_mps"].as<double>(60.0);
            vehicle.params.drive.v_stop_eps = lim["v_stop_eps"].as<double>(0.3);
        }
        
        // Validate loaded config
        vehicle.validate();
        
        LOG_INFO("[VehicleConfig] Successfully loaded: %s", vehicle.name.c_str());
        vehicle.print_summary();
        
        return vehicle;
        
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("[VehicleConfig] YAML parse error: ") + e.what()
        );
    } catch (const std::exception& e) {
        throw std::runtime_error(
            std::string("[VehicleConfig] Load error: ") + e.what()
        );
    }
}

VehicleConfig VehicleConfig::get_default() {
    VehicleConfig vehicle;
    
    vehicle.name = "XCMG XDE90 Electric (Default)";
    vehicle.description = "Default configuration matching hardcoded values";
    vehicle.manufacturer = "XCMG";
    vehicle.year = 2024;
    
    // Geometry
    vehicle.params.drive.mass_kg = 1800.0;
    vehicle.params.wheelbase_m = 2.8;
    vehicle.params.track_width_m = 1.6;
    vehicle.params.drive.wheel_radius_m = 0.33;
    
    // Drivetrain
    vehicle.params.drive.motor_torque_max_nm = 4000.0;
    vehicle.params.drive.motor_power_max_w = 300000.0;
    vehicle.params.drive.gear_ratio = 9.0;
    vehicle.params.drive.drivetrain_eff = 0.92;
    
    // Motor params
    vehicle.params.motor_params.max_power_kW = 300.0;
    vehicle.params.motor_params.max_torque_nm = 4000.0;
    vehicle.params.motor_params.efficiency = 0.92;
    
    // Brakes
    vehicle.params.drive.brake_torque_max_nm = 4000.0;
    
    // Battery
    vehicle.params.battery_params.capacity_kWh = 60.0;
    vehicle.params.battery_params.max_charge_power_kW = 50.0;
    vehicle.params.battery_params.max_discharge_power_kW = 150.0;
    vehicle.params.battery_params.efficiency_charge = 0.95;
    vehicle.params.battery_params.efficiency_discharge = 0.95;
    vehicle.params.battery_params.min_soc = 0.05;
    vehicle.params.battery_params.max_soc = 0.95;
    
    // Resistance
    vehicle.params.drive.drag_c = 0.35;
    vehicle.params.drive.roll_c = 40.0;
    
    // Limits
    vehicle.params.drive.v_max_mps = 60.0;
    vehicle.params.drive.v_stop_eps = 0.3;
    
    return vehicle;
}

void VehicleConfig::validate() const {
    // Geometry validation
    if (params.drive.mass_kg <= 0.0) {
        throw std::runtime_error("Invalid mass_kg: must be > 0");
    }
    if (params.wheelbase_m <= 0.0) {
        throw std::runtime_error("Invalid wheelbase_m: must be > 0");
    }
    if (params.drive.wheel_radius_m <= 0.0) {
        throw std::runtime_error("Invalid wheel_radius_m: must be > 0");
    }
    
    // Drivetrain validation
    if (params.drive.motor_torque_max_nm <= 0.0) {
        throw std::runtime_error("Invalid motor_torque_max_nm: must be > 0");
    }
    if (params.drive.motor_power_max_w <= 0.0) {
        throw std::runtime_error("Invalid motor_power_max_w: must be > 0");
    }
    if (params.drive.gear_ratio <= 0.0) {
        throw std::runtime_error("Invalid gear_ratio: must be > 0");
    }
    if (params.drive.drivetrain_eff <= 0.0 || params.drive.drivetrain_eff > 1.0) {
        throw std::runtime_error("Invalid drivetrain_eff: must be 0 < eff <= 1");
    }
    
    // Battery validation
    if (params.battery_params.capacity_kWh <= 0.0) {
        throw std::runtime_error("Invalid battery capacity_kWh: must be > 0");
    }
    if (params.battery_params.min_soc < 0.0 || params.battery_params.min_soc >= params.battery_params.max_soc) {
        throw std::runtime_error("Invalid SOC range: 0 <= min_soc < max_soc <= 1");
    }
    if (params.battery_params.max_soc > 1.0) {
        throw std::runtime_error("Invalid max_soc: must be <= 1.0");
    }
    
    // Limits validation
    if (params.drive.v_max_mps <= 0.0) {
        throw std::runtime_error("Invalid v_max_mps: must be > 0");
    }
    
    LOG_DEBUG("[VehicleConfig] Validation passed");
}

void VehicleConfig::print_summary() const {
    LOG_INFO("========================================");
    LOG_INFO("Vehicle Configuration Summary");
    LOG_INFO("========================================");
    LOG_INFO("Name: %s", name.c_str());
    if (!description.empty()) {
        LOG_INFO("Description: %s", description.c_str());
    }
    LOG_INFO("----------------------------------------");
    LOG_INFO("Mass: %.0f kg", params.drive.mass_kg);
    LOG_INFO("Motor Power: %.0f kW (%.0f hp)", 
             params.drive.motor_power_max_w / 1000.0,
             params.drive.motor_power_max_w / 745.7);
    LOG_INFO("Motor Torque: %.0f Nm", params.drive.motor_torque_max_nm);
    LOG_INFO("Battery: %.1f kWh", params.battery_params.capacity_kWh);
    LOG_INFO("Max Speed: %.1f m/s (%.0f km/h)", 
             params.drive.v_max_mps,
             params.drive.v_max_mps * 3.6);
    LOG_INFO("========================================");
}

} // namespace config
