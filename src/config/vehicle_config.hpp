// src/config/vehicle_config.hpp
#pragma once

#include <string>
#include <optional>
#include "plant/plant_model.hpp"

namespace config {

/**
 * Tire surface friction parameters
 */
struct TireSurfaceParams {
    std::string name;
    std::string description;
    double mu_peak = 0.72;      // Peak friction coefficient
    double mu_slide = 0.65;     // Sliding friction coefficient
};

/**
 * Tire model parameters (Dugoff model)
 */
struct TireParams {
    std::string model = "dugoff";
    double radius_m = 1.93;
    double width_m = 1.016;

    // Slip stiffness
    double Cx_base = 280000.0;
    double Cy_base = 220000.0;
    double Fz_ref = 800000.0;
    double load_exponent = 0.50;

    // Surface friction
    TireSurfaceParams surface;

    // Velocity fade (optional)
    bool velocity_fade_enabled = false;
    double fade_factor = 0.003;
    double min_friction_ratio = 0.70;

    // Slip limits
    double sigma_x_max = 0.95;
    double sigma_y_max = 0.50;
    double v_min_for_slip_calc = 0.5;
};

/**
 * Wheel rotational dynamics parameters
 */
struct WheelParams {
    double inertia_kgm2 = 1.0;   // Wheel rotational inertia (per-wheel)
};

/**
 * VehicleConfig - Loads vehicle parameters from YAML files
 *
 * Usage:
 *   auto config = VehicleConfig::load("config/vehicles/heavy_truck.yaml");
 *   PlantModel plant(config.params);
 *
 * Falls back to hardcoded defaults if file not found.
 */
class VehicleConfig {
public:
    std::string name;
    std::string description;
    std::string manufacturer;
    int year = 2024;

    // Plant parameters loaded from YAML
    plant::PlantModelParams params;

    // Wheel parameters (NEW)
    WheelParams wheel_params;

    // Tire parameters
    TireParams tire_params;

    /**
     * Load vehicle config from YAML file
     * @param yaml_path Path to YAML file (e.g., "config/vehicles/default.yaml")
     * @return VehicleConfig with loaded parameters
     * @throws std::runtime_error if file exists but is invalid
     *
     * If file doesn't exist, returns default configuration with warning.
     */
    static VehicleConfig load(const std::string& yaml_path);

    /**
     * Get default vehicle configuration (matches prior hardcoded values)
     */
    static VehicleConfig get_default();

    /**
     * Validate loaded parameters
     * @throws std::runtime_error if any parameter is invalid
     */
    void validate() const;

    /**
     * Print summary of configuration to console
     */
    void print_summary() const;

    VehicleConfig() = default;
};

} // namespace config
