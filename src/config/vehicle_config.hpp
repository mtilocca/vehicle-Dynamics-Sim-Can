// src/config/vehicle_config.hpp
#pragma once

#include <string>
#include "plant/plant_model.hpp"

namespace config {

/**
 * VehicleConfig — loads vehicle parameters from a YAML file.
 *
 * YAML sections parsed:
 *   vehicle.geometry   → mass, wheelbase, track, wheel_radius, CoG, yaw inertia
 *   vehicle.drivetrain → motor_torque_max, motor_power_max, gear_ratio, efficiency
 *   vehicle.brakes     → brake_torque_max, brake_bias_front
 *   vehicle.resistance → drag_coefficient, rolling_resistance
 *   vehicle.limits     → v_max_mps, v_stop_eps
 *   vehicle.dynamics   → mu_surface, Cy_front_Npm, Cy_rear_Npm
 *
 * Usage:
 *   auto cfg = VehicleConfig::load("config/vehicles/heavy_truck.yaml");
 *   PlantModel plant(cfg.params);
 */
class VehicleConfig {
public:
    std::string name;
    std::string description;
    std::string manufacturer;
    int year = 2025;

    plant::PlantModelParams params;

    static VehicleConfig load(const std::string& yaml_path);
    static VehicleConfig get_default();

    void validate() const;
    void print_summary() const;

    VehicleConfig() = default;
};

} // namespace config
