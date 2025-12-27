// src/config/vehicle_config.hpp
#pragma once

#include <string>
#include <optional>
#include "plant/plant_model.hpp"

namespace config {

/**
 * VehicleConfig - Loads vehicle parameters from YAML files
 * 
 * Usage:
 *   auto config = VehicleConfig::load("config/vehicles/performance_ev.yaml");
 *   PlantModel plant(config.params);
 * 
 * Falls back to hardcoded defaults if file not found.
 */
class VehicleConfig {
public:
    std::string name;
    std::string description;
    std::string manufacturer;
    int year;
    
    // Plant parameters loaded from YAML
    plant::PlantModelParams params;
    
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
     * Get default vehicle configuration (XCMG dump truck)
     * Matches current hardcoded values for backwards compatibility.
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
