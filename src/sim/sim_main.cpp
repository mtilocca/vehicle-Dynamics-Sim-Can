// src/sim/sim_main.cpp
// UPDATED: Added vehicle configuration support via command-line or JSON
#include "sim/sim_app.hpp"
#include "config/vehicle_config.hpp"
#include "utils/logging.hpp"
#include <fstream>
#include <sstream>
#include <string>

// Parse timing section from JSON (handles both old and new formats)
bool load_timing_from_json(const std::string& scenario_path, sim::SimAppConfig& cfg) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    bool in_timing_section = false;
    bool found_timing = false;
    
    while (std::getline(file, line)) {
        // Remove whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        // Check for timing section start (new format)
        if (line.find("\"timing\"") != std::string::npos) {
            in_timing_section = true;
            found_timing = true;
            continue;
        }
        
        // Check for section end
        if (in_timing_section && line.find("}") != std::string::npos) {
            in_timing_section = false;
            // Don't break - keep parsing for other fields
        }
        
        // Parse timing fields (works in both formats)
        // In new format: inside "timing" section
        // In old format: at root level
        bool should_parse = in_timing_section || !found_timing;
        
        if (should_parse) {
            // Parse dt_s
            if (line.find("\"dt_s\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.dt_s = std::stod(value_str);
                }
            }
            // Parse duration_s
            else if (line.find("\"duration_s\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.duration_s = std::stod(value_str);
                }
            }
            // Parse log_hz
            else if (line.find("\"log_hz\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    size_t comma = value_str.find(",");
                    if (comma != std::string::npos) {
                        value_str = value_str.substr(0, comma);
                    }
                    cfg.log_hz = std::stod(value_str);
                }
            }
            // Parse real_time_mode
            else if (line.find("\"real_time_mode\"") != std::string::npos) {
                if (line.find("true") != std::string::npos) {
                    cfg.real_time_mode = true;
                } else if (line.find("false") != std::string::npos) {
                    cfg.real_time_mode = false;
                }
            }
        }
    }
    
    return true; // Always return true since we got some config
}

// Parse vehicle_config field from JSON (if present)
std::string load_vehicle_config_path_from_json(const std::string& scenario_path) {
    std::ifstream file(scenario_path);
    if (!file.is_open()) {
        return "";
    }

    std::string line;
    while (std::getline(file, line)) {
        // Look for "vehicle_config" field
        if (line.find("\"vehicle_config\"") != std::string::npos) {
            size_t colon = line.find(":");
            if (colon != std::string::npos) {
                std::string value_str = line.substr(colon + 1);
                
                // Remove whitespace, quotes, and comma
                value_str.erase(0, value_str.find_first_not_of(" \t\r\n\""));
                value_str.erase(value_str.find_last_not_of(" \t\r\n\",") + 1);
                
                return value_str;
            }
        }
    }
    
    return "";
}

int main(int argc, char** argv) {
    // Parse command line arguments
    std::string scenario_path = "config/scenarios/brake_test.json";
    std::string vehicle_config_path = "";  // Empty = try JSON, then defaults
    
    if (argc > 1) {
        scenario_path = argv[1];
    }
    
    // Optional: Override vehicle config via command line
    if (argc > 2) {
        vehicle_config_path = argv[2];
    }

    // Configure logging
    utils::set_level(utils::LogLevel::Info);
    
    // ========================================================================
    // Simulation configuration with DEFAULTS
    // ========================================================================
    sim::SimAppConfig cfg{};
    
    // DEFAULT Timing (will be overridden by JSON)
    cfg.dt_s = 0.01;          // 10ms timestep
    cfg.duration_s = 20.0;    // 20 second simulation
    cfg.log_hz = 10.0;        // Log at 10 Hz
    cfg.real_time_mode = false;
    
    // Scenario
    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";
    cfg.scenario_json_path = scenario_path;
    
    // Output files
    cfg.csv_log_path = "sim_out.csv";
    cfg.debug_log_path = "sim_debug.log";
    cfg.enable_debug_log_file = true;

    // CAN configuration
    cfg.enable_can_tx = true;
    cfg.can_interface = "vcan0";
    cfg.can_map_path = "config/can_map.csv";

    // ========================================================================
    // Load timing parameters from JSON scenario file
    // ========================================================================
    load_timing_from_json(scenario_path, cfg);
    
    LOG_INFO("Loaded config: dt=%.4fs, duration=%.1fs, log_hz=%.1f, realtime=%s",
             cfg.dt_s, cfg.duration_s, cfg.log_hz, 
             cfg.real_time_mode ? "true" : "false");

    // ========================================================================
    // Load vehicle configuration
    // Priority: 1) Command-line arg, 2) JSON field, 3) Defaults
    // ========================================================================
    config::VehicleConfig vehicle;
    
    // Priority 1: Command-line argument (overrides everything)
    if (!vehicle_config_path.empty()) {
        LOG_INFO("Loading vehicle from command-line: %s", vehicle_config_path.c_str());
        vehicle = config::VehicleConfig::load(vehicle_config_path);
    }
    // Priority 2: "vehicle_config" field in JSON
    else {
        std::string json_vehicle_path = load_vehicle_config_path_from_json(scenario_path);
        if (!json_vehicle_path.empty()) {
            LOG_INFO("Loading vehicle from JSON: %s", json_vehicle_path.c_str());
            vehicle = config::VehicleConfig::load(json_vehicle_path);
        } else {
            // Priority 3: Use defaults
            LOG_INFO("No vehicle config specified, using defaults");
            vehicle = config::VehicleConfig::get_default();
        }
    }
    
    // Print vehicle summary
    vehicle.print_summary();
    
    // Pass vehicle params to SimApp
    cfg.vehicle_params = vehicle.params;

    // ========================================================================
    // Run simulation
    // ========================================================================
    LOG_INFO("========================================");
    LOG_INFO("Plant-Sensor-CAN Simulation");
    LOG_INFO("Scenario: %s", scenario_path.c_str());
    LOG_INFO("Vehicle: %s", vehicle.name.c_str());
    LOG_INFO("========================================");

    sim::SimApp app(cfg);
    return app.run_plant_only();
}