// src/sim/sim_main.cpp
// FIXED: Load timing parameters from JSON scenario file (Bug #1)
// Uses simple string parsing - no external JSON library required
#include "sim/sim_app.hpp"
#include "utils/logging.hpp"
#include <fstream>
#include <sstream>
#include <string>

// Simple JSON parser for timing section only
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
        
        // Check for timing section start
        if (line.find("\"timing\"") != std::string::npos) {
            in_timing_section = true;
            found_timing = true;
            continue;
        }
        
        // Check for section end
        if (in_timing_section && line.find("}") != std::string::npos) {
            in_timing_section = false;
            break;
        }
        
        // Parse timing fields
        if (in_timing_section) {
            // Parse dt_s
            if (line.find("\"dt_s\"") != std::string::npos) {
                size_t colon = line.find(":");
                if (colon != std::string::npos) {
                    std::string value_str = line.substr(colon + 1);
                    // Remove comma if present
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
    
    if (found_timing) {
        LOG_INFO("Loaded timing from JSON: dt=%.4fs, duration=%.1fs, log_hz=%.1f, realtime=%s",
                 cfg.dt_s, cfg.duration_s, cfg.log_hz, 
                 cfg.real_time_mode ? "true" : "false");
    }
    
    return found_timing;
}

int main(int argc, char** argv) {
    // Parse command line arguments for scenario selection
    std::string scenario_path = "config/scenarios/brake_test.json";
    
    if (argc > 1) {
        scenario_path = argv[1];
    }

    // Configure logging
    // For debugging: utils::LogLevel::Debug
    // For production: utils::LogLevel::Info
    utils::set_level(utils::LogLevel::Info);
    
    // Simulation configuration with DEFAULTS
    sim::SimAppConfig cfg{};
    
    // DEFAULT Timing (will be overridden by JSON if "timing" section present)
    cfg.dt_s = 0.01;          // 10ms timestep
    cfg.duration_s = 20.0;    // 20 second simulation (default)
    cfg.log_hz = 10.0;        // Log at 10 Hz
    
    // DEFAULT Real-time mode
    cfg.real_time_mode = false;   // true = runs in wall-clock time (good for CAN monitoring)
                                 // false = runs as fast as possible (good for batch testing)
    
    // Scenario
    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";
    cfg.scenario_json_path = scenario_path;
    
    // Output files
    cfg.csv_log_path = "sim_out.csv";
    cfg.debug_log_path = "sim_debug.log";
    cfg.enable_debug_log_file = true;  // Set to false to disable file logging

    // CAN configuration
    cfg.enable_can_tx = true;                 // Enable CAN transmission
    cfg.can_interface = "vcan0";              // CAN interface name
    cfg.can_map_path = "config/can_map.csv";  // Path to CAN map file

    // ============================================================================
    // BUG FIX #1: Load timing parameters from JSON scenario file (if present)
    // ============================================================================
    load_timing_from_json(scenario_path, cfg);

    LOG_INFO("========================================");
    LOG_INFO("Plant-Sensor-CAN Simulation");
    LOG_INFO("Scenario: %s", scenario_path.c_str());
    LOG_INFO("========================================");

    sim::SimApp app(cfg);
    return app.run_plant_only();
}