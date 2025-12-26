// src/sim/sim_main.cpp
#include "sim/sim_app.hpp"
#include "utils/logging.hpp"

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
    
    // Simulation configuration
    sim::SimAppConfig cfg{};
    
    // Timing
    cfg.dt_s = 0.01;          // 10ms timestep
    cfg.duration_s = 60.0;    // 20 second simulation
    cfg.log_hz = 10.0;        // Log at 10 Hz
    
    // Real-time mode
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

    // CAN configuration (NEW!)
    cfg.enable_can_tx = true;                 // Enable CAN transmission
    cfg.can_interface = "vcan0";              // CAN interface name
    cfg.can_map_path = "config/can_map.csv";  // Path to CAN map file

    LOG_INFO("========================================");
    LOG_INFO("Plant-Sensor-CAN Simulation");
    LOG_INFO("Scenario: %s", scenario_path.c_str());
    LOG_INFO("========================================");

    sim::SimApp app(cfg);
    return app.run_plant_only();
}