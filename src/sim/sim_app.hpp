// src/sim/sim_app.hpp
#pragma once

#include <string>
#include <optional>

#include "sim/lua_runtime.hpp"
#include "plant/plant_state.hpp"
#include "plant/plant_model.hpp"

namespace sim {

struct SimAppConfig {
    // Core sim timing
    double dt_s = 0.01;
    double duration_s = 20.0;
    double log_hz = 10.0;

    // Real-time mode
    bool real_time_mode = true;  // Enable real-time pacing (true = wall-clock time)

    // Default open-loop (used when Lua scenario disabled or fails)
    double motor_torque_nm = 1200.0;
    double brake_pct = 0.0;
    double steer_amp_deg = 10.0;
    double steer_freq_hz = 0.2;

    // Scenario via Lua
    bool use_lua_scenario = false;
    std::string lua_script_path;       // e.g. "config/lua/scenario.lua"
    std::string scenario_json_path;    // e.g. "config/scenarios/brake_test.json"

    // Output files
    std::string csv_log_path = "sim_out.csv";
    std::string debug_log_path = "sim_debug.log";
    
    // Logging control
    bool enable_debug_log_file = true;

    // CAN configuration
    bool enable_can_tx = true;                      // Enable CAN transmission
    std::string can_interface = "vcan0";            // CAN interface name
    std::string can_map_path = "config/can_map.csv"; // Path to CAN map
    
    // NEW: Vehicle configuration (optional - if not set, uses hardcoded defaults)
    std::optional<plant::PlantModelParams> vehicle_params;
};

class SimApp {
public:
    explicit SimApp(SimAppConfig cfg);

    int run_plant_only();

private:
    SimAppConfig cfg_;

    LuaRuntime lua_;
    bool lua_ready_ = false;
};

} // namespace sim