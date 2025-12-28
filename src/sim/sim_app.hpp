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
    
    // ========== NEW: CAN RX for closed-loop control ==========
    
    /**
     * Enable CAN RX for closed-loop control
     * 
     * When true:
     * - Simulator listens for ACTUATOR_CMD_1 frames on CAN
     * - Lua scenario is automatically disabled
     * - Actuator commands come from external controller (e.g., Go)
     * 
     * When false (default):
     * - Use Lua/JSON scenarios (open-loop)
     */
    bool enable_can_rx = false;
    
    /**
     * CAN frame name to listen for actuator commands
     * Must exist in can_map.csv as RX frame
     * Default: "ACTUATOR_CMD_1" (frame ID 0x100)
     */
    std::string actuator_cmd_frame_name = "ACTUATOR_CMD_1";
    
    /**
     * CAN RX timeout (seconds)
     * 
     * If no CAN messages received for this duration, simulator enters safe mode:
     * - Zero torque
     * - Zero brake
     * - Zero steering
     * - System disabled
     * 
     * Typical values: 0.1 - 1.0 seconds
     * Default: 0.5 seconds (10 missed frames at 20 Hz)
     */
    double can_rx_timeout_s = 0.5;
    
    // Vehicle configuration (optional - if not set, uses hardcoded defaults)
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