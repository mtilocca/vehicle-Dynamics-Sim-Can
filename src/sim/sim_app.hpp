// src/sim/sim_app.hpp
#pragma once

#include <string>

#include "sim/lua_runtime.hpp"     // provides sim::LuaRuntime
#include "plant/plant_state.hpp"   // plant::PlantState

namespace sim {

struct SimAppConfig {
    // Core sim timing
    double dt_s = 0.01;
    double duration_s = 20.0;
    double log_hz = 10.0;

    // Default open-loop (used when Lua scenario disabled or fails)
    double motor_torque_nm = 1200.0;
    double brake_pct = 0.0;        // 0..100
    double steer_amp_deg = 10.0;
    double steer_freq_hz = 0.2;

    // Scenario via Lua (+ optional JSON loaded by LuaRuntime)
    bool use_lua_scenario = false;
    std::string lua_script_path;       // e.g. "config/lua/scenario.lua"
    std::string scenario_json_path;    // e.g. "config/scenarios/brake_test.json"
};

class SimApp {
public:
    explicit SimApp(SimAppConfig cfg);

    // current sim you are running
    int run_plant_only();

private:
    SimAppConfig cfg_;

    // Lua scenario runtime
    LuaRuntime lua_;
    bool lua_ready_ = false;
};

} // namespace sim
