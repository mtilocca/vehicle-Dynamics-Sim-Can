#include "sim_app.hpp"

int main() {
    sim::SimAppConfig cfg{};
    cfg.dt_s = 0.01;
    cfg.duration_s = 20.0;
    cfg.log_hz = 10.0;

    // Defaults (used if Lua fails / not enabled)
    cfg.motor_torque_nm = 1200.0;
    cfg.brake_pct = 0.0;
    cfg.steer_amp_deg = 10.0;
    cfg.steer_freq_hz = 0.2;

    // Enable Lua scenario loading (JSON path passed to Lua)
    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";
    cfg.scenario_json_path = "config/scenarios/brake_test.json";

    sim::SimApp app(cfg);
    return app.run_plant_only();
}
