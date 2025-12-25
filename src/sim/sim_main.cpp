#include "sim/sim_app.hpp"

int main() {
    sim::SimAppConfig cfg{};
    cfg.dt_s = 0.01;
    cfg.duration_s = 20.0;
    cfg.log_hz = 10.0;

    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";
    // cfg.scenario_json_path intentionally unused (Lua chooses JSON path internally)

    cfg.csv_log_path = "sim_out.csv";

    sim::SimApp app(cfg);
    return app.run_plant_only();
}
