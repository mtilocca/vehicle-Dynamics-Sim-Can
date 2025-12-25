#include "sim/sim_app.hpp"
#include "utils/logging.hpp"

int main() {
    // Enable debug logging for troubleshooting
    // Set to LogLevel::Info for less verbose output
    utils::set_level(utils::LogLevel::Debug);
    
    sim::SimAppConfig cfg{};
    cfg.dt_s = 0.01;
    cfg.duration_s = 20.0;
    cfg.log_hz = 10.0;

    cfg.use_lua_scenario = true;
    cfg.lua_script_path = "config/lua/scenario.lua";

    cfg.csv_log_path = "sim_out.csv";

    sim::SimApp app(cfg);
    return app.run_plant_only();
}