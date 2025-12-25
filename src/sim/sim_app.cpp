#include "sim_app.hpp"

#include<utility>
#include <cmath>
#include <cstdio>

#include "plant/plant_model.hpp"
#include "plant/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

namespace sim {

SimApp::SimApp(SimAppConfig cfg) : cfg_(std::move(cfg)) {}

int SimApp::run_plant_only() {
    plant::PlantModelParams pm_params{};
    plant::PlantModel plant_model(pm_params);
    plant::PlantState s{};

    const double dt = cfg_.dt_s;
    const int steps = (dt > 0.0) ? static_cast<int>(cfg_.duration_s / dt) : 0;

    const double log_period_s = (cfg_.log_hz > 0.0) ? (1.0 / cfg_.log_hz) : 0.1;
    double next_log_t = 0.0;

    // ---- Scenario init (optional)
    lua_ready_ = false;
    if (cfg_.use_lua_scenario) {
        lua_ready_ = lua_.init(cfg_.lua_script_path, cfg_.scenario_json_path);
        if (!lua_ready_) {
            std::printf("[WARN] Lua scenario requested but failed to init. Falling back to defaults.\n");
        }
    }

    std::printf("Plant-only validator\n");
    std::printf("dt=%.4f s, duration=%.2f s, steps=%d\n", dt, cfg_.duration_s, steps);
    std::printf("Scenario: %s\n", (lua_ready_ ? "Lua+JSON" : "C++ defaults"));
    std::printf("Columns: t  x  y  yaw_deg  v_mps  steer_deg  fl_deg  fr_deg  motor_nm  brake_pct\n");

    for (int k = 0; k < steps; ++k) {
        const double t = s.t_s;

        ActuatorCmd cmd{};
        cmd.system_enable = true;
        cmd.mode = 0;

        if (lua_ready_) {
            // Ask Lua for cmd at time t using current state
            if (!lua_.get_actuator_cmd(t, s, cmd)) {
                // If Lua fails mid-run, fall back safely for this tick
                cmd.drive_torque_cmd_nm = cfg_.motor_torque_nm;
                cmd.brake_cmd_pct = cfg_.brake_pct;
                cmd.steer_cmd_deg =
                    cfg_.steer_amp_deg * std::sin(2.0 * M_PI * cfg_.steer_freq_hz * t);
            }
        } else {
            // ---- existing hardcoded open-loop profile (unchanged behaviour)
            cmd.drive_torque_cmd_nm = cfg_.motor_torque_nm;
            cmd.brake_cmd_pct = cfg_.brake_pct;
            cmd.steer_cmd_deg =
                cfg_.steer_amp_deg * std::sin(2.0 * M_PI * cfg_.steer_freq_hz * t);
        }

        plant_model.step(s, cmd, dt);

        if (s.t_s >= next_log_t) {
            const double yaw_deg = s.yaw_rad * 180.0 / M_PI;
            const double steer_deg = s.steer_virtual_rad * 180.0 / M_PI;
            const double fl_deg = s.delta_fl_rad * 180.0 / M_PI;
            const double fr_deg = s.delta_fr_rad * 180.0 / M_PI;

            std::printf("%.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.2f  %.0f  %.1f\n",
                        s.t_s, s.x_m, s.y_m, yaw_deg, s.v_mps,
                        steer_deg, fl_deg, fr_deg,
                        cmd.drive_torque_cmd_nm, cmd.brake_cmd_pct);

            next_log_t += log_period_s;
        }
    }

    std::printf("Done.\n");
    return 0;
}

} // namespace sim
