#include "sim_app.hpp"

int main() {
    sim::SimAppConfig cfg{};
    cfg.dt_s = 0.01;
    cfg.duration_s = 20.0;
    cfg.log_hz = 10.0;

    cfg.motor_torque_nm = 1200.0;
    cfg.brake_pct = 0.0;
    cfg.steer_amp_deg = 10.0;
    cfg.steer_freq_hz = 0.2;

    sim::SimApp app(cfg);
    return app.run_plant_only();
}
