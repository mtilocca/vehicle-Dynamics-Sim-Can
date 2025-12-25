#pragma once

#include <string>

namespace sim {

struct SimAppConfig {
    double dt_s = 0.01;         // 100 Hz
    double duration_s = 20.0;   // run time
    double log_hz = 10.0;       // print rate

    // Simple open-loop command profile (plant-only validation)
    double motor_torque_nm = 1200.0; // constant drive torque
    double brake_pct = 0.0;          // constant brake
    double steer_amp_deg = 10.0;     // sinusoidal steer amplitude
    double steer_freq_hz = 0.2;      // sinusoidal steer frequency
};

class SimApp {
public:
    explicit SimApp(SimAppConfig cfg = {});
    int run_plant_only();

private:
    SimAppConfig cfg_;
};

} // namespace sim
