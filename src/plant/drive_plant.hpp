// src/plant/drive_plant.hpp
#pragma once

#include <algorithm>

#include "plant/plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    // Vehicle / environment
    double mass_kg = 1800.0;
    double wheel_radius_m = 0.33;

    // Simple resistances (V1)
    // drag: use v*|v| so it naturally opposes motion
    double drag_c = 0.35;   // N per (m/s)^2
    double roll_c = 40.0;   // N (Coulomb-ish rolling resistance magnitude)

    // Actuator limits (interpreted as motor shaft and wheel brake)
    double motor_torque_max_nm = 4000.0; // motor shaft torque clamp
    double brake_torque_max_nm = 4000.0; // wheel brake torque at tire (magnitude)

    // Drivetrain / motor realism (V1)
    double gear_ratio = 9.0;          // motor->wheel total ratio
    double drivetrain_eff = 0.92;     // 0..1
    double motor_power_max_w = 90000; // 90 kW (tune)
    double v_stop_eps = 0.3;          // avoid divide-by-zero in power limiting

    // Speed clamp
    double v_max_mps = 60.0;          // hard clamp safety
};

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}) : p_(p) {}

    // Updates PlantState longitudinal dynamics: v_mps, a_long_mps2
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const DriveParams& params() const { return p_; }
    DriveParams& params() { return p_; }

private:
    DriveParams p_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }
};

} // namespace plant
