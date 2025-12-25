#pragma once

#include "plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct DriveParams {
    double mass_kg = 1800.0;
    double wheel_radius_m = 0.33;

    // simple resistive terms (V1)
    double drag_c = 0.35;       // N/(m/s)^2  (aero-ish)
    double roll_c = 40.0;       // N          (rolling-ish)

    // command limits
    double motor_torque_max_nm = 4000.0;  // clamp motor torque command
    double brake_torque_max_nm = 4000.0;  // max brake torque (at wheel)
};

class DrivePlant {
public:
    explicit DrivePlant(DriveParams p = {}) : p_(p) {}

    const DriveParams& params() const { return p_; }
    void set_params(const DriveParams& p) { p_ = p; }

    // Updates:
    //  - v_mps (longitudinal speed)
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

private:
    DriveParams p_;
};

} // namespace plant
