#pragma once

#include "plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct SteerParams {
    double delta_max_deg = 35.0;      // max virtual steer (deg)
    double steer_rate_dps = 200.0;    // rate limit for virtual steer (deg/s)

    // Ackermann geometry
    double wheelbase_m = 2.8;         // L
    double track_width_m = 1.6;       // W
};

class SteerPlant {
public:
    explicit SteerPlant(SteerParams p = {}) : p_(p) {}

    const SteerParams& params() const { return p_; }
    void set_params(const SteerParams& p) { p_ = p; }

    // Updates:
    //  - steer_virtual_rad (rate-limited + clamped)
    //  - delta_fl_rad / delta_fr_rad (Ackermann)
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

private:
    SteerParams p_;
};

} // namespace plant
