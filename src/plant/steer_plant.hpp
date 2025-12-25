// src/plant/steer_plant.hpp
#pragma once

#include <algorithm>

#include "plant/plant_state.hpp"

namespace sim { struct ActuatorCmd; }

namespace plant {

struct SteerParams {
    // Virtual bicycle steer limits and rate
    double delta_max_deg = 35.0;   // absolute max virtual steer angle
    double steer_rate_dps = 200.0; // deg/s max change of virtual steer

    // Geometry for Ackermann mapping
    double wheelbase_m = 2.8;
    double track_width_m = 1.6;

    // Speed-dependent steering clamp (V1 "understeer-ish")
    double v_steer_limit_start_mps = 8.0;   // start reducing after this speed
    double v_steer_limit_end_mps   = 30.0;  // at/above this, max steer reduced
    double steer_limit_ratio_highv = 0.35;  // fraction of delta_max at high speed
};

class SteerPlant {
public:
    explicit SteerPlant(SteerParams p = {}) : p_(p) {}

    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s);

    const SteerParams& params() const { return p_; }
    SteerParams& params() { return p_; }

private:
    SteerParams p_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static double deg2rad(double d) { return d * 3.14159265358979323846 / 180.0; }
    static double rad2deg(double r) { return r * 180.0 / 3.14159265358979323846; }
};

} // namespace plant
