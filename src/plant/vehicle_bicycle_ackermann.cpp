// src/plant/vehicle_bicycle_ackermann.cpp
#include "vehicle_bicycle_ackermann.hpp"

#include <cmath>
#include <limits>

namespace plant {

double VehicleBicycleAckermann::clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

int VehicleBicycleAckermann::sign(double v) {
    return (v > 0) - (v < 0);
}

void VehicleBicycleAckermann::ackermann_map(
    double delta_virtual_rad,
    const BicycleAckermannParams& p,
    double& delta_fl_rad_out,
    double& delta_fr_rad_out,
    double* R_m_out
) {
    // Clamp steering for safety/physical plausibility
    double delta = clamp(delta_virtual_rad, -p.delta_max_rad, p.delta_max_rad);

    if (std::abs(delta) < p.eps_rad) {
        delta_fl_rad_out = 0.0;
        delta_fr_rad_out = 0.0;
        if (R_m_out) *R_m_out = std::numeric_limits<double>::infinity();
        return;
    }

    // Signed bicycle turning radius
    double R = p.L_m / std::tan(delta); // signed
    double R_abs = std::abs(R);

    // Avoid singularity: need |R| > W/2
    double min_R = (p.W_m * 0.5) + p.tiny_m;
    if (R_abs < min_R) R_abs = min_R;

    int sgn = sign(delta);

    // Inner/outer wheel angles magnitudes
    double delta_inner = std::atan(p.L_m / (R_abs - p.W_m * 0.5)) * static_cast<double>(sgn);
    double delta_outer = std::atan(p.L_m / (R_abs + p.W_m * 0.5)) * static_cast<double>(sgn);

    // Convention: delta > 0 => left turn => left wheel is inner
    if (delta > 0.0) {
        delta_fl_rad_out = delta_inner;
        delta_fr_rad_out = delta_outer;
    } else {
        // right turn => right wheel is inner
        delta_fl_rad_out = delta_outer;
        delta_fr_rad_out = delta_inner;
    }

    if (R_m_out) *R_m_out = R; // keep signed R here (before abs clamp)
}

BicycleStepResult VehicleBicycleAckermann::step(
    const BicycleState2D& s,
    double v_mps,
    double delta_virtual_rad,
    const BicycleAckermannParams& p,
    double dt_s
) {
    BicycleStepResult r{};
    r.next = s;

    // Clamp steer
    double delta = clamp(delta_virtual_rad, -p.delta_max_rad, p.delta_max_rad);

    // Bicycle yaw rate
    if (std::abs(delta) < p.eps_rad) {
        r.yaw_rate_rps = 0.0;
        r.R_m = std::numeric_limits<double>::infinity();
    } else {
        r.yaw_rate_rps = (v_mps / p.L_m) * std::tan(delta);
        r.R_m = p.L_m / std::tan(delta); // signed
    }

    // Integrate kinematics (Euler), rear axle reference
    r.next.x_m += dt_s * v_mps * std::cos(s.yaw_rad);
    r.next.y_m += dt_s * v_mps * std::sin(s.yaw_rad);
    r.next.yaw_rad += dt_s * r.yaw_rate_rps;

    // Ackermann mapping to physical wheel angles
    ackermann_map(delta, p, r.delta_fl_rad, r.delta_fr_rad, nullptr);

    return r;
}

} // namespace plant
