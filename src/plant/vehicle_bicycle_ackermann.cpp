#include "vehicle_bicycle_ackermann.hpp"

#include <algorithm>

namespace plant {

static inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

void VehicleBicycleAckermann::ackermann_map(
    double steer_virtual_rad,
    const BicycleAckermannParams& p,
    double& delta_fl_rad,
    double& delta_fr_rad,
    double* curvature_out)
{
    const double delta =
        clamp(steer_virtual_rad, -p.delta_max_rad, p.delta_max_rad);

    if (std::abs(delta) < 1e-6) {
        delta_fl_rad = 0.0;
        delta_fr_rad = 0.0;
        if (curvature_out) *curvature_out = 0.0;
        return;
    }

    const double R = p.L_m / std::tan(delta);
    const double Rl = R - p.W_m * 0.5;
    const double Rr = R + p.W_m * 0.5;

    delta_fl_rad = std::atan(p.L_m / Rl);
    delta_fr_rad = std::atan(p.L_m / Rr);

    if (curvature_out) *curvature_out = 1.0 / R;
}

BicycleStepResult VehicleBicycleAckermann::step(
    const BicycleState2D& s,
    double v,
    double steer_virtual,
    const BicycleAckermannParams& p,
    double dt)
{
    BicycleStepResult out{};
    out.next = s;

    if (dt <= 0.0 || std::abs(v) < 1e-3)
        return out;

    // Nominal yaw rate
    double yaw_rate =
        v * std::tan(steer_virtual) / p.L_m;

    // --- NEW: lateral acceleration clamp
    const double a_lat = v * yaw_rate;
    const double a_lat_max = p.mu_lat * p.g;

    if (std::abs(a_lat) > a_lat_max) {
        yaw_rate *= (a_lat_max / std::abs(a_lat));
    }

    // Integrate pose
    out.next.x_m += v * std::cos(s.yaw_rad) * dt;
    out.next.y_m += v * std::sin(s.yaw_rad) * dt;
    out.next.yaw_rad += yaw_rate * dt;

    out.yaw_rate_rps = yaw_rate;

    // Wheel angles
    ackermann_map(
        steer_virtual,
        p,
        out.delta_fl_rad,
        out.delta_fr_rad,
        nullptr
    );

    return out;
}

} // namespace plant
