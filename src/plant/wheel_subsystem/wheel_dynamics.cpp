#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include <algorithm>
#include <cmath>

namespace plant {

double WheelDynamics::step(double omega_radps,
                           double tau_drive_nm,
                           double tau_brake_nm,
                           double Fx_tire_n,
                           double dt) {
    omega_radps_ = omega_radps;

    // Tire reaction torque: tau_tire = Fx * R
    // Fx > 0 (vehicle forward force) produces a resisting torque on the wheel
    const double tau_tire_nm = Fx_tire_n * p_.radius_m;

    // Brake torque always opposes rotation direction
    const double brake_dir = (omega_radps_ >= 0.0) ? 1.0 : -1.0;
    const double tau_brake_effective_nm = tau_brake_nm * brake_dir;

    // Optional viscous damping (opposes omega)
    const double tau_damp_nm = p_.viscous_damping_nm_per_radps * omega_radps_;

    // Optional rolling resistance torque (opposes omega)
    const double tau_roll_nm = p_.rolling_resist_nm * brake_dir;

    // Net torque on wheel
    const double tau_net_nm = tau_drive_nm - tau_brake_effective_nm - tau_tire_nm - tau_damp_nm - tau_roll_nm;

    // Angular acceleration
    const double domega = (p_.inertia_kgm2 > 0.0) ? (tau_net_nm / p_.inertia_kgm2) : 0.0;

    omega_radps_ += domega * dt;

    // Clamp for stability
    omega_radps_ = std::max(p_.omega_min_radps, std::min(p_.omega_max_radps, omega_radps_));

    return omega_radps_;
}

double WheelDynamics::compute_slip_ratio(double v_x_mps) const {
    // Wheel linear velocity at tread
    const double v_wheel = omega_radps_ * p_.radius_m;

    // Robustness at/near standstill:
    // If both the ground speed and wheel speed are tiny, define slip as zero.
    if (std::abs(v_x_mps) < p_.v_eps_mps && std::abs(v_wheel) < p_.v_eps_mps) {
        return 0.0;
    }

    // Denominator: max(|Vx|, |v_wheel|, ε)
    // Works for both forward and reverse (Vx may be negative).
    const double denom = std::max({std::abs(v_x_mps), std::abs(v_wheel), p_.v_eps_mps});

    // Slip ratio: σx = (ω·R - Vx) / denom
    // σx > 0: drive slip (wheel faster than ground)
    // σx < 0: brake slip (wheel slower than ground)
    return (v_wheel - v_x_mps) / denom;
}

double WheelDynamics::compute_lateral_slip(double v_x_mps, double v_y_mps) const {
    // Small-angle approximation for slip angle proxy
    const double denom = std::max(std::abs(v_x_mps), p_.v_eps_mps);
    return v_y_mps / denom;
}

} // namespace plant
