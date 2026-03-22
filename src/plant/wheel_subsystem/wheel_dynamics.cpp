// src/plant/wheel_subsystem/wheel_dynamics.cpp
#include "plant/wheel_subsystem/wheel_dynamics.hpp"

namespace plant {

void WheelDynamics::step(double tau_drive_nm, double tau_brake_nm, double fx_n, double dt_s) {
    if (dt_s <= 0.0) return;

    // Wheel rotational dynamics:
    // Iw * omega_dot = tau_drive - tau_brake - Fx*R
    const double omega_dot = compute_omega_dot(tau_drive_nm, tau_brake_nm, fx_n);

    double omega_next = omega_radps_ + omega_dot * dt_s;

    // Clamp
    omega_next = clamp(omega_next, p_.omega_min_radps, p_.omega_max_radps);

    // Zero-crossing protection near standstill
    if ((omega_radps_ > 0.0 && omega_next < 0.0) || (omega_radps_ < 0.0 && omega_next > 0.0)) {
        if (std::abs(omega_next) < p_.omega_eps_radps) {
            omega_next = 0.0;
        }
    }

    omega_radps_ = omega_next;
}

double WheelDynamics::compute_omega_dot(double tau_drive_nm, double tau_brake_nm, double fx_n) const {
    // tau_brake_nm is assumed already “direction-correct” (opposes rotation)
    // so it can be positive or negative depending on wheel direction handling upstream.
    const double tau_load = fx_n * p_.radius_m;
    return (tau_drive_nm - tau_brake_nm - tau_load) / p_.inertia_kgm2;
}

double WheelDynamics::compute_slip_ratio(double v_x_mps) const {
    const double v_wheel = omega_radps_ * p_.radius_m;

    const double denom = std::max({std::abs(v_x_mps), std::abs(v_wheel), p_.v_eps_mps});
    return (v_wheel - v_x_mps) / denom;
}

double WheelDynamics::compute_lateral_slip(double v_x_mps, double v_y_mps) const {
    const double denom = std::max(std::abs(v_x_mps), p_.v_eps_mps);
    return v_y_mps / denom;
}

double WheelDynamics::get_stability_dt_max(double cx_n_per_slip) const {
    // Very rough linearized Forward Euler stability bound:
    // dt < 2*Iw / (Cx*R^2)
    const double denom = std::max(1.0, cx_n_per_slip * p_.radius_m * p_.radius_m);
    return 2.0 * p_.inertia_kgm2 / denom;
}

} // namespace plant
