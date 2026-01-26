// src/plant/wheel_subsystem/wheel_dynamics.cpp
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include <cmath>

namespace plant {

void WheelDynamics::step(double tau_drive_nm, double tau_brake_nm, double fx_n, double dt_s) {
    if (dt_s <= 0.0) return;

    // ========================================================================
    // Brake Torque Sign Handling
    // ========================================================================
    // τ_brake is always positive input, but opposes wheel motion
    // Apply sign based on current angular velocity
    double tau_brake_signed = tau_brake_nm * static_cast<double>(sgn(omega_radps_));
    
    // If wheel nearly stopped, brake opposes drive torque instead
    if (std::abs(omega_radps_) < p_.omega_eps_radps) {
        tau_brake_signed = tau_brake_nm * static_cast<double>(sgn(tau_drive_nm));
    }

    // ========================================================================
    // Angular Acceleration: ω̇ = (τ_drive - τ_brake - Fx·R) / Iw
    // ========================================================================
    const double tau_reaction_nm = fx_n * p_.radius_m;  // Tire force reaction torque
    const double tau_net_nm = tau_drive_nm - tau_brake_signed - tau_reaction_nm;
    const double omega_dot = tau_net_nm / p_.inertia_kgm2;

    // ========================================================================
    // Forward Euler Integration with Zero-Crossing Protection
    // ========================================================================
    double omega_next = omega_radps_ + omega_dot * dt_s;

    // Zero-crossing detection: prevent sign change if result is small
    // This avoids oscillation when wheel comes to rest
    if (omega_radps_ > p_.omega_eps_radps && omega_next < 0.0) {
        // Was positive, about to go negative
        if (std::abs(omega_next) < p_.omega_eps_radps) {
            omega_next = 0.0;  // Snap to zero
        }
    } else if (omega_radps_ < -p_.omega_eps_radps && omega_next > 0.0) {
        // Was negative, about to go positive
        if (std::abs(omega_next) < p_.omega_eps_radps) {
            omega_next = 0.0;  // Snap to zero
        }
    }

    // Clamp to physical limits
    omega_radps_ = clamp(omega_next, p_.omega_min_radps, p_.omega_max_radps);
}

double WheelDynamics::compute_slip_ratio(double v_x_mps) const {
    // Wheel linear velocity
    const double v_wheel = omega_radps_ * p_.radius_m;
    
    // Denominator: max(|Vx|, |v_wheel|, ε)
    // This formulation works for both braking and driving
    const double denom = std::max({std::abs(v_x_mps), std::abs(v_wheel), p_.v_eps_mps});
    
    // Slip ratio: σx = (ω·R - Vx) / denom
    // σx > 0: wheel spinning faster (drive slip / wheel spin)
    // σx < 0: wheel spinning slower (brake slip / wheel lock)
    return (v_wheel - v_x_mps) / denom;
}

double WheelDynamics::compute_lateral_slip(double v_x_mps, double v_y_mps) const {
    // Lateral slip ≈ slip angle for small angles
    // σy = Vy / max(|Vx|, ε)
    const double denom = std::max(std::abs(v_x_mps), p_.v_eps_mps);
    return v_y_mps / denom;
}

double WheelDynamics::compute_omega_dot(double tau_drive_nm, double tau_brake_nm, double fx_n) const {
    // Same calculation as step(), but without integration
    double tau_brake_signed = tau_brake_nm * static_cast<double>(sgn(omega_radps_));
    if (std::abs(omega_radps_) < p_.omega_eps_radps) {
        tau_brake_signed = tau_brake_nm * static_cast<double>(sgn(tau_drive_nm));
    }
    
    const double tau_reaction_nm = fx_n * p_.radius_m;
    const double tau_net_nm = tau_drive_nm - tau_brake_signed - tau_reaction_nm;
    return tau_net_nm / p_.inertia_kgm2;
}

double WheelDynamics::get_stability_dt_max(double cx_n_per_slip) const {
    // For Forward Euler stability of the linearized system:
    // dt_max = 2 * Iw / (Cx * R^2)
    // 
    // Derivation: The wheel dynamics form a stiff system when tire stiffness is high.
    // The eigenvalue of the linearized system is λ = -Cx*R²/Iw
    // Forward Euler is stable when |1 + λ*dt| < 1, giving dt < 2/|λ|
    
    if (cx_n_per_slip <= 0.0) {
        return 1.0;  // No limit if no tire stiffness
    }
    
    const double r_sq = p_.radius_m * p_.radius_m;
    return 2.0 * p_.inertia_kgm2 / (cx_n_per_slip * r_sq);
}

} // namespace plant