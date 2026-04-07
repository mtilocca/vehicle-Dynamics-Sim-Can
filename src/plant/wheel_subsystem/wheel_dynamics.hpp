// src/plant/wheel_subsystem/wheel_dynamics.hpp
#pragma once

#include <cmath>
#include <algorithm>

namespace plant {

/**
 * WheelDynamicsParams - Parameters for single wheel rotational dynamics
 *
 * Based on the equation: Iw·ω̇ = τ_drive - τ_brake - Fx·R
 *
 * Units follow codebase convention: _kgm2, _m, _radps, _nm, _n
 */
struct WheelDynamicsParams {
    // Wheel inertia (kg·m²)
    // Includes tire, rim, brake rotor, hub assembly
    // Default: Heavy-Duty Electric Vehicle heavy mining truck
    double inertia_kgm2 = 1000.0;

    // Effective rolling radius (m)
    // Slightly less than tire radius due to deflection under load
    double radius_m = 1.93;  // 37.00R57 tire

    // Angular velocity limits (rad/s)
    double omega_max_radps = 50.0;   // ~175 km/h at R=1.93m
    double omega_min_radps = -10.0;  // Limited reverse

    // Velocity threshold for slip calculation (m/s)
    // Prevents division by zero at low speeds
    double v_eps_mps = 0.1;

    // Angular velocity threshold for zero-crossing (rad/s)
    // Snap to zero below this to prevent oscillations
    double omega_eps_radps = 0.01;
};

/**
 * WheelDynamics - Rotational dynamics for a single wheel
 *
 * Implements:
 *   Iw·ω̇ = τ_drive - τ_brake - Fx·R
 *
 * Notes:
 * - tau_drive can be +/- (forward/reverse)
 * - tau_brake is expected >= 0 and should oppose motion (handled in WheelSubsystem)
 * - fx_n is the tire longitudinal force produced by the tire model (sign included)
 */
class WheelDynamics {
public:
    explicit WheelDynamics(const WheelDynamicsParams& params = {})
        : p_(params), omega_radps_(0.0) {}

    /**
     * step() - Integrate wheel angular velocity
     *
     * @param tau_drive_nm  Drive torque (Nm), positive = forward
     * @param tau_brake_nm  Brake torque (Nm), always >= 0, should oppose wheel rotation
     * @param fx_n          Tire longitudinal force (N), from Dugoff model
     * @param dt_s          Timestep (s)
     */
    void step(double tau_drive_nm, double tau_brake_nm, double fx_n, double dt_s);

    /**
     * compute_slip_ratio() - Longitudinal slip ratio
     *
     * σx = (ωR - Vx) / max(|Vx|, |ωR|, eps)
     *
     * Works for forward AND reverse:
     * - Vx < 0 is valid
     * - ω can be < 0
     */
    double compute_slip_ratio(double v_x_mps) const;

    /**
     * compute_lateral_slip() - Lateral slip proxy (≈ tan(alpha))
     * σy ≈ Vy / max(|Vx|, eps)
     */
    double compute_lateral_slip(double v_x_mps, double v_y_mps) const;

    // State access
    double omega_radps() const { return omega_radps_; }

    void set_omega_radps(double omega) {
        omega_radps_ = clamp(omega, p_.omega_min_radps, p_.omega_max_radps);
    }

    void init_from_velocity(double v_mps) {
        omega_radps_ = v_mps / p_.radius_m;
    }

    double wheel_velocity_mps() const { return omega_radps_ * p_.radius_m; }

    // Diagnostics
    double compute_omega_dot(double tau_drive_nm, double tau_brake_nm, double fx_n) const;

    double get_stability_dt_max(double cx_n_per_slip) const;

    const WheelDynamicsParams& params() const { return p_; }
    WheelDynamicsParams& params() { return p_; }

private:
    WheelDynamicsParams p_;
    double omega_radps_;

    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }
};

} // namespace plant
