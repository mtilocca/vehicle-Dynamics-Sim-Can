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
    // Default: XCMG XDE320/360 heavy mining truck
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
 * Implements the hybrid torque-slip model:
 *   Iw·ω̇ = τ_drive - τ_brake - Fx·R
 * 
 * Where:
 *   Iw      = wheel inertia (kg·m²)
 *   ω       = wheel angular velocity (rad/s)
 *   τ_drive = drive torque from motor (Nm)
 *   τ_brake = brake torque (Nm, always opposes motion)
 *   Fx      = tire longitudinal force (N, from Dugoff model)
 *   R       = wheel radius (m)
 * 
 * The key insight is that Fx appears on BOTH sides:
 *   - Accelerates vehicle: m·V̇x = ΣFx
 *   - Decelerates wheel: Iw·ω̇ = τ - Fx·R
 * 
 * This creates the closed-loop feedback that enables:
 *   - Wheel spin (τ_drive >> Fx·R)
 *   - Wheel lock (τ_brake >> Fx·R)
 *   - Slip-limited traction
 */
class WheelDynamics {
public:
    explicit WheelDynamics(const WheelDynamicsParams& params = {})
        : p_(params), omega_radps_(0.0) {}

    // ========================================================================
    // Main Interface
    // ========================================================================

    /**
     * step() - Integrate wheel angular velocity
     * 
     * @param tau_drive_nm  Drive torque (Nm), positive = forward
     * @param tau_brake_nm  Brake torque (Nm), always positive, opposes motion
     * @param fx_n          Tire longitudinal force (N), from Dugoff model
     * @param dt_s          Timestep (s)
     * 
     * Uses Forward Euler with zero-crossing protection.
     * Stability criterion: dt < 2·Iw / (Cx·R²) ≈ 1.9ms for XCMG trucks
     */
    void step(double tau_drive_nm, double tau_brake_nm, double fx_n, double dt_s);

    /**
     * compute_slip_ratio() - Calculate longitudinal slip ratio
     * 
     * @param v_x_mps  Vehicle longitudinal velocity at wheel (m/s)
     * @return         Slip ratio σx (dimensionless)
     * 
     * Sign convention:
     *   σx > 0: Wheel spinning faster than vehicle (drive slip)
     *   σx < 0: Wheel spinning slower than vehicle (brake slip)
     * 
     * Formula: σx = (ω·R - Vx) / max(|Vx|, |ω·R|, ε)
     */
    double compute_slip_ratio(double v_x_mps) const;

    /**
     * compute_lateral_slip() - Calculate lateral slip (slip angle approximation)
     * 
     * @param v_x_mps  Longitudinal velocity (m/s)
     * @param v_y_mps  Lateral velocity (m/s)
     * @return         Lateral slip σy (dimensionless, ≈ tan(α))
     */
    double compute_lateral_slip(double v_x_mps, double v_y_mps) const;

    // ========================================================================
    // State Access
    // ========================================================================

    /// Get current angular velocity (rad/s)
    double omega_radps() const { return omega_radps_; }

    /// Set angular velocity (rad/s) - use for initialization
    void set_omega_radps(double omega) { 
        omega_radps_ = clamp(omega, p_.omega_min_radps, p_.omega_max_radps); 
    }

    /// Initialize from vehicle velocity (kinematic mode)
    void init_from_velocity(double v_mps) {
        omega_radps_ = v_mps / p_.radius_m;
    }

    /// Get wheel linear velocity (m/s)
    double wheel_velocity_mps() const { return omega_radps_ * p_.radius_m; }

    // ========================================================================
    // Diagnostic / Analysis
    // ========================================================================

    /**
     * compute_omega_dot() - Get angular acceleration without integrating
     * 
     * Useful for diagnostics and stability analysis
     */
    double compute_omega_dot(double tau_drive_nm, double tau_brake_nm, double fx_n) const;

    /**
     * get_stability_dt_max() - Maximum stable timestep
     * 
     * For Forward Euler stability: dt < 2·Iw / (Cx·R²)
     * 
     * @param cx_n_per_slip  Tire longitudinal stiffness (N/unit slip)
     * @return               Maximum stable timestep (s)
     */
    double get_stability_dt_max(double cx_n_per_slip) const;

    /// Get parameters (const)
    const WheelDynamicsParams& params() const { return p_; }

    /// Get parameters (mutable)
    WheelDynamicsParams& params() { return p_; }

private:
    WheelDynamicsParams p_;
    double omega_radps_;  // Current angular velocity (rad/s)

    /// Clamp utility
    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    /// Sign function
    static int sgn(double x) { return (x > 0.0) - (x < 0.0); }
};

} // namespace plant