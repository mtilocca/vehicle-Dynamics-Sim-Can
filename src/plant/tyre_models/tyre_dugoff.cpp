// src/plant/tyre_subsystem/tyre_dugoff.cpp
// FIXED: Added lambda clamping to prevent 1e11 explosion
#include "tyre_models/tyre_dugoff.hpp"
#include "utils/logging.hpp"
#include <cmath>
#include <algorithm>

namespace plant {

TyreDugoff::TyreDugoff(const TyreDugoffParams& params)
    : params_(params)
{
    LOG_DEBUG("[TyreDugoff] Initialized: Cx_base=%.0f N, Cy_base=%.0f N, mu_peak=%.2f",
              params_.Cx_base, params_.Cy_base, params_.mu_peak);
}

TyreForces TyreDugoff::compute_forces(
    double omega,
    double R,
    double Vx,
    double Vy,
    double Fz,
    int /*gear_dir*/
) const {
    TyreForces forces;
    
    // Prevent division by zero at low speeds
    const double Vx_safe = std::max(std::abs(Vx), params_.v_min);
    
    // ========================================================================
    // Step 1: Compute slip ratios
    // ========================================================================
    
    // Longitudinal slip: sigma_x = (omega * R - Vx) / Vx
    forces.sigma_x = (omega * R - Vx) / Vx_safe;
    forces.sigma_x = clamp_sigma_x(forces.sigma_x);
    
    // Lateral slip: sigma_y = Vy / Vx (small angle approximation)
    forces.sigma_y = Vy / Vx_safe;
    forces.sigma_y = clamp_sigma_y(forces.sigma_y);
    
    // ========================================================================
    // Step 2: Compute load-dependent slip stiffness
    // ========================================================================
    
    double Cx = compute_Cx(Fz);
    double Cy = compute_Cy(Fz);
    
    // ========================================================================
    // Step 3: Compute friction utilization parameter lambda
    // ========================================================================
    
    // Effective friction coefficient (with velocity fade if enabled)
    double mu = compute_mu_effective(Vx);
    
    // Combined slip magnitude
    double slip_magnitude = std::sqrt(
        Cx * Cx * forces.sigma_x * forces.sigma_x +
        Cy * Cy * forces.sigma_y * forces.sigma_y
    );
    
    // Add small epsilon to prevent division by zero
    const double epsilon = 1e-6;
    slip_magnitude = std::max(slip_magnitude, epsilon);
    
    // Lambda = (mu * Fz) / (2 * slip_magnitude)
    forces.lambda = (mu * Fz) / (2.0 * slip_magnitude);
    
    // ========================================================================
    // FIX #1: Clamp lambda to prevent numerical overflow
    // ========================================================================
    // During cruise, tire forces approach zero → slip_magnitude → 0 → lambda → ∞
    // Lambda > 1000 indicates "lots of available grip" but exact value is not meaningful
    // This prevents lambda from exploding to 1e11 and breaking graphs
    forces.lambda = std::min(forces.lambda, 1000.0);
    
    // ========================================================================
    // Step 4: Compute friction saturation function f(lambda)
    // ========================================================================
    
    double f_lambda = friction_saturation(forces.lambda);
    
    // ========================================================================
    // Step 5: Compute tire forces
    // ========================================================================

    // Longitudinal force sign convention:
    // sigma_x = (omega*R - Vx) / Vx_safe already encodes the correct sign:
    //   Forward driving  (omega*R > Vx,  sigma_x > 0) → Fx > 0  (propels vehicle)
    //   Forward braking  (omega*R < Vx,  sigma_x < 0) → Fx < 0  (retards vehicle)
    //   Reverse driving  (omega*R < Vx,  sigma_x < 0) → Fx < 0  (propels vehicle backward)
    //   Reverse braking  (omega*R > Vx,  sigma_x > 0) → Fx > 0  (retards rearward motion)
    // No gear_dir factor needed — sigma_x captures direction automatically.
    // NOTE: This Fx is also passed to WheelDynamics as the load torque:
    //   omega_dot = (tau_drive - tau_brake - Fx*R) / Iw
    // Positive Fx correctly opposes wheel spin-up during traction.

    forces.Fx = Cx * forces.sigma_x * f_lambda;

    // Lateral force opposes lateral slip (independent of longitudinal direction)
    forces.Fy = -Cy * forces.sigma_y * f_lambda;
    
    // ========================================================================
    // Verification: Total force should never exceed mu * Fz
    // ========================================================================
    
    double F_total = std::sqrt(forces.Fx * forces.Fx + forces.Fy * forces.Fy);
    double F_max = mu * Fz;
    
    if (F_total > F_max + 1.0) {  // Allow 1N tolerance for numerical error
        LOG_WARN("[TyreDugoff] Force exceeds friction limit: %.0f N > %.0f N (mu=%.2f, Fz=%.0f N)",
                 F_total, F_max, mu, Fz);
    }
    
    return forces;
}

double TyreDugoff::compute_Cx(double Fz) const {
    // Load-dependent scaling: Cx(Fz) = Cx_base * (Fz / Fz_ref)^exponent
    double load_ratio = Fz / params_.Fz_ref;
    double Cx = params_.Cx_base * std::pow(load_ratio, params_.load_exponent);
    
    return Cx;
}

double TyreDugoff::compute_Cy(double Fz) const {
    // Load-dependent scaling: Cy(Fz) = Cy_base * (Fz / Fz_ref)^exponent
    double load_ratio = Fz / params_.Fz_ref;
    double Cy = params_.Cy_base * std::pow(load_ratio, params_.load_exponent);
    
    return Cy;
}

double TyreDugoff::compute_mu_effective(double Vx) const {
    if (!params_.velocity_fade_enabled) {
        return params_.mu_peak;
    }
    
    // Velocity-dependent friction reduction
    // mu_effective = mu_peak * (1 - fade_factor * |V|)
    double speed = std::abs(Vx);
    double reduction_factor = 1.0 - params_.fade_factor * speed;
    
    // Clamp to minimum friction ratio
    reduction_factor = std::max(reduction_factor, params_.min_friction_ratio);
    
    double mu_effective = params_.mu_peak * reduction_factor;
    
    return mu_effective;
}

double TyreDugoff::friction_saturation(double lambda) const {
    if (lambda >= 1.0) {
        // Linear regime - no saturation
        return 1.0;
    } else {
        // Nonlinear regime - friction saturation
        // f(lambda) = (2 - lambda) * lambda = 2*lambda - lambda^2
        return (2.0 - lambda) * lambda;
    }
}

double TyreDugoff::clamp_sigma_x(double sigma_x) const {
    return std::clamp(sigma_x, -params_.sigma_x_max, params_.sigma_x_max);
}

double TyreDugoff::clamp_sigma_y(double sigma_y) const {
    return std::clamp(sigma_y, -params_.sigma_y_max, params_.sigma_y_max);
}

} // namespace plant