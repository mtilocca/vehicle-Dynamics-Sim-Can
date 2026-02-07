// src/plant/tyre_subsystem/tyre_dugoff.hpp
#pragma once

#include <cmath>
#include <algorithm>

namespace plant {

/**
 * Dugoff tire model parameters
 */
struct TyreDugoffParams {
    // Slip stiffness coefficients
    double Cx_base = 280000.0;      // Longitudinal slip stiffness [N] at Fz_ref
    double Cy_base = 220000.0;      // Lateral slip stiffness [N] at Fz_ref
    double Fz_ref = 800000.0;       // Reference normal load [N]
    double load_exponent = 0.50;    // Load scaling exponent
    
    // Surface friction
    double mu_peak = 0.72;          // Peak friction coefficient
    double mu_slide = 0.65;         // Sliding friction coefficient
    
    // Velocity fade (optional)
    bool velocity_fade_enabled = false;
    double fade_factor = 0.003;     // Friction reduction per m/s
    double min_friction_ratio = 0.70;  // Don't reduce below 70% of peak
    
    // Slip limits (numerical stability)
    double sigma_x_max = 0.95;      // Max longitudinal slip
    double sigma_y_max = 0.50;      // Max lateral slip
    double v_min = 0.5;             // Min velocity for slip calc [m/s]
};

/**
 * Tire forces output from Dugoff model
 */
struct TyreForces {
    double Fx = 0.0;        // Longitudinal force [N]
    double Fy = 0.0;        // Lateral force [N]
    double sigma_x = 0.0;   // Longitudinal slip ratio
    double sigma_y = 0.0;   // Lateral slip ratio
    double lambda = 0.0;    // Friction utilization parameter
};

/**
 * TyreDugoff - Dugoff tire model implementation
 * 
 * Computes tire forces based on slip ratios and normal load.
 * Handles combined longitudinal and lateral slip with friction saturation.
 * 
 * Reference: Dugoff, H., et al. (1970). "An Analysis of Tire Traction 
 *            Properties and Their Influence on Vehicle Dynamic Performance"
 */
class TyreDugoff {
public:
    explicit TyreDugoff(const TyreDugoffParams& params = TyreDugoffParams());
    
    /**
     * Compute tire forces from slip ratios
     * 
     * @param omega Wheel angular velocity [rad/s]
     * @param R Tire effective radius [m]
     * @param Vx Longitudinal velocity [m/s]
     * @param Vy Lateral velocity at tire contact patch [m/s]
     * @param Fz Normal load [N]
     * @return TyreForces structure with Fx, Fy, slip ratios, lambda
     */
    TyreForces compute_forces(
        double omega,
        double R,
        double Vx,
        double Vy,
        double Fz
    ) const;
    
    /**
     * Update parameters at runtime
     */
    void set_params(const TyreDugoffParams& params) { params_ = params; }
    
    /**
     * Get current parameters
     */
    const TyreDugoffParams& get_params() const { return params_; }

private:
    TyreDugoffParams params_;
    
    /**
     * Compute load-dependent slip stiffness
     * C(Fz) = C_base * (Fz / Fz_ref)^exponent
     */
    double compute_Cx(double Fz) const;
    double compute_Cy(double Fz) const;
    
    /**
     * Compute effective friction coefficient with velocity fade
     */
    double compute_mu_effective(double Vx) const;
    
    /**
     * Compute friction saturation function f(lambda)
     * f(lambda) = (2 - lambda) * lambda  for lambda < 1
     * f(lambda) = 1                      for lambda >= 1
     */
    double friction_saturation(double lambda) const;
    
    /**
     * Clamp slip ratios to prevent numerical issues
     */
    double clamp_sigma_x(double sigma_x) const;
    double clamp_sigma_y(double sigma_y) const;
};

} // namespace plant