// src/plant/tyre_subsystem/tyre_subsystem.cpp
// FIXED: Added standstill/launch handling for vehicle start from rest
#include "tyre_subsystem/tyre_subsystem.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

TyreSubsystem::TyreSubsystem(const config::TireParams& tire_params)
    : tyre_model_(TyreDugoffParams{
          tire_params.Cx_base,
          tire_params.Cy_base,
          tire_params.Fz_ref,
          tire_params.load_exponent,
          tire_params.surface.mu_peak,
          tire_params.surface.mu_slide,
          tire_params.velocity_fade_enabled,
          tire_params.fade_factor,
          tire_params.min_friction_ratio,
          tire_params.sigma_x_max,
          tire_params.sigma_y_max,
          tire_params.v_min_for_slip_calc
      }),
      wheelbase_m_(2.8),        // Default, should be set from config
      track_width_m_(1.6),      // Default
      mass_kg_(1800.0),         // Default
      cg_height_m_(0.5),        // Default
      cg_to_front_m_(1.4),      // Default
      cg_to_rear_m_(1.4),       // Default
      tire_radius_m_(tire_params.radius_m),
      dynamic_model_enabled_(false)  // Start disabled for backward compatibility
{
    LOG_INFO("[TyreSubsystem] Initialized: model=Dugoff, surface=%s, mu_peak=%.2f",
             tire_params.surface.name.c_str(), tire_params.surface.mu_peak);
}

TyreSubsystem::TyreSubsystem(
    double wheelbase_m,
    double track_width_m,
    double mass_kg,
    double cg_height_m,
    double cg_to_front_m,
    double cg_to_rear_m
)
    : tyre_model_(TyreDugoffParams{}),
      wheelbase_m_(wheelbase_m),
      track_width_m_(track_width_m),
      mass_kg_(mass_kg),
      cg_height_m_(cg_height_m),
      cg_to_front_m_(cg_to_front_m),
      cg_to_rear_m_(cg_to_rear_m),
      tire_radius_m_(0.33),
      dynamic_model_enabled_(false)
{
    LOG_INFO("[TyreSubsystem] Initialized with geometry: L=%.2f m, W=%.2f m, m=%.0f kg, h_cg=%.2f m",
             wheelbase_m_, track_width_m_, mass_kg_, cg_height_m_);
}

void TyreSubsystem::initialize(PlantState& s) {
    LOG_INFO("[TyreSubsystem] Initializing tire forces to zero");
    
    // Zero all tire forces
    s.Fx_fl = s.Fy_fl = 0.0;
    s.Fx_fr = s.Fy_fr = 0.0;
    s.Fx_rl = s.Fy_rl = 0.0;
    s.Fx_rr = s.Fy_rr = 0.0;
    
    // Zero slip ratios
    s.sigma_x_fl = s.sigma_y_fl = 0.0;
    s.sigma_x_fr = s.sigma_y_fr = 0.0;
    s.sigma_x_rl = s.sigma_y_rl = 0.0;
    s.sigma_x_rr = s.sigma_y_rr = 0.0;
    
    // Initialize friction utilization to 1.0 (linear regime)
    s.lambda_fl = s.lambda_fr = 1.0;
    s.lambda_rl = s.lambda_rr = 1.0;
    
    // Compute initial normal loads (static only)
    compute_normal_loads(s);
}

void TyreSubsystem::reset(PlantState& s) {
    // Reset is same as initialize for tire subsystem
    initialize(s);
}

void TyreSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    // No pre-step processing needed for tire subsystem
    (void)s;
    (void)cmd;
    (void)dt_s;
}

void TyreSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    // No post-step processing needed for tire subsystem
    (void)s;
    (void)cmd;
    (void)dt_s;
}

void TyreSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    (void)cmd;  // Not used directly
    (void)dt_s; // Not used directly
    
    // ========================================================================
    // Skip if dynamic model is disabled (backward compatibility)
    // ========================================================================
    if (!s.dynamic_model_enabled) {
        // In kinematic mode, tire forces are not used
        // Just ensure normal loads are computed for completeness
        compute_normal_loads(s);
        return;
    }
    
    // ========================================================================
    // DYNAMIC MODEL: Compute tire forces via Dugoff model
    // ========================================================================
    
    // Step 1: Compute normal loads (static + load transfer)
    compute_normal_loads(s);
    
    // Step 2: Compute slip ratios
    compute_slip_ratios(s);
    
    // Step 3: Compute tire forces from Dugoff model
    compute_tire_forces(s);
}

void TyreSubsystem::set_geometry(
    double wheelbase_m,
    double track_width_m,
    double mass_kg,
    double cg_height_m,
    double cg_to_front_m,
    double cg_to_rear_m
) {
    wheelbase_m_ = wheelbase_m;
    track_width_m_ = track_width_m;
    mass_kg_ = mass_kg;
    cg_height_m_ = cg_height_m;
    cg_to_front_m_ = cg_to_front_m;
    cg_to_rear_m_ = cg_to_rear_m;
    
    LOG_INFO("[TyreSubsystem] Geometry updated: L=%.2f m, W=%.2f m, m=%.0f kg, h_cg=%.2f m",
             wheelbase_m_, track_width_m_, mass_kg_, cg_height_m_);
}

// set_tire_radius is defined inline in header

void TyreSubsystem::set_surface_friction(double mu_peak) {
    // Update Dugoff model parameters
    TyreDugoffParams params = tyre_model_.get_params();
    params.mu_peak = mu_peak;
    params.mu_slide = mu_peak * 0.85;  // Typical ratio
    tyre_model_.set_params(params);
    
    LOG_INFO("[TyreSubsystem] Surface friction updated: mu_peak=%.2f, mu_slide=%.2f",
             mu_peak, params.mu_slide);
}

// set_dynamic_model_enabled is defined inline in header

// ============================================================================
// PRIVATE METHODS
// ============================================================================

void TyreSubsystem::compute_normal_loads(PlantState& s) {
    // ========================================================================
    // Static normal load distribution
    // From PDF Section 3.1: Eq. 11-12
    // ========================================================================
    
    const double g = 9.81;  // m/s²
    double W_total = mass_kg_ * g;  // Total weight (N)
    
    // Static distribution based on CoG position
    // Wf_static = W * (d_rear / L) - Eq. 11
    // Wr_static = W * (d_front / L) - Eq. 12
    double Wf_static = W_total * (cg_to_rear_m_ / wheelbase_m_);
    double Wr_static = W_total * (cg_to_front_m_ / wheelbase_m_);
    
    // ========================================================================
    // Longitudinal load transfer (acceleration/braking)
    // From PDF Section 3.2: Eq. 13-15
    // ΔFz = m * a_x * h_cg / L
    // ========================================================================
    
    double ax = s.a_long_mps2;  // Longitudinal acceleration
    double dW_long = (mass_kg_ * ax * cg_height_m_) / wheelbase_m_;
    
    // During acceleration (ax > 0): weight shifts to rear
    // During braking (ax < 0): weight shifts to front
    double Wf = Wf_static - dW_long;
    double Wr = Wr_static + dW_long;
    
    // ========================================================================
    // Lateral load transfer (cornering) - TODO: Future enhancement
    // For now, assume equal left/right distribution
    // ========================================================================
    
    // Front axle (2 wheels) - Eq. 16
    s.Fz_fl = Wf / 2.0;
    s.Fz_fr = Wf / 2.0;
    
    // Rear axle (2 wheels for passenger car, 4 for mining truck)
    // TODO: Handle dual rear axle configuration - Eq. 17
    s.Fz_rl = Wr / 2.0;
    s.Fz_rr = Wr / 2.0;
    
    // Prevent negative normal loads (wheels lifting off ground)
    const double Fz_min = 100.0;  // Minimum 100N to avoid numerical issues
    s.Fz_fl = std::max(s.Fz_fl, Fz_min);
    s.Fz_fr = std::max(s.Fz_fr, Fz_min);
    s.Fz_rl = std::max(s.Fz_rl, Fz_min);
    s.Fz_rr = std::max(s.Fz_rr, Fz_min);
}

void TyreSubsystem::compute_slip_ratios(PlantState& s) {
    // Longitudinal velocity (body frame)
    double Vx = s.v_mps;
    
    // ========================================================================
    // Wheel angular velocities
    // ========================================================================
    // Get from wheel speed sensors in PlantState (rad/s)
    double omega_fl = s.wheel_fl_rps;
    double omega_fr = s.wheel_fr_rps;
    double omega_rl = s.wheel_rl_rps;
    double omega_rr = s.wheel_rr_rps;
    
    // If wheel speeds not set (zero), assume free rolling (no slip)
    if (std::abs(omega_fl) < 1e-6 && std::abs(Vx) > 0.5) {
        omega_fl = Vx / tire_radius_m_;
        omega_fr = Vx / tire_radius_m_;
        omega_rl = Vx / tire_radius_m_;
        omega_rr = Vx / tire_radius_m_;
    }
    
    // ========================================================================
    // Lateral velocities at tire contact patches
    // From PDF Section 5
    // ========================================================================
    
    double Vy_fl = compute_lateral_velocity_front(s, s.delta_fl_rad);
    double Vy_fr = compute_lateral_velocity_front(s, s.delta_fr_rad);
    double Vy_rl = compute_lateral_velocity_rear(s);
    double Vy_rr = compute_lateral_velocity_rear(s);
    
    // ========================================================================
    // Compute slip ratios
    // From PDF Section 4.2: Eq. 30, 34
    // σx = (ω*R - Vx) / Vx
    // σy = Vy / Vx
    // ========================================================================
    
    const double Vx_safe = std::max(std::abs(Vx), 0.5);  // Prevent div by zero
    
    // Longitudinal slip (Eq. 30)
    s.sigma_x_fl = (omega_fl * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_x_fr = (omega_fr * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_x_rl = (omega_rl * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_x_rr = (omega_rr * tire_radius_m_ - Vx) / Vx_safe;
    
    // Lateral slip (Eq. 34)
    s.sigma_y_fl = Vy_fl / Vx_safe;
    s.sigma_y_fr = Vy_fr / Vx_safe;
    s.sigma_y_rl = Vy_rl / Vx_safe;
    s.sigma_y_rr = Vy_rr / Vx_safe;
}

void TyreSubsystem::compute_tire_forces(PlantState& s) {
    // Get wheel states
    double Vx = s.v_mps;
    
    // Get Dugoff parameters for friction limit
    TyreDugoffParams params = tyre_model_.get_params();
    double mu = params.mu_peak;
    
    // ========================================================================
    // TORQUE-BASED TIRE MODEL
    // ========================================================================
    // Since we don't have dynamic wheel speed modeling (wheel speeds are derived
    // from vehicle velocity), the slip-based Dugoff model gives σx ≈ 0 always.
    //
    // Instead, we use a torque-based approach:
    //   1. Compute demanded force from motor torque
    //   2. Limit by available friction (μ × Fz)
    //   3. Compute slip ratio from force (inverse Dugoff)
    //   4. Compute lateral forces using Dugoff with lateral slip
    //
    // This gives realistic traction-limited behavior without needing dynamic
    // wheel speed integration.
    // ========================================================================
    
    // Get drivetrain parameters (should come from config, hardcoded for now)
    const double gear_ratio = 9.0;
    const double drivetrain_eff = 0.92;
    
    // ========================================================================
    // LONGITUDINAL FORCES (Fx) - Torque-based with friction limiting
    // ========================================================================
    
    // Wheel torque from motor (only rear wheels driven - RWD)
    double wheel_torque_nm = s.motor_torque_nm * gear_ratio * drivetrain_eff;
    
    // Demanded force at tire contact patch
    double Fx_demanded = wheel_torque_nm / tire_radius_m_;
    
    // Friction-limited force per rear wheel
    double Fx_max_rl = mu * s.Fz_rl;
    double Fx_max_rr = mu * s.Fz_rr;
    double Fx_max_rear = Fx_max_rl + Fx_max_rr;
    
    // Apply friction limit
    double Fx_rear;
    double traction_ratio;  // How much of friction circle is used (0-1+)
    
    if (std::abs(Fx_demanded) > Fx_max_rear && Fx_max_rear > 0.0) {
        // TRACTION LIMITED - wheels would spin
        Fx_rear = std::copysign(Fx_max_rear, Fx_demanded);
        traction_ratio = std::abs(Fx_demanded) / Fx_max_rear;  // > 1.0
        
        LOG_DEBUG("[TyreSubsystem] TRACTION LIMITED: demanded=%.0f N, max=%.0f N, ratio=%.2f",
                  Fx_demanded, Fx_max_rear, traction_ratio);
    } else {
        // Not limited
        Fx_rear = Fx_demanded;
        traction_ratio = (Fx_max_rear > 0.0) ? std::abs(Fx_demanded) / Fx_max_rear : 0.0;
    }
    
    // Distribute to rear wheels (equal split for straight line, could add diff model)
    s.Fx_rl = Fx_rear / 2.0;
    s.Fx_rr = Fx_rear / 2.0;
    
    // Front wheels have no drive force (RWD)
    s.Fx_fl = 0.0;
    s.Fx_fr = 0.0;
    
    // ========================================================================
    // SLIP RATIOS - Estimated from force using inverse Dugoff relationship
    // ========================================================================
    // For logging/diagnostics, estimate what slip ratio would produce this force
    // σx ≈ Fx / (Cx * Fz) for small slip (linear region)
    
    double Cx = params.Cx_base;  // Longitudinal stiffness
    
    // Estimate slip from force (inverse of linear Dugoff)
    if (s.Fz_rl > 100.0) {
        s.sigma_x_rl = s.Fx_rl / (Cx * s.Fz_rl / params.Fz_ref);
        s.sigma_x_rl = std::max(-0.3, std::min(0.3, s.sigma_x_rl));  // Clamp to realistic range
    } else {
        s.sigma_x_rl = 0.0;
    }
    
    if (s.Fz_rr > 100.0) {
        s.sigma_x_rr = s.Fx_rr / (Cx * s.Fz_rr / params.Fz_ref);
        s.sigma_x_rr = std::max(-0.3, std::min(0.3, s.sigma_x_rr));
    } else {
        s.sigma_x_rr = 0.0;
    }
    
    // Front wheels - no drive, no longitudinal slip
    s.sigma_x_fl = 0.0;
    s.sigma_x_fr = 0.0;
    
    // ========================================================================
    // LATERAL FORCES (Fy) - Dugoff model with lateral slip
    // ========================================================================
    // Lateral slip is computed from steering angles and yaw rate
    // This part of Dugoff works correctly since lateral slip doesn't depend on
    // wheel speed modeling
    
    double Vy_fl = compute_lateral_velocity_front(s, s.delta_fl_rad);
    double Vy_fr = compute_lateral_velocity_front(s, s.delta_fr_rad);
    double Vy_rl = compute_lateral_velocity_rear(s);
    double Vy_rr = compute_lateral_velocity_rear(s);
    
    // Compute lateral slip ratios
    const double Vx_safe = std::max(std::abs(Vx), 0.5);  // Prevent div by zero
    
    s.sigma_y_fl = Vy_fl / Vx_safe;
    s.sigma_y_fr = Vy_fr / Vx_safe;
    s.sigma_y_rl = Vy_rl / Vx_safe;
    s.sigma_y_rr = Vy_rr / Vx_safe;
    
    // Clamp lateral slip to realistic range
    s.sigma_y_fl = std::max(-0.5, std::min(0.5, s.sigma_y_fl));
    s.sigma_y_fr = std::max(-0.5, std::min(0.5, s.sigma_y_fr));
    s.sigma_y_rl = std::max(-0.5, std::min(0.5, s.sigma_y_rl));
    s.sigma_y_rr = std::max(-0.5, std::min(0.5, s.sigma_y_rr));
    
    // Compute lateral forces using Dugoff
    double Cy = params.Cy_base;  // Lateral stiffness (cornering stiffness)
    
    // Combined slip affects available friction for lateral force
    // Use friction circle: Fy_max = sqrt(mu²Fz² - Fx²)
    
    // Front wheels (no drive force, full friction available for cornering)
    double Fy_max_fl = mu * s.Fz_fl;
    double Fy_max_fr = mu * s.Fz_fr;
    
    // Rear wheels (drive force uses some friction, less available for cornering)
    double Fy_max_rl = std::sqrt(std::max(0.0, std::pow(mu * s.Fz_rl, 2) - std::pow(s.Fx_rl, 2)));
    double Fy_max_rr = std::sqrt(std::max(0.0, std::pow(mu * s.Fz_rr, 2) - std::pow(s.Fx_rr, 2)));
    
    // Compute raw lateral force from slip (linear model)
    double Fy_raw_fl = -Cy * s.sigma_y_fl * s.Fz_fl / params.Fz_ref;
    double Fy_raw_fr = -Cy * s.sigma_y_fr * s.Fz_fr / params.Fz_ref;
    double Fy_raw_rl = -Cy * s.sigma_y_rl * s.Fz_rl / params.Fz_ref;
    double Fy_raw_rr = -Cy * s.sigma_y_rr * s.Fz_rr / params.Fz_ref;
    
    // Apply friction limits (saturation)
    s.Fy_fl = std::max(-Fy_max_fl, std::min(Fy_max_fl, Fy_raw_fl));
    s.Fy_fr = std::max(-Fy_max_fr, std::min(Fy_max_fr, Fy_raw_fr));
    s.Fy_rl = std::max(-Fy_max_rl, std::min(Fy_max_rl, Fy_raw_rl));
    s.Fy_rr = std::max(-Fy_max_rr, std::min(Fy_max_rr, Fy_raw_rr));
    
    // ========================================================================
    // FRICTION UTILIZATION (Lambda)
    // ========================================================================
    // Lambda represents how close we are to the friction limit
    // λ > 1: linear region (not saturated)
    // λ ≤ 1: saturated (at friction limit)
    //
    // λ = μFz / sqrt(Fx² + Fy²)  [ratio of available to used friction]
    
    auto compute_lambda = [mu](double Fx, double Fy, double Fz) -> double {
        double F_used = std::sqrt(Fx * Fx + Fy * Fy);
        if (F_used < 1.0) return 10.0;  // Cap at 10 for numerical stability
        double F_available = mu * Fz;
        return std::min(10.0, F_available / F_used);  // Cap at 10
    };
    
    s.lambda_fl = compute_lambda(s.Fx_fl, s.Fy_fl, s.Fz_fl);
    s.lambda_fr = compute_lambda(s.Fx_fr, s.Fy_fr, s.Fz_fr);
    s.lambda_rl = compute_lambda(s.Fx_rl, s.Fy_rl, s.Fz_rl);
    s.lambda_rr = compute_lambda(s.Fx_rr, s.Fy_rr, s.Fz_rr);
}

double TyreSubsystem::compute_lateral_velocity_front(
    PlantState& s,
    double steering_angle
) const {
    // From PDF Section 5.1: Eq. 47
    // Vy_front = Vy + ψ̇·d_front + Vx·sin(δ)
    
    // Yaw rate from bicycle model: ψ̇ = v/L * tan(δ)
    double yaw_rate = (s.v_mps / wheelbase_m_) * std::tan(s.steer_virtual_rad);
    
    // Lateral velocity at front axle due to yaw rotation
    double Vy_yaw = yaw_rate * cg_to_front_m_;
    
    // Add component from steering angle (body frame)
    // For small angles: sin(δ) ≈ tan(δ) ≈ δ
    double Vy_steer = s.v_mps * std::sin(steering_angle);
    
    // Total lateral velocity (body frame)
    double Vy = Vy_yaw + Vy_steer;
    
    return Vy;
}

double TyreSubsystem::compute_lateral_velocity_rear(PlantState& s) const {
    // From PDF Section 5.2: Eq. 48
    // Vy_rear = Vy - ψ̇·d_rear
    
    // Yaw rate from bicycle model
    double yaw_rate = (s.v_mps / wheelbase_m_) * std::tan(s.steer_virtual_rad);
    
    // Lateral velocity at rear axle (reduced by distance from CoG)
    double Vy = -yaw_rate * cg_to_rear_m_;
    
    return Vy;
}

} // namespace plant