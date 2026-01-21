// src/plant/tyre_subsystem/tyre_subsystem.cpp
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
    LOG_INFO("[TyreSubsystem] Initialized with default parameters");
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
    
    // Compute initial normal loads (static only)
    compute_normal_loads(s);
}

void TyreSubsystem::reset(PlantState& s) {
    LOG_INFO("[TyreSubsystem] Resetting tire forces");
    initialize(s);
}

void TyreSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Nothing needed in pre_step for now
    (void)s; (void)cmd; (void)dt;
}

void TyreSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    if (!dynamic_model_enabled_) {
        // Dynamic tire model disabled - skip computation
        // Kinematic model in bicycle_ackermann will be used instead
        return;
    }
    
    // ========================================================================
    // Step 1: Compute normal loads (static + dynamic load transfer)
    // ========================================================================
    compute_normal_loads(s);
    
    // ========================================================================
    // Step 2: Compute slip ratios for all wheels
    // ========================================================================
    compute_slip_ratios(s);
    
    // ========================================================================
    // Step 3: Compute tire forces using Dugoff model
    // ========================================================================
    compute_tire_forces(s);
    
    LOG_DEBUG("[TyreSubsystem] Fx_total=%.0f N, Fy_total=%.0f N",
              s.Fx_fl + s.Fx_fr + s.Fx_rl + s.Fx_rr,
              s.Fy_fl + s.Fy_fr + s.Fy_rl + s.Fy_rr);
}

void TyreSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Nothing needed in post_step
    (void)s; (void)cmd; (void)dt;
}

void TyreSubsystem::set_params(const TyreDugoffParams& params) {
    tyre_model_.set_params(params);
    LOG_INFO("[TyreSubsystem] Parameters updated: mu_peak=%.2f", params.mu_peak);
}

// ============================================================================
// Helper Functions
// ============================================================================

void TyreSubsystem::compute_normal_loads(PlantState& s) {
    const double g = 9.81;  // m/s^2
    
    // ========================================================================
    // Static weight distribution (front/rear)
    // ========================================================================
    
    double Wf_static = mass_kg_ * g * (cg_to_rear_m_ / wheelbase_m_);
    double Wr_static = mass_kg_ * g * (cg_to_front_m_ / wheelbase_m_);
    
    // ========================================================================
    // Longitudinal load transfer (acceleration/braking)
    // ========================================================================
    
    double ax = s.a_long_mps2;  // Longitudinal acceleration
    double dW_long = (mass_kg_ * ax * cg_height_m_) / wheelbase_m_;
    
    // During acceleration: weight shifts to rear
    // During braking: weight shifts to front
    double Wf = Wf_static - dW_long;
    double Wr = Wr_static + dW_long;
    
    // ========================================================================
    // Lateral load transfer (cornering) - TODO: Future enhancement
    // ========================================================================
    // For now, assume equal left/right distribution
    
    // Front axle (2 wheels)
    s.Fz_fl = Wf / 2.0;
    s.Fz_fr = Wf / 2.0;
    
    // Rear axle (2 wheels for passenger car, 4 for mining truck)
    // TODO: Handle dual rear axle configuration
    s.Fz_rl = Wr / 2.0;
    s.Fz_rr = Wr / 2.0;
    
    // Prevent negative normal loads (wheels lifting off ground)
    s.Fz_fl = std::max(s.Fz_fl, 100.0);  // Minimum 100N
    s.Fz_fr = std::max(s.Fz_fr, 100.0);
    s.Fz_rl = std::max(s.Fz_rl, 100.0);
    s.Fz_rr = std::max(s.Fz_rr, 100.0);
}

void TyreSubsystem::compute_slip_ratios(PlantState& s) {
    // Longitudinal velocity (body frame)
    double Vx = s.v_mps;
    
    // ========================================================================
    // Wheel angular velocities
    // ========================================================================
    // TODO: Get from wheel speed sensors or DriveSubsystem
    // For now, assume free rolling (no slip)
    
    double omega_fl = Vx / tire_radius_m_;
    double omega_fr = Vx / tire_radius_m_;
    double omega_rl = Vx / tire_radius_m_;
    double omega_rr = Vx / tire_radius_m_;
    
    // ========================================================================
    // Lateral velocities at tire contact patches
    // ========================================================================
    
    double Vy_fl = compute_lateral_velocity_front(s, s.delta_fl_rad);
    double Vy_fr = compute_lateral_velocity_front(s, s.delta_fr_rad);
    double Vy_rl = compute_lateral_velocity_rear(s);
    double Vy_rr = compute_lateral_velocity_rear(s);
    
    // ========================================================================
    // Compute slip ratios (stored in PlantState for diagnostics)
    // ========================================================================
    // Note: Actual slip computation happens in TyreDugoff::compute_forces()
    // These are just for logging/debugging
    
    const double Vx_safe = std::max(std::abs(Vx), 0.5);
    
    s.sigma_x_fl = (omega_fl * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_y_fl = Vy_fl / Vx_safe;
    
    s.sigma_x_fr = (omega_fr * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_y_fr = Vy_fr / Vx_safe;
    
    s.sigma_x_rl = (omega_rl * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_y_rl = Vy_rl / Vx_safe;
    
    s.sigma_x_rr = (omega_rr * tire_radius_m_ - Vx) / Vx_safe;
    s.sigma_y_rr = Vy_rr / Vx_safe;
}

void TyreSubsystem::compute_tire_forces(PlantState& s) {
    // Get wheel states
    double Vx = s.v_mps;
    
    // Assume free rolling for now (omega = V / R)
    // TODO: Get actual wheel speeds from sensors or DriveSubsystem
    double omega_fl = Vx / tire_radius_m_;
    double omega_fr = Vx / tire_radius_m_;
    double omega_rl = Vx / tire_radius_m_;
    double omega_rr = Vx / tire_radius_m_;
    
    // Lateral velocities
    double Vy_fl = compute_lateral_velocity_front(s, s.delta_fl_rad);
    double Vy_fr = compute_lateral_velocity_front(s, s.delta_fr_rad);
    double Vy_rl = compute_lateral_velocity_rear(s);
    double Vy_rr = compute_lateral_velocity_rear(s);
    
    // ========================================================================
    // Compute forces for each wheel using Dugoff model
    // ========================================================================
    
    TyreForces forces_fl = tyre_model_.compute_forces(
        omega_fl, tire_radius_m_, Vx, Vy_fl, s.Fz_fl
    );
    s.Fx_fl = forces_fl.Fx;
    s.Fy_fl = forces_fl.Fy;
    
    TyreForces forces_fr = tyre_model_.compute_forces(
        omega_fr, tire_radius_m_, Vx, Vy_fr, s.Fz_fr
    );
    s.Fx_fr = forces_fr.Fx;
    s.Fy_fr = forces_fr.Fy;
    
    TyreForces forces_rl = tyre_model_.compute_forces(
        omega_rl, tire_radius_m_, Vx, Vy_rl, s.Fz_rl
    );
    s.Fx_rl = forces_rl.Fx;
    s.Fy_rl = forces_rl.Fy;
    
    TyreForces forces_rr = tyre_model_.compute_forces(
        omega_rr, tire_radius_m_, Vx, Vy_rr, s.Fz_rr
    );
    s.Fx_rr = forces_rr.Fx;
    s.Fy_rr = forces_rr.Fy;
}

double TyreSubsystem::compute_lateral_velocity_front(
    PlantState& s,
    double steering_angle
) const {
    // Yaw rate from bicycle model
    double yaw_rate = s.steer_rate_radps;  // Or compute: (v/L)*tan(delta)
    
    // Lateral velocity at front axle due to yaw rotation
    double Vy_yaw = yaw_rate * cg_to_front_m_;
    
    // Add component from steering angle (body frame)
    double Vy_steer = s.v_mps * std::tan(steering_angle);  // Small angle: tan≈sin
    
    // Total lateral velocity (body frame)
    double Vy = Vy_yaw + Vy_steer;
    
    return Vy;
}

double TyreSubsystem::compute_lateral_velocity_rear(PlantState& s) const {
    // Lateral velocity at rear axle (simplified)
    double Vy_center = s.v_mps * std::sin(s.yaw_rad);
    
    // Subtract contribution from yaw rate
    double distance_to_rear = cg_to_rear_m_;
    double yaw_rate = (s.v_mps / wheelbase_m_) * std::tan(s.steer_virtual_rad);
    
    double Vy = Vy_center - yaw_rate * distance_to_rear;
    
    return Vy;
}

} // namespace plant