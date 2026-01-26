// src/plant/wheel_subsystem/wheel_subsystem.cpp
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

// ============================================================================
// Construction
// ============================================================================

WheelSubsystem::WheelSubsystem(const WheelSubsystemParams& params)
    : p_(params)
    , wheel_fl_(params.wheel)
    , wheel_fr_(params.wheel)
    , wheel_rl_(params.wheel)
    , wheel_rr_(params.wheel)
    , tyre_model_(params.tyre_params)  // Use existing TyreDugoff class
{
}

// ============================================================================
// PhysicsSubsystem Interface
// ============================================================================

void WheelSubsystem::initialize(PlantState& s) {
    LOG_INFO("[WheelSubsystem] Initializing: Iw=%.0f kg·m², R=%.2f m, mode=%s",
             p_.wheel.inertia_kgm2, p_.wheel.radius_m,
             p_.dynamic_mode_enabled ? "DYNAMIC" : "KINEMATIC");
    
    // Initialize wheel speeds from vehicle velocity
    wheel_fl_.init_from_velocity(s.v_mps);
    wheel_fr_.init_from_velocity(s.v_mps);
    wheel_rl_.init_from_velocity(s.v_mps);
    wheel_rr_.init_from_velocity(s.v_mps);
    
    // Sync wheel speeds to PlantState (sets both omega_*_radps and wheel_*_rps)
    sync_wheel_speeds_to_plantstate(s);
    
    // Set default normal loads (static, 50/50 distribution)
    const double weight_n = p_.mass_kg * 9.81;
    s.Fz_fl = weight_n * 0.25;
    s.Fz_fr = weight_n * 0.25;
    s.Fz_rl = weight_n * 0.25;
    s.Fz_rr = weight_n * 0.25;
    
    // Initialize model mode flag
    s.dynamic_model_enabled = p_.dynamic_mode_enabled;
}

void WheelSubsystem::reset(PlantState& s) {
    LOG_INFO("[WheelSubsystem] Resetting wheel dynamics");
    
    // Reset wheel speeds to match vehicle velocity
    wheel_fl_.init_from_velocity(s.v_mps);
    wheel_fr_.init_from_velocity(s.v_mps);
    wheel_rl_.init_from_velocity(s.v_mps);
    wheel_rr_.init_from_velocity(s.v_mps);
    
    // Sync wheel speeds to PlantState (sets both omega_*_radps and wheel_*_rps)
    sync_wheel_speeds_to_plantstate(s);
    
    // Clear tire forces
    s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
    s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
    
    // Clear slip ratios
    s.sigma_x_fl = s.sigma_x_fr = s.sigma_x_rl = s.sigma_x_rr = 0.0;
    s.sigma_y_fl = s.sigma_y_fr = s.sigma_y_rl = s.sigma_y_rr = 0.0;
    
    // Reset friction utilization
    s.lambda_fl = s.lambda_fr = s.lambda_rl = s.lambda_rr = 1.0;
    
    any_wheel_saturated_ = false;
}

void WheelSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)cmd;
    (void)dt;
    
    // Update surface friction from params
    s.surface_mu = p_.tyre_params.mu_peak;
    s.surface_mu_slide = p_.tyre_params.mu_slide;
    
    // Compute normal loads (affects tire forces)
    compute_normal_loads(s);
}

void WheelSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)cmd;
    
    if (p_.dynamic_mode_enabled) {
        // ====================================================================
        // DYNAMIC MODE: Independent wheel dynamics
        // ====================================================================
        
        // 1. Compute tire forces from wheel speeds (TyreDugoff computes slip internally)
        compute_tire_forces(s);
        
        // 2. Integrate wheel angular velocities using tire forces
        //    This is the key closed-loop feedback:
        //    Iw·ω̇ = τ_drive - τ_brake - Fx·R
        integrate_wheel_dynamics(s, dt);
        
    } else {
        // ====================================================================
        // KINEMATIC MODE: Wheel speeds derived from vehicle velocity
        // ====================================================================
        update_kinematic_wheel_speeds(s);
        
        // Still compute tire forces for logging/diagnostics
        compute_tire_forces(s);
    }
    
    // Sync wheel speeds to PlantState (for CAN/CSV logging)
    sync_wheel_speeds_to_plantstate(s);
}

void WheelSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)cmd;
    (void)dt;
    
    // Compute total tire forces for vehicle dynamics (for logging)
    double fx_total = s.Fx_fl + s.Fx_fr + s.Fx_rl + s.Fx_rr;
    double fy_total = s.Fy_fl + s.Fy_fr + s.Fy_rl + s.Fy_rr;
    (void)fy_total;  // Suppress unused warning
    
    LOG_DEBUG("[WheelSubsystem] Fx_total=%.1f kN, σx_rl=%.3f, λ_rl=%.2f, saturated=%d",
              fx_total / 1000.0, s.sigma_x_rl, s.lambda_rl, any_wheel_saturated_);
}

// ============================================================================
// Configuration Interface
// ============================================================================

void WheelSubsystem::set_params(const WheelSubsystemParams& params) {
    p_ = params;
    
    // Update wheel instances
    wheel_fl_.params() = params.wheel;
    wheel_fr_.params() = params.wheel;
    wheel_rl_.params() = params.wheel;
    wheel_rr_.params() = params.wheel;
    
    // Update tire model
    tyre_model_.set_params(params.tyre_params);
}

void WheelSubsystem::set_surface_mu(double mu_peak, double mu_slide) {
    p_.tyre_params.mu_peak = mu_peak;
    p_.tyre_params.mu_slide = mu_slide;
    tyre_model_.set_params(p_.tyre_params);
}

void WheelSubsystem::set_dynamic_mode(bool enabled) {
    p_.dynamic_mode_enabled = enabled;
}

double WheelSubsystem::get_stability_dt_max() const {
    // For Forward Euler stability: dt < 2 * Iw / (Cx * R^2)
    double Cx = p_.tyre_params.Cx_base;
    double R = p_.wheel.radius_m;
    double Iw = p_.wheel.inertia_kgm2;
    
    if (Cx <= 0.0 || R <= 0.0) {
        return 1.0;  // No limit
    }
    return 2.0 * Iw / (Cx * R * R);
}

// ============================================================================
// Internal Methods
// ============================================================================

void WheelSubsystem::compute_normal_loads(PlantState& s) {
    // Total vehicle weight
    const double W = p_.mass_kg * 9.81;  // N
    
    // Static load distribution (based on CG position)
    const double L = p_.wheelbase_m;
    const double lf = p_.cg_to_front_m;              // CG to front axle
    const double lr = L - lf;                         // CG to rear axle
    
    const double Wf_static = W * lr / L;  // Front axle static load
    const double Wr_static = W * lf / L;  // Rear axle static load
    
    // Longitudinal load transfer due to acceleration
    // ΔW = m·a·h / L
    const double h = p_.cg_height_m;
    const double dW_long = p_.mass_kg * s.a_long_mps2 * h / L;
    
    // Front axle loses load during acceleration (dW_long > 0)
    // Rear axle gains load during acceleration
    const double Wf = Wf_static - dW_long;
    const double Wr = Wr_static + dW_long;
    
    // TODO: Add lateral load transfer during cornering
    // For now, assume 50/50 left-right distribution
    s.Fz_fl = std::max(0.0, Wf * 0.5);
    s.Fz_fr = std::max(0.0, Wf * 0.5);
    s.Fz_rl = std::max(0.0, Wr * 0.5);
    s.Fz_rr = std::max(0.0, Wr * 0.5);
}

void WheelSubsystem::compute_tire_forces(PlantState& s) {
    // Reset saturation flag
    any_wheel_saturated_ = false;
    
    // Get vehicle velocities
    const double Vx = s.v_mps;
    const double R = p_.wheel.radius_m;
    
    // Compute lateral velocities at each tire contact patch
    double Vy_fl = compute_lateral_velocity_front(s, s.delta_fl_rad);
    double Vy_fr = compute_lateral_velocity_front(s, s.delta_fr_rad);
    double Vy_rl = compute_lateral_velocity_rear(s);
    double Vy_rr = compute_lateral_velocity_rear(s);
    
    // ========================================================================
    // Front-left wheel (non-driven)
    // ========================================================================
    TyreForces forces_fl = tyre_model_.compute_forces(
        wheel_fl_.omega_radps(), R, Vx, Vy_fl, s.Fz_fl);
    s.Fx_fl = forces_fl.Fx;
    s.Fy_fl = forces_fl.Fy;
    s.sigma_x_fl = forces_fl.sigma_x;
    s.sigma_y_fl = forces_fl.sigma_y;
    s.lambda_fl = forces_fl.lambda;
    if (forces_fl.lambda < 1.0) any_wheel_saturated_ = true;
    
    // ========================================================================
    // Front-right wheel (non-driven)
    // ========================================================================
    TyreForces forces_fr = tyre_model_.compute_forces(
        wheel_fr_.omega_radps(), R, Vx, Vy_fr, s.Fz_fr);
    s.Fx_fr = forces_fr.Fx;
    s.Fy_fr = forces_fr.Fy;
    s.sigma_x_fr = forces_fr.sigma_x;
    s.sigma_y_fr = forces_fr.sigma_y;
    s.lambda_fr = forces_fr.lambda;
    if (forces_fr.lambda < 1.0) any_wheel_saturated_ = true;
    
    // ========================================================================
    // Rear-left wheel (driven)
    // ========================================================================
    TyreForces forces_rl = tyre_model_.compute_forces(
        wheel_rl_.omega_radps(), R, Vx, Vy_rl, s.Fz_rl);
    s.Fx_rl = forces_rl.Fx;
    s.Fy_rl = forces_rl.Fy;
    s.sigma_x_rl = forces_rl.sigma_x;
    s.sigma_y_rl = forces_rl.sigma_y;
    s.lambda_rl = forces_rl.lambda;
    if (forces_rl.lambda < 1.0) any_wheel_saturated_ = true;
    
    // ========================================================================
    // Rear-right wheel (driven)
    // ========================================================================
    TyreForces forces_rr = tyre_model_.compute_forces(
        wheel_rr_.omega_radps(), R, Vx, Vy_rr, s.Fz_rr);
    s.Fx_rr = forces_rr.Fx;
    s.Fy_rr = forces_rr.Fy;
    s.sigma_x_rr = forces_rr.sigma_x;
    s.sigma_y_rr = forces_rr.sigma_y;
    s.lambda_rr = forces_rr.lambda;
    if (forces_rr.lambda < 1.0) any_wheel_saturated_ = true;
}

void WheelSubsystem::integrate_wheel_dynamics(PlantState& s, double dt) {
    // Read drive and brake torques from PlantState
    // (set by DriveSubsystem before this executes)
    
    // Front wheels: non-driven, only brake torque
    // Note: Front wheels have zero drive torque for RWD vehicle
    wheel_fl_.step(0.0, s.tau_brake_fl_nm, s.Fx_fl, dt);
    wheel_fr_.step(0.0, s.tau_brake_fr_nm, s.Fx_fr, dt);
    
    // Rear wheels: driven + brake torque
    wheel_rl_.step(s.tau_drive_rl_nm, s.tau_brake_rl_nm, s.Fx_rl, dt);
    wheel_rr_.step(s.tau_drive_rr_nm, s.tau_brake_rr_nm, s.Fx_rr, dt);
    
    // Update PlantState omega values
    s.omega_fl_radps = wheel_fl_.omega_radps();
    s.omega_fr_radps = wheel_fr_.omega_radps();
    s.omega_rl_radps = wheel_rl_.omega_radps();
    s.omega_rr_radps = wheel_rr_.omega_radps();
}

void WheelSubsystem::update_kinematic_wheel_speeds(PlantState& s) {
    // In kinematic mode, wheel speed = vehicle speed / wheel radius
    // No slip possible
    const double omega = s.v_mps / p_.wheel.radius_m;
    
    wheel_fl_.set_omega_radps(omega);
    wheel_fr_.set_omega_radps(omega);
    wheel_rl_.set_omega_radps(omega);
    wheel_rr_.set_omega_radps(omega);
    
    s.omega_fl_radps = omega;
    s.omega_fr_radps = omega;
    s.omega_rl_radps = omega;
    s.omega_rr_radps = omega;
}

void WheelSubsystem::sync_wheel_speeds_to_plantstate(PlantState& s) {
    // Set omega_*_radps fields (primary)
    s.omega_fl_radps = wheel_fl_.omega_radps();
    s.omega_fr_radps = wheel_fr_.omega_radps();
    s.omega_rl_radps = wheel_rl_.omega_radps();
    s.omega_rr_radps = wheel_rr_.omega_radps();
    
    // Convert rad/s to rev/s for backward compatibility
    // wheel_*_rps uses revolutions per second (legacy)
    const double rad_to_rps = 1.0 / (2.0 * M_PI);
    
    s.wheel_fl_rps = wheel_fl_.omega_radps() * rad_to_rps;
    s.wheel_fr_rps = wheel_fr_.omega_radps() * rad_to_rps;
    s.wheel_rl_rps = wheel_rl_.omega_radps() * rad_to_rps;
    s.wheel_rr_rps = wheel_rr_.omega_radps() * rad_to_rps;
}

double WheelSubsystem::compute_lateral_velocity_front(
    const PlantState& s,
    double steering_angle
) const {
    // Vy_front = Vy + ψ̇·d_front + Vx·sin(δ)
    
    // Yaw rate from bicycle model: ψ̇ = v/L * tan(δ)
    double yaw_rate = (s.v_mps / p_.wheelbase_m) * std::tan(s.steer_virtual_rad);
    
    // Lateral velocity at front axle due to yaw rotation
    double Vy_yaw = yaw_rate * p_.cg_to_front_m;
    
    // Add component from steering angle (body frame)
    double Vy_steer = s.v_mps * std::sin(steering_angle);
    
    return Vy_yaw + Vy_steer;
}

double WheelSubsystem::compute_lateral_velocity_rear(const PlantState& s) const {
    // Vy_rear = Vy - ψ̇·d_rear
    
    // Yaw rate from bicycle model
    double yaw_rate = (s.v_mps / p_.wheelbase_m) * std::tan(s.steer_virtual_rad);
    
    // CG to rear = wheelbase - CG to front
    double cg_to_rear = p_.wheelbase_m - p_.cg_to_front_m;
    
    // Lateral velocity at rear axle (reduced by distance from CoG)
    return -yaw_rate * cg_to_rear;
}

} // namespace plant