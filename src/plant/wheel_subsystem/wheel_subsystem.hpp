// src/plant/wheel_subsystem/wheel_subsystem.hpp
#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/tyre_subsystem/tyre_dugoff.hpp"

namespace plant {

/**
 * WheelSubsystemParams - Configuration for wheel dynamics subsystem
 * 
 * Contains geometry and physics parameters for 4-wheel dynamics.
 */
struct WheelSubsystemParams {
    // Individual wheel parameters (same for all wheels by default)
    WheelDynamicsParams wheel{};
    
    // Vehicle geometry
    double wheelbase_m = 5.92;     // Front-to-rear axle distance (XCMG XDE320)
    double track_width_m = 4.20;   // Left-to-right wheel distance
    double cg_height_m = 2.5;      // Center of gravity height (for load transfer)
    double cg_to_front_m = 2.96;   // CG distance to front axle (50% weight dist)
    
    // Vehicle mass (for load transfer calculation)
    double mass_kg = 220000.0;     // XCMG XDE320 operating mass
    
    // Tire parameters (passed to TyreDugoff)
    TyreDugoffParams tyre_params{};
    
    // Model mode
    bool dynamic_mode_enabled = false;  // true = hybrid model with wheel dynamics
};

/**
 * WheelSubsystem - Wheel rotational dynamics and tire force computation
 * 
 * Responsibilities:
 * - Independent wheel angular velocity integration (4 wheels)
 * - Slip ratio computation from wheel speeds vs vehicle velocity
 * - Tire force computation via existing TyreDugoff model
 * - Load transfer calculation
 * - Surface friction management
 * 
 * Operation Modes:
 * 1. KINEMATIC (dynamic_mode_enabled = false):
 *    - Wheel speeds derived from vehicle velocity: ω = V/R
 *    - No wheel spin/lock possible
 *    - Simple, stable, fast
 * 
 * 2. DYNAMIC (dynamic_mode_enabled = true):
 *    - Independent wheel angular velocity integration
 *    - Slip computed from actual ω vs vehicle velocity
 *    - Enables wheel spin, wheel lock, TCS/ABS testing
 *    - Requires small timestep (< 2ms for stability)
 * 
 * Dependencies:
 * - DriveSubsystem (reads wheel torques from PlantState)
 * - SteerSubsystem (reads steering angles from PlantState)
 * 
 * Priority: 105 (after Drive=100, before Battery=150)
 * This ensures torques are computed before tire forces are applied.
 */
class WheelSubsystem : public PhysicsSubsystem {
public:
    explicit WheelSubsystem(const WheelSubsystemParams& params = {});

    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    const char* name() const override { return "Wheel"; }
    int priority() const override { return 105; }  // After Drive(100), before Battery(150)

    // ========================================================================
    // Configuration Interface
    // ========================================================================

    /// Get current parameters
    const WheelSubsystemParams& get_params() const { return p_; }

    /// Update parameters at runtime
    void set_params(const WheelSubsystemParams& params);

    /// Set surface friction coefficient
    void set_surface_mu(double mu_peak, double mu_slide);

    /// Enable/disable dynamic tire model
    void set_dynamic_mode(bool enabled);

    // ========================================================================
    // Diagnostic Interface
    // ========================================================================

    /// Check if any wheel is in friction-limited region (λ < 1)
    bool any_wheel_saturated() const { return any_wheel_saturated_; }

    /// Get maximum stable timestep for current parameters
    double get_stability_dt_max() const;

private:
    WheelSubsystemParams p_;
    
    // Four wheel dynamics instances (FL, FR, RL, RR)
    WheelDynamics wheel_fl_;
    WheelDynamics wheel_fr_;
    WheelDynamics wheel_rl_;
    WheelDynamics wheel_rr_;
    
    // Existing TyreDugoff model (reuse, don't duplicate!)
    TyreDugoff tyre_model_;
    
    // Diagnostic state
    bool any_wheel_saturated_ = false;

    // ========================================================================
    // Internal Methods
    // ========================================================================

    /// Compute normal loads with longitudinal load transfer
    void compute_normal_loads(PlantState& s);

    /// Compute tire forces using TyreDugoff model
    void compute_tire_forces(PlantState& s);

    /// Integrate wheel angular velocities (dynamic mode only)
    void integrate_wheel_dynamics(PlantState& s, double dt);

    /// Update wheel speeds from vehicle velocity (kinematic mode)
    void update_kinematic_wheel_speeds(PlantState& s);

    /// Copy omega to wheel_*_rps fields (for backward compatibility)
    void sync_wheel_speeds_to_plantstate(PlantState& s);
    
    /// Compute lateral velocity at front tire contact patch
    double compute_lateral_velocity_front(const PlantState& s, double steering_angle) const;
    
    /// Compute lateral velocity at rear tire contact patch
    double compute_lateral_velocity_rear(const PlantState& s) const;
};

} // namespace plant