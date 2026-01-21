// src/plant/tyre_subsystem/tyre_subsystem.hpp
#pragma once

#include "plant_main/physics_subsystem.hpp"
#include "tyre_subsystem/tyre_dugoff.hpp"
#include "config/vehicle_config.hpp"

namespace plant {

/**
 * TyreSubsystem - Tire force computation with Dugoff model
 * 
 * Responsibilities:
 * - Compute normal loads from vehicle dynamics (load transfer)
 * - Compute slip ratios from wheel speeds and vehicle velocity
 * - Compute tire forces using Dugoff model (friction-limited)
 * - Publish forces to PlantState for DriveSubsystem to use
 * 
 * Priority: 105 (after Drive=100, before Battery=150)
 * 
 * Dependencies:
 * - DriveSubsystem provides wheel angular velocities
 * - SteerSubsystem provides steering angles for lateral slip
 * 
 * Note: This subsystem bridges kinematic (old) and dynamic (new) models.
 *       Set enable_dynamic_model=true to use tire-limited forces.
 */
class TyreSubsystem : public PhysicsSubsystem {
public:
    /**
     * Constructor from vehicle config tire parameters
     */
    explicit TyreSubsystem(const config::TireParams& tire_params);
    
    /**
     * Constructor from plant model params (for backward compatibility)
     */
    explicit TyreSubsystem(
        double wheelbase_m,
        double track_width_m,
        double mass_kg,
        double cg_height_m = 1.5,
        double cg_to_front_m = 1.4,
        double cg_to_rear_m = 1.4
    );
    
    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================
    
    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    
    const char* name() const override { return "Tyre"; }
    int priority() const override { return 105; }  // After Drive, before Battery
    
    // ========================================================================
    // Tyre-Specific Interface
    // ========================================================================
    
    /**
     * Enable/disable dynamic tire model
     * When disabled, forces are not computed (kinematic model used instead)
     */
    void set_dynamic_model_enabled(bool enabled) { dynamic_model_enabled_ = enabled; }
    bool is_dynamic_model_enabled() const { return dynamic_model_enabled_; }
    
    /**
     * Update tire parameters at runtime
     */
    void set_params(const TyreDugoffParams& params);
    
    /**
     * Get current tire model parameters
     */
    const TyreDugoffParams& get_params() const { return tyre_model_.get_params(); }

private:
    // Tire model
    TyreDugoff tyre_model_;
    
    // Vehicle geometry (for load distribution)
    double wheelbase_m_;
    double track_width_m_;
    double mass_kg_;
    double cg_height_m_;
    double cg_to_front_m_;
    double cg_to_rear_m_;
    
    // Tire radius
    double tire_radius_m_;
    
    // Enable flag for dynamic model
    bool dynamic_model_enabled_;
    
    // ========================================================================
    // Helper Functions
    // ========================================================================
    
    /**
     * Compute normal loads on each wheel
     * Includes static weight distribution + longitudinal load transfer
     */
    void compute_normal_loads(PlantState& s);
    
    /**
     * Compute slip ratios for each wheel
     * Requires wheel angular velocities and vehicle velocity
     */
    void compute_slip_ratios(PlantState& s);
    
    /**
     * Compute tire forces using Dugoff model for each wheel
     * Stores results in PlantState (Fx_fl, Fy_fl, etc.)
     */
    void compute_tire_forces(PlantState& s);
    
    /**
     * Compute lateral velocity at each tire contact patch
     * Depends on vehicle yaw rate and steering angles
     */
    double compute_lateral_velocity_front(PlantState& s, double steering_angle) const;
    double compute_lateral_velocity_rear(PlantState& s) const;
};

} // namespace plant