// src/plant/steer_subsystem.hpp
#pragma once

#include "plant/physics_subsystem.hpp"
#include "plant/steer_plant.hpp"

namespace plant {

/**
 * SteerSubsystem - Steering geometry and Ackermann mapping
 * 
 * Responsibilities:
 * - Virtual steering angle rate limiting
 * - Speed-dependent steering reduction (understeer)
 * - Ackermann geometry mapping (virtual → FL/FR wheel angles)
 * 
 * Dependencies: None
 */
class SteerSubsystem : public PhysicsSubsystem {
public:
    explicit SteerSubsystem(const SteerParams& params = {});

    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    const char* name() const override { return "Steer"; }
    int priority() const override { return 50; } // Actuators: 50-99

    // ========================================================================
    // Steer-Specific Interface
    // ========================================================================

    /**
     * get_params() - Get current steering parameters
     */
    const SteerParams& get_params() const { return steer_.params(); }

    /**
     * set_params() - Update steering parameters at runtime
     */
    void set_params(const SteerParams& params);

private:
    SteerPlant steer_;
};

} // namespace plant
