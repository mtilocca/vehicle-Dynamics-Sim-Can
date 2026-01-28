// src/plant/steer_subsystem/steer_subsystem.hpp
//
// SteerSubsystem - PhysicsSubsystem wrapper for steering geometry
//
// Priority: 50 (Actuators category)
// Executes BEFORE DriveSubsystem (100) so steering angles are available

#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/steer_subsystem/steer_plant.hpp"

namespace plant {

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
    int priority() const override { return 50; }  // Actuators: 50-99

    // ========================================================================
    // Configuration Interface
    // ========================================================================

    const SteerParams& get_params() const { return steer_.params(); }
    void set_params(const SteerParams& params);

private:
    SteerPlant steer_;
};

} // namespace plant