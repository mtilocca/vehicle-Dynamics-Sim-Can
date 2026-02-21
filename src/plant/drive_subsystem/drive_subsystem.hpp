// src/plant/drive_subsystem/drive_subsystem.hpp
//
// DriveSubsystem - Powertrain + linear tire subsystem (priority 100)
//
// Computes per-wheel Fx/Fy/Fz and wheel speeds from motor torque command.
// Executes before VehicleSubsystem (110) so forces are available for 3-DOF integration.

#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/drive_subsystem/drive_plant.hpp"

namespace plant {

class DriveSubsystem : public PhysicsSubsystem {
public:
    explicit DriveSubsystem(const DriveParams& params = {});

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;

    const char* name() const override { return "Drive"; }
    int priority() const override { return 100; }

    const DriveParams& get_params() const { return drive_.params(); }
    void set_params(const DriveParams& params);

private:
    DrivePlant drive_;
};

} // namespace plant