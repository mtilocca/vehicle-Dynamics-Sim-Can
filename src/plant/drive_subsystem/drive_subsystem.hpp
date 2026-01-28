// src/plant/drive_subsystem/drive_subsystem.hpp
//
// DriveSubsystem - Powertrain subsystem wrapper
//
// This is a PhysicsSubsystem wrapper around DrivePlant.
// It follows the established pattern: BatterySubsystem wraps BatteryPlant.
//
// Priority: 100 (Dynamics category)
// Executes BEFORE WheelSubsystem (105) so tau_* are available

#pragma once

#include "plant/plant_main/physics_subsystem.hpp"
#include "plant/drive_subsystem/drive_plant.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"

namespace plant {

class DriveSubsystem : public PhysicsSubsystem {
public:
    explicit DriveSubsystem(
        const DriveParams& params = {},
        BatterySubsystem* battery_subsystem = nullptr
    );

    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    
    const char* name() const override { return "Drive"; }
    int priority() const override { return 100; }  // Before WheelSubsystem (105)

    // ========================================================================
    // Configuration Interface
    // ========================================================================

    /// Inject battery dependency (must call before first step)
    void set_battery_subsystem(BatterySubsystem* battery);

    /// Get current drive parameters
    const DriveParams& get_params() const { return drive_.params(); }

    /// Update drive parameters at runtime
    void set_params(const DriveParams& params);

private:
    DrivePlant drive_;
    BatterySubsystem* battery_subsystem_;

    // Cached from pre_step
    double available_power_kW_ = 0.0;
};

} // namespace plant