// src/plant/drive_subsystem.hpp
#pragma once

#include "plant/physics_subsystem.hpp"
#include "plant/drive_plant.hpp"
#include "plant/battery_subsystem.hpp"

namespace plant {

/**
 * DriveSubsystem - Longitudinal dynamics and energy flow
 * 
 * Responsibilities:
 * - Motor torque → wheel force conversion
 * - Brake force application
 * - Resistive forces (drag, rolling resistance)
 * - Speed integration
 * - Power/energy management with BatterySubsystem
 * 
 * Dependencies:
 * - BatterySubsystem (for power requests and regen storage)
 */
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
    int priority() const override { return 100; } // Dynamics: 100-149

    // ========================================================================
    // Drive-Specific Interface
    // ========================================================================

    /**
     * set_battery_subsystem() - Inject battery dependency
     * 
     * Must be called before first step() if battery integration is needed.
     */
    void set_battery_subsystem(BatterySubsystem* battery);

    /**
     * get_params() - Get current drive parameters
     */
    const DriveParams& get_params() const { return drive_.params(); }

    /**
     * set_params() - Update drive parameters at runtime
     */
    void set_params(const DriveParams& params);

private:
    DrivePlant drive_;
    BatterySubsystem* battery_subsystem_;

    // Cached values from pre_step
    double available_power_kW_;
};

} // namespace plant
