// src/plant/battery_subsystem.hpp
#pragma once

#include "plant/physics_subsystem.hpp"
#include "plant/battery_plant.hpp"

namespace plant {

/**
 * BatterySubsystem - Energy storage and power management
 * 
 * Wraps existing BatteryPlant as a PhysicsSubsystem.
 * 
 * Dependencies:
 * - Called by DriveSubsystem to request/store power
 * 
 * Outputs:
 * - SOC, voltage, current (written to PlantState)
 * - Available power (queryable by other subsystems)
 */
class BatterySubsystem : public PhysicsSubsystem {
public:
    explicit BatterySubsystem(
        const BatteryPlantParams& battery_params = {},
        const MotorParams& motor_params = {}
    );

    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    const char* name() const override { return "Battery"; }
    int priority() const override { return 150; } // Energy subsystems: 150-199

    // ========================================================================
    // Battery-Specific Interface
    // ========================================================================

    /**
     * request_power() - Request power from battery (called by DriveSubsystem)
     * 
     * @param power_kW - Requested power (positive = discharge, negative = charge)
     * @param dt - Timestep duration
     * 
     * Returns actual power delivered (may be clamped by limits)
     */
    double request_power(double power_kW, double dt);

    /**
     * store_energy() - Store regenerated energy (called by DriveSubsystem)
     * 
     * @param energy_J - Energy to store (Joules)
     * @param regen_power_kW - Instantaneous regen power for current calculation
     */
    void store_energy(double energy_J, double regen_power_kW = 0.0);

    /**
     * get_available_power_kW() - Maximum power battery can deliver now
     */
    double get_available_power_kW() const;

    /**
     * get_soc() - State of charge (0.0 to 1.0)
     */
    double get_soc() const { return battery_.get_soc(); }

    /**
     * get_voltage() - Pack voltage (V)
     */
    double get_voltage() const { return battery_.get_voltage(); }

    /**
     * get_current() - Pack current (A, positive = discharge)
     */
    double get_current() const { return battery_.get_current(); }

    /**
     * set_params() - Update battery parameters at runtime
     */
    void set_params(const BatteryPlantParams& battery_params, const MotorParams& motor_params);

private:
    BatteryPlant battery_;
    BatteryPlantParams params_;
    MotorParams motor_params_;

    // Cached power request from this timestep
    double power_request_kW_ = 0.0;
};

} // namespace plant
