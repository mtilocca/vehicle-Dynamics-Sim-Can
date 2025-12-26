// src/plant/battery_subsystem.hpp
#pragma once

#include "plant/physics_subsystem.hpp"
#include "plant/battery_plant.hpp"

namespace plant {

/**
 * BatterySubsystem - Energy storage and management
 * 
 * Responsibilities:
 * - SOC tracking
 * - Voltage/current monitoring
 * - Power limiting (charge/discharge)
 * - Energy integration
 * 
 * Dependencies: None (lowest priority subsystem)
 */
class BatterySubsystem : public PhysicsSubsystem {
public:
    explicit BatterySubsystem(
        const BatteryPlantParams& params = {},
        const MotorParams& motor_params = {}
    );

    // ========================================================================
    // PhysicsSubsystem Interface
    // ========================================================================

    void initialize(PlantState& s) override;
    void reset(PlantState& s) override;
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    const char* name() const override { return "Battery"; }
    int priority() const override { return 150; } // Energy: 150-199

    // ========================================================================
    // Battery-Specific Interface
    // ========================================================================

    /**
     * Request power consumption from battery
     * Returns actual power delivered (may be less than requested due to limits)
     */
    double request_power(double power_kW, double dt);

    /**
     * Store regenerated energy in battery
     * Returns actual power stored (may be less than offered due to limits)
     */
    double store_power(double power_kW, double dt);

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
     * get_battery_plant() - Direct access to underlying BatteryPlant
     * CRITICAL FIX: For backward compatibility with DrivePlant
     */
    BatteryPlant& get_battery_plant() { return battery_; }

private:
    BatteryPlant battery_;
    
    // Track power requests for limiting
    double power_requested_kW_;
    double power_delivered_kW_;
};

} // namespace plant