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
    void pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override { (void)s; (void)cmd; (void)dt; }
    void step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override;
    void post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) override { (void)s; (void)cmd; (void)dt; }
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
     */
    void store_energy(double energy_J, double regen_power_kW);

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

    /**
     * set_params() - Update battery parameters at runtime
     * Called by PlantModel::set_params()
     */
    void set_params(const BatteryPlantParams& battery_params, const MotorParams& motor_params);

private:
    BatteryPlant battery_;
    BatteryPlantParams params_;
    MotorParams motor_params_;
    
    // Track power requests for diagnostics
    double power_request_kW_ = 0.0;
};

} // namespace plant