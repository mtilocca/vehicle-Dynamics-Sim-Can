// src/plant/drive_subsystem.cpp
#include "plant/drive_subsystem.hpp"
#include "utils/logging.hpp"

namespace plant {

DriveSubsystem::DriveSubsystem(
    const DriveParams& params,
    BatterySubsystem* battery_subsystem
)
    : drive_(params, nullptr),  // Will set battery later via set_battery_subsystem
      battery_subsystem_(battery_subsystem),
      available_power_kW_(0.0)
{
}

void DriveSubsystem::initialize(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Initializing: mass=%.0f kg, max_torque=%.0f Nm, max_power=%.0f kW",
             drive_.params().mass_kg,
             drive_.params().motor_torque_max_nm,
             drive_.params().motor_power_max_w / 1000.0);

    // Set initial velocity to zero
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
    s.motor_torque_nm = 0.0;
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
}

void DriveSubsystem::reset(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Resetting to zero velocity");
    
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
    s.motor_torque_nm = 0.0;
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
}

void DriveSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Query available power from battery (if connected)
    if (battery_subsystem_) {
        available_power_kW_ = battery_subsystem_->get_available_power_kW();
        
        LOG_DEBUG("[DriveSubsystem] Available battery power: %.1f kW", available_power_kW_);
    } else {
        // No battery - assume unlimited power
        available_power_kW_ = drive_.params().motor_power_max_w / 1000.0;
    }
}

void DriveSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Delegate to existing DrivePlant implementation
    // DrivePlant internally handles battery interaction via battery_plant_ pointer
    drive_.step(s, cmd, dt);
    
    LOG_DEBUG("[DriveSubsystem] v=%.2f m/s, a=%.2f m/s², torque=%.0f Nm, power=%.1f kW",
              s.v_mps, s.a_long_mps2, s.motor_torque_nm, s.motor_power_kW);
}

void DriveSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    // Nothing to publish - all outputs already in PlantState
    (void)s; (void)cmd; (void)dt;
}

void DriveSubsystem::set_battery_subsystem(BatterySubsystem* battery) {
    battery_subsystem_ = battery;
    
    // ========================================================================
    // CRITICAL FIX: Connect the BatteryPlant to DrivePlant
    // ========================================================================
    // DrivePlant still uses BatteryPlant* interface for energy tracking.
    // This is the missing link that was causing zero power/current!
    // ========================================================================
    if (battery) {
        drive_.set_battery_plant(&battery->get_battery_plant());
        LOG_INFO("[DriveSubsystem] ✓ Battery plant connected to DrivePlant");
    }
    
    LOG_INFO("[DriveSubsystem] Battery subsystem connected");
}

void DriveSubsystem::set_params(const DriveParams& params) {
    drive_.params() = params;
}

} // namespace plant