// src/plant/drive_subsystem/drive_subsystem.cpp
//
// DriveSubsystem Implementation
//
// Execution order in SubsystemManager:
//   1. SteerSubsystem (50)     - Computes steering angles
//   2. DriveSubsystem (100)    - Computes tau_drive_*, tau_brake_*
//   3. WheelSubsystem (105)    - Integrates wheel dynamics, computes Fx_*
//   4. BatterySubsystem (150)  - Updates battery state
//   5. VehicleBicycleAckermann - Integrates position and velocity

#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "utils/logging.hpp"

namespace plant {

DriveSubsystem::DriveSubsystem(
    const DriveParams& params,
    BatterySubsystem* battery_subsystem
)
    : drive_(params, nullptr)  // Battery set via set_battery_subsystem()
    , battery_subsystem_(battery_subsystem)
    , available_power_kW_(0.0)
{
}

void DriveSubsystem::initialize(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Initializing: mass=%.0f kg, max_torque=%.0f Nm, max_power=%.0f kW, Resistance: drag_c=%.2f, roll_c=%.1f", 
             drive_.params().mass_kg,
             drive_.params().motor_torque_max_nm,
             drive_.params().motor_power_max_w / 1000.0);
    LOG_INFO("[DriveSubsystem] Brake bias: %.0f%% front / %.0f%% rear",
             drive_.params().brake_bias_front * 100.0,
             (1.0 - drive_.params().brake_bias_front) * 100.0);

    // Initialize velocity to zero
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
    
    // Initialize motor state
    s.motor_torque_nm = 0.0;
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
    
    // Initialize wheel torques to zero
    s.tau_drive_fl_nm = 0.0;
    s.tau_drive_fr_nm = 0.0;
    s.tau_drive_rl_nm = 0.0;
    s.tau_drive_rr_nm = 0.0;
    
    s.tau_brake_fl_nm = 0.0;
    s.tau_brake_fr_nm = 0.0;
    s.tau_brake_rl_nm = 0.0;
    s.tau_brake_rr_nm = 0.0;
}

void DriveSubsystem::reset(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Resetting to zero velocity");
    
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
    s.motor_torque_nm = 0.0;
    s.motor_power_kW = 0.0;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
    
    // Reset wheel torques
    s.tau_drive_fl_nm = 0.0;
    s.tau_drive_fr_nm = 0.0;
    s.tau_drive_rl_nm = 0.0;
    s.tau_drive_rr_nm = 0.0;
    
    s.tau_brake_fl_nm = 0.0;
    s.tau_brake_fr_nm = 0.0;
    s.tau_brake_rl_nm = 0.0;
    s.tau_brake_rr_nm = 0.0;
}

void DriveSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)s;
    (void)cmd;
    (void)dt;
    
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
    // Delegate to DrivePlant
    // DrivePlant outputs:
    //   - tau_drive_rl_nm, tau_drive_rr_nm (for WheelSubsystem)
    //   - tau_brake_*_nm (for WheelSubsystem)
    //   - motor_torque_nm, motor_power_kW (diagnostics)
    //   - In kinematic mode: v_mps, a_long_mps2, wheel_*_rps
    
    drive_.step(s, cmd, dt);
    
    LOG_DEBUG("[DriveSubsystem] v=%.2f m/s, a=%.2f m/s², τ_motor=%.0f Nm, P=%.1f kW",
              s.v_mps, s.a_long_mps2, s.motor_torque_nm, s.motor_power_kW);
    LOG_DEBUG("[DriveSubsystem] τ_drive_rl=%.0f, τ_drive_rr=%.0f, τ_brake_rl=%.0f Nm",
              s.tau_drive_rl_nm, s.tau_drive_rr_nm, s.tau_brake_rl_nm);
}

void DriveSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)s;
    (void)cmd;
    (void)dt;
    // All outputs already in PlantState from step()
}

void DriveSubsystem::set_battery_subsystem(BatterySubsystem* battery) {
    battery_subsystem_ = battery;
    
    // Connect BatteryPlant to DrivePlant for energy tracking
    if (battery) {
        drive_.set_battery_plant(&battery->get_battery_plant());
        LOG_INFO("[DriveSubsystem] ✓ Battery plant connected to DrivePlant");
    }
    
    LOG_INFO("[DriveSubsystem] Battery subsystem connected");
}

void DriveSubsystem::set_params(const DriveParams& params) {
    drive_.params() = params;
    LOG_INFO("[DriveSubsystem] Parameters updated: mass=%.0f kg, brake_bias=%.0f%% front",
             params.mass_kg, params.brake_bias_front * 100.0);
}

} // namespace plant