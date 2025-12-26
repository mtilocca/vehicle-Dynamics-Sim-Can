#pragma once

#include <cstdint>

namespace plant {

// "Truth" vehicle + subsystem state.
// Keep this as the single source of truth for sensors + CAN.
// Units are embedded in field names.
struct PlantState {
    // --- Time (optional but useful for logging / sensor timestamps)
    double t_s = 0.0;

    // --- Pose (rear-axle center reference for kinematic bicycle)
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;      // heading (psi)

    // --- Kinematics
    double v_mps = 0.0;        // longitudinal speed at rear axle reference
    double a_long_mps2 = 0.0;

    // --- Steering (virtual bicycle steer + physical wheel angles)
    double steer_virtual_rad = 0.0;  // bicycle steer angle δ
    double steer_rate_radps = 0.0;
    double delta_fl_rad = 0.0;       // front-left wheel angle (Ackermann)
    double delta_fr_rad = 0.0;       // front-right wheel angle (Ackermann)

    // --- Wheel speeds (optional V1; if you don't model wheels yet, keep derived)
    double wheel_fl_rps = 0.0;
    double wheel_fr_rps = 0.0;
    double wheel_rl_rps = 0.0;
    double wheel_rr_rps = 0.0;

    // --- Battery "truth" (simple V1)
    double batt_soc_pct = 50.0;      // 0..100
    double batt_v = 400.0;           // pack voltage
    double batt_i = 0.0;             // + discharge, - charge (choose and stick)
    double batt_temp_c = 25.0;
    bool   batt_contactor_on = true;

    // --- Motor & Energy (new additions)
    double motor_torque_nm = 0.0;    // Current motor torque in Nm
    double motor_power_kW = 0.0;     // Motor power (kW), positive for consumption, negative for regen
    double regen_power_kW = 0.0;     // Regenerative braking power (kW)

    // --- Brake Force (new addition for logging and calculation)
    double brake_force_kN = 0.0;     // Brake force in kN

    // --- Fault/status bits you may want to expose later
    uint32_t status_flags = 0;
};

} // namespace plant
