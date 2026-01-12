// src/plant/plant_state.hpp
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

    // ========================================================================
    // VISITOR PATTERN - Field Enumeration for Automation
    // ========================================================================
    
    /**
     * Accept a visitor that enumerates all fields with their CAN signal names.
     * 
     * This enables automatic CAN encoding, CSV logging, debugging, etc.
     * without manual field-by-field mapping.
     * 
     * Usage:
     *   auto visitor = [](const char* name, double value) {
     *       std::cout << name << " = " << value << "\n";
     *   };
     *   state.accept_fields(visitor);
     * 
     * The visitor receives:
     *   - Signal name (matches CAN map signal_name column)
     *   - Field value (converted to double)
     */
    template<typename Visitor>
    void accept_fields(Visitor& visitor) const {
        // === VEHICLE DYNAMICS (Frame 0x300: VEHICLE_STATE_1) ===
        visitor.visit("vehicle_speed_mps", v_mps);
        visitor.visit("vehicle_accel_mps2", a_long_mps2);
        visitor.visit("yaw_rate_radps", steer_rate_radps);  // Proxy for yaw rate
        visitor.visit("status_flags", status_flags);
        
        // === MOTOR STATE (Frame 0x310: MOTOR_STATE_1) ===
        visitor.visit("motor_torque_nm", motor_torque_nm);
        visitor.visit("motor_power_kw", motor_power_kW);
        // motor_speed_rpm - calculated in PlantStatePacker
        // motor_temp_c - not yet modeled
        
        // === BRAKE STATE (Frame 0x320: BRAKE_STATE) ===
        visitor.visit("brake_force_kn", brake_force_kN);
        // brake_pct_actual - needs to come from ActuatorCmd
        visitor.visit("regen_power_kw", regen_power_kW);
        // brake_temp_c - not yet modeled
        
        // === POSITION (Frame 0x330: POSITION_STATE) ===
        visitor.visit("pos_x_m", x_m);
        visitor.visit("pos_y_m", y_m);
        
        // === ORIENTATION (Frame 0x331: ORIENTATION_STATE) ===
        // yaw_deg - calculated from yaw_rad
        visitor.visit("yaw_rad", yaw_rad);
        // yaw_rate_dps - calculated from steer_rate_radps
        
        // === DRIVETRAIN (Frame 0x340: DRIVETRAIN_STATE) ===
        // All drivetrain parameters are constants from PlantModelParams
        // These are handled separately in PlantStatePacker
        
        // === DIAGNOSTIC (Frame 0x3F0: DIAGNOSTIC_STATE) ===
        visitor.visit("sim_time_s", t_s);
        // loop_time_us - calculated in sim loop
        // error_count, status - not yet implemented
        
        // === BATTERY (Frame 0x230: BATT_STATE) ===
        visitor.visit("batt_v", batt_v);
        visitor.visit("batt_i", batt_i);
        visitor.visit("batt_soc_pct", batt_soc_pct);
        visitor.visit("batt_temp_c", batt_temp_c);
        // batt_power_kw - calculated from V*I
        
        // === STEERING (Frame 0x221: STEER_STATE) ===
        // steer_deg - calculated from steer_virtual_rad
        // steer_rate_dps - calculated from steer_rate_radps
        // delta_fl_deg, delta_fr_deg - calculated from rad values
        visitor.visit("delta_fl_rad", delta_fl_rad);
        visitor.visit("delta_fr_rad", delta_fr_rad);
        
        // === WHEELS (Frame 0x220: WHEELS_1) ===
        visitor.visit("wheel_fl_rps", wheel_fl_rps);
        visitor.visit("wheel_fr_rps", wheel_fr_rps);
        visitor.visit("wheel_rl_rps", wheel_rl_rps);
        visitor.visit("wheel_rr_rps", wheel_rr_rps);
    }
};

} // namespace plant
