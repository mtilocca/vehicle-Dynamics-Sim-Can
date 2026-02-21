// src/plant/plant_main/plant_state.hpp
#pragma once

#include <cstdint>
#include "sim/actuator_cmd.hpp"  // For GearPosition enum

namespace plant {

// "Truth" vehicle state.
// Populated by subsystems in priority order:
//   50: SteerSubsystem  → delta_fl_rad, delta_fr_rad
//  100: DriveSubsystem  → Fx/Fy/Fz per wheel, wheel_*_rps, motor_torque_nm, brake_force_kN
//  110: VehicleSubsystem → v_mps, vy_mps, yaw_rate_radps, x_m, y_m, yaw_rad
struct PlantState {
    // --- Time
    double t_s = 0.0;

    // --- Pose (global frame)
    double x_m = 0.0;
    double y_m = 0.0;
    double yaw_rad = 0.0;      // heading (psi)

    // --- Kinematics (3-DOF: longitudinal, lateral, yaw)
    double v_mps = 0.0;              // longitudinal speed [m/s]
    double a_long_mps2 = 0.0;       // longitudinal acceleration [m/s²]
    double vy_mps = 0.0;             // lateral velocity at CG [m/s]
    double yaw_rate_radps = 0.0;     // yaw rate [rad/s]
    double a_lat_mps2 = 0.0;        // lateral acceleration [m/s²]

    // --- Steering
    double steer_virtual_rad = 0.0;  // bicycle steer angle δ
    double steer_rate_radps = 0.0;
    double delta_fl_rad = 0.0;       // front-left wheel angle (Ackermann)
    double delta_fr_rad = 0.0;       // front-right wheel angle (Ackermann)

    // --- Gear Position (from CAN command)
    sim::GearPosition gear_position = sim::GearPosition::FORWARD;

    // --- Wheel speeds (derived from v, no slip model)
    double wheel_fl_rps = 0.0;
    double wheel_fr_rps = 0.0;
    double wheel_rl_rps = 0.0;
    double wheel_rr_rps = 0.0;

    // --- Tire forces (N) populated by DriveSubsystem, consumed by VehicleSubsystem
    // Longitudinal: positive = driving force, negative = braking
    double Fx_fl = 0.0;
    double Fx_fr = 0.0;
    double Fx_rl = 0.0;
    double Fx_rr = 0.0;

    // Lateral: positive = force to the left (vehicle frame)
    double Fy_fl = 0.0;
    double Fy_fr = 0.0;
    double Fy_rl = 0.0;
    double Fy_rr = 0.0;

    // Normal loads (N): static weight distribution
    double Fz_fl = 0.0;
    double Fz_fr = 0.0;
    double Fz_rl = 0.0;
    double Fz_rr = 0.0;

    // --- Drivetrain outputs
    double motor_torque_nm = 0.0;   // Applied motor torque [Nm]
    double brake_force_kN = 0.0;    // Total brake force [kN]

    // --- Status
    uint32_t status_flags = 0;

    // ========================================================================
    // VISITOR PATTERN - Field Enumeration for CAN encoding / CSV logging
    // ========================================================================
    template<typename Visitor>
    void accept_fields(Visitor& visitor) const {
        // === VEHICLE DYNAMICS (Frame 0x300: VEHICLE_STATE_1) ===
        visitor.visit("vehicle_speed_mps", v_mps);
        visitor.visit("vehicle_accel_mps2", a_long_mps2);
        visitor.visit("yaw_rate_radps", yaw_rate_radps);
        visitor.visit("status_flags", static_cast<double>(status_flags));

        // === MOTOR STATE (Frame 0x310: MOTOR_STATE_1) ===
        visitor.visit("motor_torque_nm", motor_torque_nm);
        // motor_speed_rpm, motor_power_kw, motor_temp_c → derived in PlantStatePacker

        // === BRAKE STATE (Frame 0x320: BRAKE_STATE) ===
        visitor.visit("brake_force_kn", brake_force_kN);
        // brake_pct_actual, brake_temp_c → derived in PlantStatePacker

        // === POSITION (Frame 0x330: POSITION_STATE) ===
        visitor.visit("pos_x_m", x_m);
        visitor.visit("pos_y_m", y_m);

        // === ORIENTATION (Frame 0x331: ORIENTATION_STATE) ===
        visitor.visit("yaw_rad", yaw_rad);
        // yaw_deg, yaw_rate_dps → derived in PlantStatePacker

        // === DIAGNOSTIC (Frame 0x3F0: DIAGNOSTIC_STATE) ===
        visitor.visit("sim_time_s", t_s);
        // loop_time_us, error_count, status → derived in PlantStatePacker

        // === STEERING (Frame 0x221: STEER_STATE) ===
        visitor.visit("delta_fl_rad", delta_fl_rad);
        visitor.visit("delta_fr_rad", delta_fr_rad);
        // steer_deg, steer_rate_dps, delta_fl/fr_deg, steer_fault → derived

        // === WHEELS (Frame 0x220: WHEELS_1) ===
        visitor.visit("wheel_fl_rps", wheel_fl_rps);
        visitor.visit("wheel_fr_rps", wheel_fr_rps);
        visitor.visit("wheel_rl_rps", wheel_rl_rps);
        visitor.visit("wheel_rr_rps", wheel_rr_rps);
    }
};

} // namespace plant
