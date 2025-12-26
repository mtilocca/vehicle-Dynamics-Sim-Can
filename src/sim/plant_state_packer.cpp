// src/sim/plant_state_packer.cpp
#include "plant_state_packer.hpp"
#include <cmath>

namespace sim {

can::SignalMap PlantStatePacker::pack(
    const plant::PlantState& state,
    const can::FrameDef& frame_def
) {
    // Dispatch based on frame ID
    switch (frame_def.frame_id) {
        case 0x300: return pack_vehicle_state_1(state);
        case 0x310: return pack_motor_state_1(state);
        case 0x320: return pack_brake_state(state);
        case 0x330: return pack_position_state(state);
        case 0x331: return pack_orientation_state(state);
        case 0x340: return pack_drivetrain_state(state);
        case 0x3F0: return pack_diagnostic_state(state);
        default:
            // Unknown frame - return empty map
            return can::SignalMap{};
    }
}

can::SignalMap PlantStatePacker::pack_vehicle_state_1(const plant::PlantState& s) {
    can::SignalMap signals;
    
    signals["vehicle_speed_mps"] = s.v_mps;
    signals["vehicle_accel_mps2"] = s.a_long_mps2;
    signals["yaw_rate_radps"] = calc_yaw_rate_radps(s);
    signals["status_flags"] = 0;  // TODO: Implement status flags
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_motor_state_1(const plant::PlantState& s) {
    can::SignalMap signals;
    
    signals["motor_torque_nm"] = s.motor_torque_nm;
    signals["motor_power_kw"] = s.motor_power_kW;
    signals["motor_speed_rpm"] = calc_motor_rpm(s, DEFAULT_GEAR_RATIO, DEFAULT_WHEEL_RADIUS);
    signals["motor_temp_c"] = 25.0;  // TODO: Add motor temperature to PlantState
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_brake_state(const plant::PlantState& s) {
    can::SignalMap signals;
    
    signals["brake_force_kn"] = s.brake_force_kN;
    signals["brake_pct_actual"] = 0.0;  // TODO: Add to PlantState or pass from ActuatorCmd
    signals["regen_power_kw"] = s.regen_power_kW;
    signals["brake_temp_c"] = 25.0;  // TODO: Add brake temperature model
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_position_state(const plant::PlantState& s) {
    can::SignalMap signals;
    
    signals["pos_x_m"] = s.x_m;
    signals["pos_y_m"] = s.y_m;
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_orientation_state(const plant::PlantState& s) {
    can::SignalMap signals;
    
    // Convert yaw to degrees
    const double yaw_deg = s.yaw_rad * 180.0 / M_PI;
    
    signals["yaw_deg"] = yaw_deg;
    signals["yaw_rad"] = s.yaw_rad;
    signals["yaw_rate_dps"] = calc_yaw_rate_radps(s) * 180.0 / M_PI;
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_drivetrain_state(const plant::PlantState& s) {
    can::SignalMap signals;
    
    // Static configuration (should ideally come from PlantModelParams)
    signals["gear_ratio"] = DEFAULT_GEAR_RATIO;
    signals["drivetrain_eff_pct"] = DEFAULT_DRIVETRAIN_EFF * 100.0;
    signals["wheel_radius_mm"] = DEFAULT_WHEEL_RADIUS * 1000.0;
    signals["wheelbase_mm"] = DEFAULT_WHEELBASE * 1000.0;
    signals["track_width_mm"] = DEFAULT_TRACK_WIDTH * 1000.0;
    
    return signals;
}

can::SignalMap PlantStatePacker::pack_diagnostic_state(const plant::PlantState& s) {
    can::SignalMap signals;
    
    signals["sim_time_s"] = s.t_s;
    signals["loop_time_us"] = 10000.0;  // TODO: Calculate actual loop time
    signals["error_count"] = 0;
    signals["status"] = 0;  // 0 = OK
    
    return signals;
}

// ============================================================================
// Helper Functions
// ============================================================================

double PlantStatePacker::calc_motor_rpm(
    const plant::PlantState& s,
    double gear_ratio,
    double wheel_radius
) {
    // ω_motor = (v / r_wheel) * N_gear
    // RPM = ω_motor * 60 / (2π)
    
    if (wheel_radius <= 0.0) return 0.0;
    
    const double omega_wheel_radps = s.v_mps / wheel_radius;
    const double omega_motor_radps = omega_wheel_radps * gear_ratio;
    const double motor_rpm = omega_motor_radps * 60.0 / (2.0 * M_PI);
    
    return motor_rpm;
}

double PlantStatePacker::calc_yaw_rate_radps(const plant::PlantState& s) {
    // For bicycle model: ψ̇ = (v / L) * tan(δ)
    // We can also use steer_rate_radps if available
    
    // Return steer rate as proxy (should be same as yaw rate for bicycle model)
    return s.steer_rate_radps;
    
    // Alternative: Calculate from speed and steering angle
    // const double L = DEFAULT_WHEELBASE;
    // return (s.v_mps / L) * std::tan(s.steer_virtual_rad);
}

} // namespace sim