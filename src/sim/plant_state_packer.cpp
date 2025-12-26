// src/sim/plant_state_packer.cpp
#include "plant_state_packer.hpp"
#include <cmath>
#include <algorithm>

namespace sim {

can::SignalMap PlantStatePacker::pack(
    const plant::PlantState& state,
    const can::FrameDef& frame_def
) {
    can::SignalMap signals;
    
    // ========================================================================
    // AUTOMATIC FIELD EXTRACTION via Visitor Pattern
    // ========================================================================
    
    // Create visitor that filters fields by frame definition
    auto visitor = plant::make_visitor([&](const char* name, double value) {
        if (frame_has_signal(frame_def, name)) {
            signals[name] = value;
        }
    });
    
    // Extract all fields that match this frame's signals
    state.accept_fields(visitor);
    
    // ========================================================================
    // ADD DERIVED SIGNALS (calculated from multiple fields)
    // ========================================================================
    
    add_derived_signals(state, frame_def, signals);
    
    return signals;
}

// ============================================================================
// Helper Functions
// ============================================================================

bool PlantStatePacker::frame_has_signal(
    const can::FrameDef& frame_def,
    const std::string& signal_name
) {
    for (const auto& sig : frame_def.signals) {
        if (sig.signal_name == signal_name) {
            return true;
        }
    }
    return false;
}

void PlantStatePacker::add_derived_signals(
    const plant::PlantState& state,
    const can::FrameDef& frame_def,
    can::SignalMap& signals
) {
    // ========================================================================
    // MOTOR STATE (0x310)
    // ========================================================================
    if (frame_has_signal(frame_def, "motor_speed_rpm")) {
        // ω_motor = (v / r_wheel) * N_gear
        // RPM = ω_motor * 60 / (2π)
        const double omega_wheel = state.v_mps / DEFAULT_WHEEL_RADIUS;
        const double omega_motor = omega_wheel * DEFAULT_GEAR_RATIO;
        const double motor_rpm = omega_motor * 60.0 / (2.0 * M_PI);
        signals["motor_speed_rpm"] = motor_rpm;
    }
    
    if (frame_has_signal(frame_def, "motor_temp_c")) {
        // TODO: Implement motor thermal model
        signals["motor_temp_c"] = 25.0;  // Ambient for now
    }
    
    // ========================================================================
    // BATTERY STATE (0x230)
    // ========================================================================
    if (frame_has_signal(frame_def, "batt_power_kw")) {
        // P = V * I
        const double power_kw = (state.batt_v * state.batt_i) / 1000.0;
        signals["batt_power_kw"] = power_kw;
    }
    
    // ========================================================================
    // BRAKE STATE (0x320)
    // ========================================================================
    if (frame_has_signal(frame_def, "brake_pct_actual")) {
        // TODO: Get this from ActuatorCmd or add to PlantState
        signals["brake_pct_actual"] = 0.0;
    }
    
    if (frame_has_signal(frame_def, "brake_temp_c")) {
        // TODO: Implement brake thermal model
        signals["brake_temp_c"] = 25.0;
    }
    
    // ========================================================================
    // ORIENTATION STATE (0x331)
    // ========================================================================
    if (frame_has_signal(frame_def, "yaw_deg")) {
        signals["yaw_deg"] = state.yaw_rad * 180.0 / M_PI;
    }
    
    if (frame_has_signal(frame_def, "yaw_rate_dps")) {
        // Use steer_rate as proxy for yaw rate
        signals["yaw_rate_dps"] = state.steer_rate_radps * 180.0 / M_PI;
    }
    
    // ========================================================================
    // STEERING STATE (0x221)
    // ========================================================================
    if (frame_has_signal(frame_def, "steer_deg")) {
        signals["steer_deg"] = state.steer_virtual_rad * 180.0 / M_PI;
    }
    
    if (frame_has_signal(frame_def, "steer_rate_dps")) {
        signals["steer_rate_dps"] = state.steer_rate_radps * 180.0 / M_PI;
    }
    
    if (frame_has_signal(frame_def, "delta_fl_deg")) {
        signals["delta_fl_deg"] = state.delta_fl_rad * 180.0 / M_PI;
    }
    
    if (frame_has_signal(frame_def, "delta_fr_deg")) {
        signals["delta_fr_deg"] = state.delta_fr_rad * 180.0 / M_PI;
    }
    
    if (frame_has_signal(frame_def, "steer_fault")) {
        // TODO: Implement steering fault detection
        signals["steer_fault"] = 0.0;
    }
    
    // ========================================================================
    // DRIVETRAIN STATE (0x340) - Static Configuration
    // ========================================================================
    if (frame_has_signal(frame_def, "gear_ratio")) {
        signals["gear_ratio"] = DEFAULT_GEAR_RATIO;
    }
    
    if (frame_has_signal(frame_def, "drivetrain_eff_pct")) {
        signals["drivetrain_eff_pct"] = DEFAULT_DRIVETRAIN_EFF * 100.0;
    }
    
    if (frame_has_signal(frame_def, "wheel_radius_mm")) {
        signals["wheel_radius_mm"] = DEFAULT_WHEEL_RADIUS * 1000.0;
    }
    
    if (frame_has_signal(frame_def, "wheelbase_mm")) {
        signals["wheelbase_mm"] = DEFAULT_WHEELBASE * 1000.0;
    }
    
    if (frame_has_signal(frame_def, "track_width_mm")) {
        signals["track_width_mm"] = DEFAULT_TRACK_WIDTH * 1000.0;
    }
    
    // ========================================================================
    // DIAGNOSTIC STATE (0x3F0)
    // ========================================================================
    if (frame_has_signal(frame_def, "loop_time_us")) {
        // TODO: Calculate actual loop time in sim_app
        signals["loop_time_us"] = 10000.0;  // 10ms nominal
    }
    
    if (frame_has_signal(frame_def, "error_count")) {
        signals["error_count"] = 0.0;
    }
    
    if (frame_has_signal(frame_def, "status")) {
        signals["status"] = 0.0;  // 0 = OK
    }
}

} // namespace sim
