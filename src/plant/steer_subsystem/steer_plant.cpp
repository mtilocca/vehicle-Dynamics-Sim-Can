// src/plant/steer_subsystem/steer_plant.cpp
//
// SteerPlant Implementation
//
// Ackermann Geometry (PDF Section 7):
// - Virtual steering angle δ → turn radius R = L / tan(δ)
// - For right turn (δ > 0): turn center is on the RIGHT
// - Left wheel (FL) is OUTER → turn radius R_FL = R + W/2
// - Right wheel (FR) is INNER → turn radius R_FR = R - W/2
// - Inner wheel turns MORE: δ_FR > δ_FL for right turn
//
// This satisfies the Ackermann condition:
//   cot(δ_outer) - cot(δ_inner) = W / L

#include "steer_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"

namespace plant {

void SteerPlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s) {
    if (dt_s <= 0.0) return;

    const bool enabled = cmd.system_enable;

    // ========================================================================
    // STEP 1: Parse command
    // ========================================================================
    const double delta_cmd_rad = enabled ? deg2rad(cmd.steer_cmd_deg) : 0.0;
    const double base_delta_max_rad = deg2rad(p_.delta_max_deg);

    // ========================================================================
    // STEP 2: Speed-dependent steering reduction
    // ========================================================================
    // At high speed, reduce max steering to improve stability
    // and prevent rollover risk for mining trucks
    //
    // At very low speed, also reduce steering authority to avoid yaw-rate
    // oscillations in the simplified linear tire model.

    double ratio_highv = 1.0;
    if (s.v_mps > p_.v_steer_limit_start_mps) {
        const double v0 = p_.v_steer_limit_start_mps;
        const double v1 = std::max(p_.v_steer_limit_end_mps, v0 + 1e-6);
        const double alpha = clamp((s.v_mps - v0) / (v1 - v0), 0.0, 1.0);
        ratio_highv = (1.0 - alpha) + alpha * p_.steer_limit_ratio_highv;
    }

    // Low-speed steering clamp: ramp from 10% at standstill to 100% at 2 m/s.
    const double v_low0 = 0.0;
    const double v_low1 = 2.0;
    const double alpha_low = clamp((s.v_mps - v_low0) / (v_low1 - v_low0), 0.0, 1.0);
    const double ratio_lowv = 0.1 + 0.9 * alpha_low;

    const double ratio = std::min(ratio_highv, ratio_lowv);
    const double delta_max_rad = base_delta_max_rad * ratio;
    const double delta_des_rad = clamp(delta_cmd_rad, -delta_max_rad, +delta_max_rad);

    // ========================================================================
    // STEP 3: Rate limiting
    // ========================================================================
    // Models hydraulic steering system bandwidth (~45°/s for mining trucks)
    
    const double max_step_rad = deg2rad(p_.steer_rate_dps) * dt_s;
    const double delta_prev_rad = s.steer_virtual_rad;

    const double delta_error = delta_des_rad - delta_prev_rad;
    const double delta_step = clamp(delta_error, -max_step_rad, +max_step_rad);
    const double delta_next_rad = clamp(delta_prev_rad + delta_step, 
                                        -delta_max_rad, +delta_max_rad);

    s.steer_virtual_rad = delta_next_rad;
    s.steer_rate_radps = delta_step / dt_s;

    // ========================================================================
    // STEP 4: Ackermann mapping
    // ========================================================================
    ackermann_map(
        delta_next_rad,
        p_.wheelbase_m,
        p_.track_width_m,
        p_.ackermann_pct,
        s.delta_fl_rad,
        s.delta_fr_rad
    );

    LOG_DEBUG("[SteerPlant] cmd=%.1f°, virtual=%.1f°, FL=%.1f°, FR=%.1f°, rate=%.1f°/s",
              rad2deg(delta_cmd_rad),
              rad2deg(s.steer_virtual_rad),
              rad2deg(s.delta_fl_rad),
              rad2deg(s.delta_fr_rad),
              rad2deg(s.steer_rate_radps));
}

void SteerPlant::ackermann_map(
    double steer_virtual_rad,
    double L,
    double W,
    double ackermann_pct,
    double& delta_fl_rad,
    double& delta_fr_rad)
{
    // ========================================================================
    // Handle straight-ahead case
    // ========================================================================
    if (std::abs(steer_virtual_rad) < 1e-6) {
        delta_fl_rad = 0.0;
        delta_fr_rad = 0.0;
        return;
    }

    // ========================================================================
    // Perfect Ackermann Geometry
    // ========================================================================
    // 
    // For a right turn (δ > 0):
    //   - Turn center is to the RIGHT of the vehicle
    //   - R = L / tan(δ) is the distance from rear axle center to turn center
    //   - Left wheel (FL) is OUTER, farther from turn center
    //   - Right wheel (FR) is INNER, closer to turn center
    //
    // Turn radii:
    //   R_FL = R + W/2  (outer wheel, larger radius)
    //   R_FR = R - W/2  (inner wheel, smaller radius)
    //
    // Steering angles (from tan(δ) = L / R):
    //   δ_FL = atan(L / R_FL) = atan(L / (R + W/2))  → smaller angle
    //   δ_FR = atan(L / R_FR) = atan(L / (R - W/2))  → larger angle
    //
    // For left turn (δ < 0), R < 0, and the roles swap automatically.
    
    const double R = L / std::tan(steer_virtual_rad);
    
    // Turn radii for each wheel
    // Note: For right turn (R > 0), R_FL > R_FR
    //       For left turn (R < 0), R_FL > R_FR (but both negative)
    const double R_FL = R + W / 2.0;  // Left wheel (outer for right turn)
    const double R_FR = R - W / 2.0;  // Right wheel (inner for right turn)
    
    // Perfect Ackermann angles
    double delta_fl_ackermann = std::atan(L / R_FL);
    double delta_fr_ackermann = std::atan(L / R_FR);

    // ========================================================================
    // Blend with Parallel Steering
    // ========================================================================
    // 
    // ackermann_pct = 1.0: Full Ackermann (inner wheel turns more)
    // ackermann_pct = 0.0: Parallel steering (both wheels same angle)
    //
    // Real vehicles use 80-90% Ackermann as a compromise:
    // - Full Ackermann is ideal for low-speed maneuvering (no tire scrub)
    // - Parallel steering is better at high speed (tire wear, stability)
    // 
    // Mining trucks benefit from high Ackermann % due to low-speed operation
    
    const double ack = clamp(ackermann_pct, 0.0, 1.0);
    
    delta_fl_rad = ack * delta_fl_ackermann + (1.0 - ack) * steer_virtual_rad;
    delta_fr_rad = ack * delta_fr_ackermann + (1.0 - ack) * steer_virtual_rad;
}

} // namespace plant
