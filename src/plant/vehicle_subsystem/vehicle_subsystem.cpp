// src/plant/vehicle_subsystem/vehicle_subsystem.cpp
// DEBUG VERSION: Added detailed logging to track acceleration sign
//
// Execution order in SubsystemManager:
//   1. SteerSubsystem (50)      - Computes steering angles
//   2. DriveSubsystem (100)     - Computes tau_drive_*, tau_brake_*
//   3. WheelSubsystem (105)     - Integrates wheel dynamics, computes Fx_*
//   4. BatterySubsystem (150)   - Updates battery state
//   5. VehicleSubsystem (110)   - Integrates position and velocity ← THIS

#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

VehicleSubsystem::VehicleSubsystem(const VehicleParams& params)
    : p_(params)
{
}

void VehicleSubsystem::initialize(PlantState& s) {
    LOG_INFO("[VehicleSubsystem] Initializing: mass=%.0f kg, wheelbase=%.2f m",
             p_.mass_kg, p_.wheelbase_m);
    LOG_INFO("[VehicleSubsystem] Resistance: drag_c=%.2f, roll_c=%.1f N",
             p_.drag_c, p_.roll_c);
    
    // Initialize vehicle state to rest
    s.x_m = 0.0;
    s.y_m = 0.0;
    s.yaw_rad = 0.0;
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
}

void VehicleSubsystem::reset(PlantState& s) {
    LOG_INFO("[VehicleSubsystem] Resetting to origin");
    
    s.x_m = 0.0;
    s.y_m = 0.0;
    s.yaw_rad = 0.0;
    s.v_mps = 0.0;
    s.a_long_mps2 = 0.0;
}

void VehicleSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)cmd;  // Not used - we read tire forces from PlantState
    
    if (dt <= 0.0) return;
    
    // ========================================================================
    // LONGITUDINAL DYNAMICS (PDF Eq. 111)
    // ========================================================================
    
    // Sum tire forces from all wheels (computed by WheelSubsystem)
    const double Fx_total = s.Fx_fl + s.Fx_fr + s.Fx_rl + s.Fx_rr;
    
    // Compute resistive forces
    double F_drag = 0.0;
    double F_roll = 0.0;
    compute_resistive_forces(s.v_mps, F_drag, F_roll);
    
    // Net longitudinal force
    const double F_net = Fx_total - F_drag - F_roll;
    
    // Longitudinal acceleration: a = F_net / m
    const double a_long = F_net / p_.mass_kg;
    
    // DEBUG LOGGING FOR ACCELERATION SIGN BUG
    static int debug_ctr = 0;
    if (debug_ctr++ % 50 == 0) {  // Every 0.5 seconds
        LOG_INFO("========== ACCELERATION DEBUG ==========");
        LOG_INFO("v_mps = %.3f m/s", s.v_mps);
        LOG_INFO("Fx_total = %.1f N (FL=%.1f, FR=%.1f, RL=%.1f, RR=%.1f)",
                 Fx_total, s.Fx_fl, s.Fx_fr, s.Fx_rl, s.Fx_rr);
        LOG_INFO("F_drag = %.1f N (opposes motion)", F_drag);
        LOG_INFO("F_roll = %.1f N (opposes motion)", F_roll);
        LOG_INFO("F_net = Fx_total - F_drag - F_roll = %.1f - %.1f - %.1f = %.1f N",
                 Fx_total, F_drag, F_roll, F_net);
        LOG_INFO("a_long = F_net / mass = %.1f / %.0f = %.6f m/s²",
                 F_net, p_.mass_kg, a_long);
        LOG_INFO("========================================");
    }
    
    // Forward Euler integration: v[k+1] = v[k] + a·Δt
    double v_next = s.v_mps + a_long * dt;
    
    // Apply speed limits
    v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);
    
    // Zero-crossing protection (prevent oscillation near standstill)
    if ((s.v_mps > 0.0 && v_next < 0.0) || (s.v_mps < 0.0 && v_next > 0.0)) {
        if (std::abs(v_next) < p_.v_stop_eps) {
            v_next = 0.0;  // Snap to zero
        }
    }
    
    // Update state
    s.v_mps = v_next;
    s.a_long_mps2 = a_long;
    
    // ========================================================================
    // BICYCLE KINEMATICS (PDF Eq. 114-117)
    // ========================================================================
    
    // Yaw rate from bicycle model: ψ̇ = v/L·tan(δ)
    double yaw_rate = 0.0;
    if (std::abs(s.v_mps) > p_.v_stop_eps) {
        yaw_rate = s.v_mps / p_.wheelbase_m * std::tan(s.steer_virtual_rad);
    }
    
    // Integrate yaw angle: ψ[k+1] = ψ[k] + ψ̇·Δt
    s.yaw_rad += yaw_rate * dt;
    
    // Normalize yaw to [-π, π]
    while (s.yaw_rad > M_PI) s.yaw_rad -= 2.0 * M_PI;
    while (s.yaw_rad < -M_PI) s.yaw_rad += 2.0 * M_PI;
    
    // Integrate position: x[k+1] = x[k] + v·cos(ψ)·Δt
    s.x_m += s.v_mps * std::cos(s.yaw_rad) * dt;
    s.y_m += s.v_mps * std::sin(s.yaw_rad) * dt;
}

void VehicleSubsystem::set_params(const VehicleParams& params) {
    p_ = params;
    LOG_INFO("[VehicleSubsystem] Parameters updated: mass=%.0f kg, drag_c=%.2f",
             params.mass_kg, params.drag_c);
}

void VehicleSubsystem::compute_resistive_forces(
    double v_mps,
    double& Fdrag_out,
    double& Froll_out
) const {
    // Aerodynamic drag: Fdrag = drag_c · v · |v|
    // Sign of drag always opposes motion
    Fdrag_out = p_.drag_c * v_mps * std::abs(v_mps);
    
    // Rolling resistance: Froll = roll_c · sgn(v)
    // Constant magnitude, opposes motion direction
    Froll_out = p_.roll_c * static_cast<double>(sgn(v_mps));
}

} // namespace plant