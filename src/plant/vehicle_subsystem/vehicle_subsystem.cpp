// src/plant/vehicle_subsystem/vehicle_subsystem.cpp
// DEBUG VERSION: Added detailed logging to track acceleration sign

#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

VehicleSubsystem::VehicleSubsystem(const VehicleParams& params)
    : p_(params)
{
}

void VehicleSubsystem::initialize(PlantState& s) {
    LOG_INFO("[VehicleSubsystem] Initializing 3-DOF dynamics:");
    LOG_INFO("  - mass=%.0f kg, Iz=%.2e kg·m²", p_.mass_kg, p_.yaw_inertia_kgm2);
    LOG_INFO("  - wheelbase=%.2f m, track=%.2f m", p_.wheelbase_m, p_.track_m);
    LOG_INFO("  - drag_c=%.2f, roll_c=%.1f N", p_.drag_c, p_.roll_c);

    // Initialize vehicle state to rest (3-DOF: vx, vy, yaw_rate)
    s.x_m = 0.0;
    s.y_m = 0.0;
    s.yaw_rad = 0.0;
    s.v_mps = 0.0;
    s.vy_mps = 0.0;
    s.yaw_rate_radps = 0.0;
    s.a_long_mps2 = 0.0;
    s.a_lat_mps2 = 0.0;

    dir_latch_ = 0;
}

void VehicleSubsystem::reset(PlantState& s) {
    LOG_INFO("[VehicleSubsystem] Resetting to origin (3-DOF)");

    s.x_m = 0.0;
    s.y_m = 0.0;
    s.yaw_rad = 0.0;
    s.v_mps = 0.0;
    s.vy_mps = 0.0;
    s.yaw_rate_radps = 0.0;
    s.a_long_mps2 = 0.0;
    s.a_lat_mps2 = 0.0;

    dir_latch_ = 0;
}

void VehicleSubsystem::update_direction_latch(const PlantState& s, const sim::ActuatorCmd& cmd) {
    const double v = s.v_mps;
    const double v_abs = std::abs(v);

    // If moving meaningfully, direction is just the velocity sign
    if (v_abs > p_.v_stop_eps) {
        dir_latch_ = sgn(v);
        return;
    }

    // Near standstill: decide intent from requested driver torque (deadbanded)
    const double tq_req = cmd.drive_torque_cmd_nm;
    if (std::abs(tq_req) >= p_.torque_dir_deadband_nm) {
        int req_dir = sgn(tq_req);

        // If reverse not allowed, force forward intent
        if (!p_.allow_reverse && req_dir < 0) req_dir = +1;

        dir_latch_ = req_dir;
        return;
    }

    // No clear intent near standstill → neutral
    dir_latch_ = 0;
}

void VehicleSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    if (dt <= 0.0) return;

    // 1) Update direction intent (for reverse support + stability at 0)
    update_direction_latch(s, cmd);

    // ========================================================================
    // LONGITUDINAL DYNAMICS
    // ========================================================================

    const double Fx_total = s.Fx_fl + s.Fx_fr + s.Fx_rl + s.Fx_rr;

    // Choose a direction reference for resistive forces:
    // - If moving: oppose actual motion sign
    // - If near standstill: oppose intended direction (prevents sign noise flip)
    int dir_ref = 0;
    if (std::abs(s.v_mps) > p_.v_stop_eps) dir_ref = sgn(s.v_mps);
    else dir_ref = dir_latch_;

    double F_drag = 0.0;
    double F_roll = 0.0;
    compute_resistive_forces(s.v_mps, dir_ref, F_drag, F_roll);

    const double F_net = Fx_total - F_drag - F_roll;
    const double a_long = F_net / p_.mass_kg;

    // DEBUG LOGGING
    static int debug_ctr = 0;
    if (debug_ctr++ % 50 == 0) {
        LOG_INFO("========== ACCELERATION DEBUG ==========");
        LOG_INFO("v_mps = %.3f m/s  (dir_latch=%d dir_ref=%d)", s.v_mps, dir_latch_, dir_ref);
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

    double v_next = s.v_mps + a_long * dt;
    v_next = clamp(v_next, -p_.v_max_mps, +p_.v_max_mps);

    // ========================================================================
    // ZERO-CROSSING + INTENT GATING
    // ========================================================================
    const bool crossing =
        ((s.v_mps > 0.0 && v_next < 0.0) || (s.v_mps < 0.0 && v_next > 0.0));

    if (crossing) {
        // Allow crossing ONLY if user is clearly requesting the opposite direction
        const int req_dir = (std::abs(cmd.drive_torque_cmd_nm) >= p_.torque_dir_deadband_nm)
                                ? sgn(cmd.drive_torque_cmd_nm)
                                : 0;

        const bool allow_cross =
            p_.allow_reverse &&
            (req_dir != 0) &&
            (req_dir != sgn(s.v_mps));  // explicitly asking to reverse direction

        if (!allow_cross) {
            // No clear reversal request → snap to 0 at the boundary
            v_next = 0.0;
        } else {
            // Still do a small snap zone to prevent chatter
            if (std::abs(v_next) < p_.v_stop_eps) v_next = 0.0;
        }
    } else {
        // Normal near-standstill snap
        if (std::abs(v_next) < p_.v_stop_eps && dir_latch_ == 0) {
            v_next = 0.0;
        }
    }

    // If reverse is disabled, clamp negative velocity to zero
    if (!p_.allow_reverse && v_next < 0.0) {
        v_next = 0.0;
    }

    // Update longitudinal state
    s.v_mps = v_next;
    s.a_long_mps2 = a_long;

    // ========================================================================
    // LATERAL AND YAW DYNAMICS (3-DOF)
    // ========================================================================

    // Sum lateral forces from all wheels
    const double Fy_total = s.Fy_fl + s.Fy_fr + s.Fy_rl + s.Fy_rr;

    // Compute yaw moment about CG
    // Mz = Σ(Fx_i * y_i - Fy_i * x_i) where (x_i, y_i) is wheel position relative to CG
    const double cg_to_rear = p_.wheelbase_m - p_.cg_to_front_m;
    const double half_track = p_.track_m / 2.0;

    const double x_f = p_.cg_to_front_m;
    const double x_r = -cg_to_rear;

    const double Mz_fl = s.Fx_fl * (+half_track) - s.Fy_fl * x_f;
    const double Mz_fr = s.Fx_fr * (-half_track) - s.Fy_fr * x_f;
    const double Mz_rl = s.Fx_rl * (+half_track) - s.Fy_rl * x_r;
    const double Mz_rr = s.Fx_rr * (-half_track) - s.Fy_rr * x_r;

    const double Mz_total = Mz_fl + Mz_fr + Mz_rl + Mz_rr;

    // Lateral acceleration (accounting for centripetal coupling)
    // ay = Fy / m - yaw_rate * vx
    const double ay_from_forces = Fy_total / p_.mass_kg;
    const double ay = ay_from_forces - s.yaw_rate_radps * s.v_mps;

    // Yaw acceleration
    const double yaw_ddot = Mz_total / p_.yaw_inertia_kgm2;

    // Integrate lateral velocity
    double vy_next = s.vy_mps + ay_from_forces * dt;
    s.vy_mps = vy_next;
    s.a_lat_mps2 = ay;  // Total lateral acceleration (with centripetal term)

    // Integrate yaw rate
    s.yaw_rate_radps += yaw_ddot * dt;
    s.yaw_rad += s.yaw_rate_radps * dt;

    // Normalize yaw angle to [-π, +π]
    while (s.yaw_rad > M_PI) s.yaw_rad -= 2.0 * M_PI;
    while (s.yaw_rad < -M_PI) s.yaw_rad += 2.0 * M_PI;

    // ========================================================================
    // POSITION INTEGRATION (global frame)
    // ========================================================================
    const double cos_yaw = std::cos(s.yaw_rad);
    const double sin_yaw = std::sin(s.yaw_rad);

    // Transform body velocities (vx, vy) to global frame
    const double vx_global = s.v_mps * cos_yaw - s.vy_mps * sin_yaw;
    const double vy_global = s.v_mps * sin_yaw + s.vy_mps * cos_yaw;

    s.x_m += vx_global * dt;
    s.y_m += vy_global * dt;
}

void VehicleSubsystem::set_params(const VehicleParams& params) {
    p_ = params;
    LOG_INFO("[VehicleSubsystem] Parameters updated: mass=%.0f kg, drag_c=%.2f, allow_reverse=%d",
             params.mass_kg, params.drag_c, (int)params.allow_reverse);
}

void VehicleSubsystem::compute_resistive_forces(
    double v_mps,
    int dir_ref,
    double& Fdrag_out,
    double& Froll_out
) const {
    // Quadratic drag: opposes direction of motion.
    // Using v*|v| gives the correct sign automatically (positive when v>0).
    Fdrag_out = p_.drag_c * v_mps * std::abs(v_mps);

    // Rolling resistance: constant magnitude opposing direction.
    // Near standstill v≈0, use dir_ref (intent) to avoid sign noise.
    if (dir_ref == 0) {
        Froll_out = 0.0;
    } else {
        Froll_out = p_.roll_c * static_cast<double>(dir_ref);
    }
}

} // namespace plant
