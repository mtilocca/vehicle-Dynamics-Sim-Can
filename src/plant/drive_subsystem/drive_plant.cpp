// src/plant/drive_subsystem/drive_plant.cpp
//
// DrivePlant - Simplified powertrain + linear bicycle tire model
//
// No Dugoff, no wheel rotational dynamics, no regen/battery.
// Traction force limited by gear ratio, efficiency, power, and surface friction.
// Lateral forces from linear cornering stiffness (3-DOF stays intact).

#include "drive_plant.hpp"
#include "sim/actuator_cmd.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

static constexpr double G = 9.81;  // [m/s²]
static const double TWO_PI = 2.0 * M_PI;

void DrivePlant::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt_s)
{
    if (dt_s <= 0.0) return;

    const bool enabled = cmd.system_enable;
    const double vx    = s.v_mps;
    const double vx_abs = std::abs(vx);
    const double vy    = s.vy_mps;
    const double psi_dot = s.yaw_rate_radps;

    const double lf = p_.cg_to_front_m;
    const double lr = p_.wheelbase_m - lf;
    const double L  = p_.wheelbase_m;

    // ========================================================================
    // STEP 1: Motor torque → traction force (RWD rear axle only)
    // ========================================================================
    double motor_tq_cmd = enabled
        ? clamp(cmd.drive_torque_cmd_nm, -p_.motor_torque_max_nm, +p_.motor_torque_max_nm)
        : 0.0;

    // Wheel angular speed estimate (no-slip)
    const double omega_wheel = std::max(vx_abs, p_.v_stop_eps) / p_.wheel_radius_m;

    // Traction force at wheel rim
    double F_traction = motor_tq_cmd * p_.gear_ratio * p_.drivetrain_eff / p_.wheel_radius_m;

    // Power limit: F_max = P_max / v_wheel_rim
    const double v_rim = omega_wheel * p_.wheel_radius_m;
    const double F_power_max = p_.motor_power_max_w / std::max(v_rim, 0.1);
    F_traction = clamp(F_traction, -F_power_max, +F_power_max);

    // Friction limit on rear axle (static weight × mu)
    const double Fz_rear_total = p_.mass_kg * G * (lf / L);
    const double F_friction_max = p_.mu_surface * Fz_rear_total;
    F_traction = clamp(F_traction, -F_friction_max, +F_friction_max);

    // ========================================================================
    // STEP 2: Brake force
    // ========================================================================
    const double brake_pct    = clamp(cmd.brake_cmd_pct, 0.0, 100.0);
    const double F_brake_total = (brake_pct / 100.0) * p_.brake_torque_max_nm / p_.wheel_radius_m;
    const double F_brake_front = F_brake_total * p_.brake_bias_front;
    const double F_brake_rear  = F_brake_total * (1.0 - p_.brake_bias_front);

    // ========================================================================
    // STEP 3: Static normal loads (no dynamic load transfer for simplicity)
    // ========================================================================
    const double Fz_front_axle = p_.mass_kg * G * (lr / L);
    const double Fz_rear_axle  = Fz_rear_total;

    s.Fz_fl = Fz_front_axle * 0.5;
    s.Fz_fr = Fz_front_axle * 0.5;
    s.Fz_rl = Fz_rear_axle  * 0.5;
    s.Fz_rr = Fz_rear_axle  * 0.5;

    // ========================================================================
    // STEP 4: Per-wheel longitudinal forces
    // RWD: rear wheels get traction, all wheels get brake
    // Brake on front wheels: forward direction means braking is negative Fx
    // ========================================================================
    const double v_dir = (vx >= 0.0) ? 1.0 : -1.0;
    s.Fx_rl = (F_traction - F_brake_rear  * v_dir) * 0.5;
    s.Fx_rr = (F_traction - F_brake_rear  * v_dir) * 0.5;
    s.Fx_fl = -F_brake_front * v_dir * 0.5;
    s.Fx_fr = -F_brake_front * v_dir * 0.5;

    // ========================================================================
    // STEP 5: Slip angles (linear bicycle model)
    // alpha = delta - atan2(vy_wheel, |vx|)  (positive alpha → force opposes it)
    // Guard against near-zero vx to avoid division issues.
    // ========================================================================
    const double vx_safe = std::max(vx_abs, p_.v_stop_eps);

    const double delta_f = s.steer_virtual_rad;  // front axle average steer angle

    // Front slip angle: α_f = δ - (vy + lf·ψ̇)/vx
    const double alpha_f = delta_f - (vy + lf * psi_dot) / vx_safe;

    // Rear slip angle: α_r = -(vy - lr·ψ̇)/vx
    const double alpha_r = -(vy - lr * psi_dot) / vx_safe;

    // ========================================================================
    // STEP 6: Lateral forces (linear cornering: Fy opposes slip, hence negative)
    // Full axle stiffness split 50/50 per wheel.
    // Friction-circle saturation: |Fy| ≤ √((μ·Fz)² − Fx²) prevents divergence.
    // ========================================================================
    const double Fy_front_raw = -(p_.Cy_front_Npm / 2.0) * alpha_f;
    const double Fy_rear_raw  = -(p_.Cy_rear_Npm  / 2.0) * alpha_r;

    auto fy_cap = [&](double fy_raw, double Fz, double Fx) -> double {
        const double mu_fz  = p_.mu_surface * Fz;
        const double fx_sq  = Fx * Fx;
        const double avail  = std::sqrt(std::max(0.0, mu_fz * mu_fz - fx_sq));
        return clamp(fy_raw, -avail, avail);
    };

    s.Fy_fl = fy_cap(Fy_front_raw, s.Fz_fl, s.Fx_fl);
    s.Fy_fr = fy_cap(Fy_front_raw, s.Fz_fr, s.Fx_fr);
    s.Fy_rl = fy_cap(Fy_rear_raw,  s.Fz_rl, s.Fx_rl);
    s.Fy_rr = fy_cap(Fy_rear_raw,  s.Fz_rr, s.Fx_rr);

    // ========================================================================
    // STEP 7: Wheel speeds (no-slip derivation)
    // ========================================================================
    const double rps = vx / (p_.wheel_radius_m * TWO_PI);
    s.wheel_fl_rps = rps;
    s.wheel_fr_rps = rps;
    s.wheel_rl_rps = rps;
    s.wheel_rr_rps = rps;

    // ========================================================================
    // STEP 8: Motor / brake state (for logging and CAN)
    // ========================================================================
    s.motor_torque_nm = motor_tq_cmd;
    s.brake_force_kN  = F_brake_total / 1000.0;

    static int log_ctr = 0;
    if (++log_ctr % 200 == 0) {
        LOG_DEBUG("[DrivePlant] vx=%.2f m/s, F_traction=%.0f N, F_brake=%.0f N, "
                  "alpha_f=%.3f rad, alpha_r=%.3f rad, Fy_f=%.0f N, Fy_r=%.0f N",
                  vx, F_traction, F_brake_total,
                  alpha_f, alpha_r, (s.Fy_fl + s.Fy_fr), (s.Fy_rl + s.Fy_rr));
    }
}

} // namespace plant
