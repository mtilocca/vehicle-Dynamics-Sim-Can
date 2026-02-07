// src/plant/wheel_subsystem/wheel_subsystem.cpp
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_kinematics.hpp"
#include "plant/wheel_subsystem/load_transfer_model.hpp"
#include "utils/logging.hpp"
#include <cmath>

namespace plant {

WheelSubsystem::WheelSubsystem(const WheelSubsystemParams& params)
    : p_(params),
      wd_fl_(params.wheel),
      wd_fr_(params.wheel),
      wd_rl_(params.wheel),
      wd_rr_(params.wheel),
      tyre_model_(params.tyre_params) {}

void WheelSubsystem::initialize(PlantState& s) {
    LOG_INFO("[WheelSubsystem] Initializing: Iw=%.0f kg·m², R=%.2f m, mode=%s",
             p_.wheel.inertia_kgm2, p_.wheel.radius_m,
             p_.dynamic_mode_enabled ? "DYNAMIC" : "KINEMATIC");

    // Push surface friction into tyre_params (since TyreDugoff has no set_surface_friction())
    p_.tyre_params.mu_peak = p_.mu_peak;
    p_.tyre_params.mu_slide = p_.mu_slide;
    tyre_model_.set_params(p_.tyre_params);

    // Initialize wheel speeds from vehicle speed
    wd_fl_.init_from_velocity(s.v_mps);
    wd_fr_.init_from_velocity(s.v_mps);
    wd_rl_.init_from_velocity(s.v_mps);
    wd_rr_.init_from_velocity(s.v_mps);

    s.omega_fl_radps = wd_fl_.omega_radps();
    s.omega_fr_radps = wd_fr_.omega_radps();
    s.omega_rl_radps = wd_rl_.omega_radps();
    s.omega_rr_radps = wd_rr_.omega_radps();

    // Compute initial static normal loads (zero acceleration)
    LoadTransferModel::compute(
        p_.mass_kg, 0.0, 0.0,
        p_.wheelbase_m, p_.track_m, p_.cg_height_m, p_.cg_to_front_m,
        s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr
    );

    s.dynamic_model_enabled = p_.dynamic_mode_enabled;
}

void WheelSubsystem::reset(PlantState& s) {
    wd_fl_.set_omega_radps(0.0);
    wd_fr_.set_omega_radps(0.0);
    wd_rl_.set_omega_radps(0.0);
    wd_rr_.set_omega_radps(0.0);

    s.omega_fl_radps = 0.0;
    s.omega_fr_radps = 0.0;
    s.omega_rl_radps = 0.0;
    s.omega_rr_radps = 0.0;

    // Zero forces/slips
    s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
    s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
    s.sigma_x_fl = s.sigma_x_fr = s.sigma_x_rl = s.sigma_x_rr = 0.0;
    s.sigma_y_fl = s.sigma_y_fr = s.sigma_y_rl = s.sigma_y_rr = 0.0;
    s.lambda_fl = s.lambda_fr = s.lambda_rl = s.lambda_rr = 1.0;
}

void WheelSubsystem::set_params(const WheelSubsystemParams& params) {
    p_ = params;

    // Keep tyre params consistent
    p_.tyre_params.mu_peak = p_.mu_peak;
    p_.tyre_params.mu_slide = p_.mu_slide;
    tyre_model_.set_params(p_.tyre_params);

    // Update wheel dynamics params
    wd_fl_.params() = p_.wheel;
    wd_fr_.params() = p_.wheel;
    wd_rl_.params() = p_.wheel;
    wd_rr_.params() = p_.wheel;

    LOG_INFO("[WheelSubsystem] Params updated: R=%.2f, mu_peak=%.2f, mode=%s",
             p_.wheel.radius_m, p_.tyre_params.mu_peak,
             p_.dynamic_mode_enabled ? "DYNAMIC" : "KINEMATIC");
}

void WheelSubsystem::step(PlantState& s, const sim::ActuatorCmd& /*cmd*/, double dt) {
    if (dt <= 0.0) return;

    s.dynamic_model_enabled = p_.dynamic_mode_enabled;

    // If dynamic mode disabled, just kinematically follow vehicle speed
    if (!p_.dynamic_mode_enabled) {
        wd_fl_.init_from_velocity(s.v_mps);
        wd_fr_.init_from_velocity(s.v_mps);
        wd_rl_.init_from_velocity(s.v_mps);
        wd_rr_.init_from_velocity(s.v_mps);

        s.omega_fl_radps = wd_fl_.omega_radps();
        s.omega_fr_radps = wd_fr_.omega_radps();
        s.omega_rl_radps = wd_rl_.omega_radps();
        s.omega_rr_radps = wd_rr_.omega_radps();

        // Zero forces
        s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
        s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
        return;
    }

    // ---------------------------------------------------------------------
    // Robust reverse-capable handling:
    // - Brake torque must oppose wheel rotation, not “vehicle direction”.
    // - When omega ~ 0, oppose the *requested* longitudinal direction from Vx.
    // ---------------------------------------------------------------------
    auto brake_opposing_torque = [&](double tau_brake_nm, double omega_radps, double v_ref_mps) {
        if (tau_brake_nm <= 0.0) return 0.0;

        const double eps = p_.wheel.omega_eps_radps;
        int dir = 0;
        if (std::abs(omega_radps) > eps) {
            dir = sgn(omega_radps);
        } else if (std::abs(v_ref_mps) > p_.wheel.v_eps_mps) {
            dir = sgn(v_ref_mps);
        } else {
            dir = 0;
        }

        // Oppose direction => same sign as dir (because we subtract tau_brake upstream in WheelDynamics)
        // We want tau_brake_term = dir * |tau_brake|
        return static_cast<double>(dir) * tau_brake_nm;
    };

    // -------------------------------------------------------------------------
    // WHEEL KINEMATICS: Compute per-wheel velocities from vehicle body state
    // -------------------------------------------------------------------------
    // Wheel positions relative to CG (vehicle frame: +x forward, +y left)
    const double cg_to_rear = p_.wheelbase_m - p_.cg_to_front_m;
    const double half_track = p_.track_m / 2.0;

    // Front wheels: forward of CG, left/right of centerline
    const double x_f = p_.cg_to_front_m;
    const double y_fl = +half_track;
    const double y_fr = -half_track;

    // Rear wheels: behind CG, left/right of centerline
    const double x_r = -cg_to_rear;
    const double y_rl = +half_track;
    const double y_rr = -half_track;

    // Compute per-wheel velocities in wheel frame
    double vx_fl, vy_fl, vx_fr, vy_fr, vx_rl, vy_rl, vx_rr, vy_rr;

    WheelKinematics::compute(s.v_mps, s.vy_mps, s.yaw_rate_radps,
                             x_f, y_fl, s.delta_fl_rad, vx_fl, vy_fl);
    WheelKinematics::compute(s.v_mps, s.vy_mps, s.yaw_rate_radps,
                             x_f, y_fr, s.delta_fr_rad, vx_fr, vy_fr);
    WheelKinematics::compute(s.v_mps, s.vy_mps, s.yaw_rate_radps,
                             x_r, y_rl, 0.0, vx_rl, vy_rl);  // Rear: no steer
    WheelKinematics::compute(s.v_mps, s.vy_mps, s.yaw_rate_radps,
                             x_r, y_rr, 0.0, vx_rr, vy_rr);  // Rear: no steer

    // Compute slip angles
    s.alpha_fl = WheelKinematics::slip_angle(vx_fl, vy_fl, p_.tyre_params.v_min);
    s.alpha_fr = WheelKinematics::slip_angle(vx_fr, vy_fr, p_.tyre_params.v_min);
    s.alpha_rl = WheelKinematics::slip_angle(vx_rl, vy_rl, p_.tyre_params.v_min);
    s.alpha_rr = WheelKinematics::slip_angle(vx_rr, vy_rr, p_.tyre_params.v_min);

    // -------------------------------------------------------------------------
    // NORMAL LOADS: Compute Fz with load transfer
    // -------------------------------------------------------------------------
    LoadTransferModel::compute(
        p_.mass_kg, s.a_long_mps2, s.a_lat_mps2,
        p_.wheelbase_m, p_.track_m, p_.cg_height_m, p_.cg_to_front_m,
        s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr
    );

    // --- Per-wheel: slip -> tyre forces (Dugoff) -> wheel omega integration ---
    auto process_wheel = [&](WheelDynamics& wd,
                             double tau_drive_nm,
                             double tau_brake_nm,
                             double Vx_mps,
                             double Vy_mps,
                             double Fz_n,
                             double delta_rad,
                             double& out_Fx,
                             double& out_Fy,
                             double& out_sigma_x,
                             double& out_sigma_y,
                             double& out_lambda)
    {
        // Compute tyre forces using Dugoff API (5 args, returns TyreForces)
        // Forces are in WHEEL frame
        const TyreForces tf = tyre_model_.compute_forces(
            wd.omega_radps(),
            p_.wheel.radius_m,
            Vx_mps,
            Vy_mps,
            Fz_n
        );

        // CRITICAL: Transform forces from wheel frame to vehicle frame
        // Wheel frame is rotated by +delta (counterclockwise) from vehicle frame
        // To rotate vector from wheel frame to vehicle frame: rotate by -delta (clockwise)
        // Rotation by -delta:
        //   Fx_vehicle = Fx_wheel * cos(delta) + Fy_wheel * sin(delta)
        //   Fy_vehicle = -Fx_wheel * sin(delta) + Fy_wheel * cos(delta)
        const double cos_delta = std::cos(delta_rad);
        const double sin_delta = std::sin(delta_rad);

        out_Fx = tf.Fx * cos_delta + tf.Fy * sin_delta;   // Fixed: + not -
        out_Fy = -tf.Fx * sin_delta + tf.Fy * cos_delta;  // Fixed: - not +
        out_sigma_x = tf.sigma_x;
        out_sigma_y = tf.sigma_y;
        out_lambda = tf.lambda;

        // Make brake oppose current wheel rotation (or Vx if omega ~ 0)
        const double tau_brake_term = brake_opposing_torque(tau_brake_nm, wd.omega_radps(), Vx_mps);

        // Integrate wheel omega with the WHEEL-FRAME Fx (not vehicle-frame!)
        wd.step(tau_drive_nm, tau_brake_term, tf.Fx, dt);
    };

    // -------------------------------------------------------------------------
    // TIRE FORCES: Process each wheel (slip → Dugoff → forces → ω integration)
    // Forces are transformed from wheel frame to vehicle frame
    // -------------------------------------------------------------------------
    process_wheel(wd_fl_, s.tau_drive_fl_nm, s.tau_brake_fl_nm, vx_fl, vy_fl, s.Fz_fl, s.delta_fl_rad,
                  s.Fx_fl, s.Fy_fl, s.sigma_x_fl, s.sigma_y_fl, s.lambda_fl);

    process_wheel(wd_fr_, s.tau_drive_fr_nm, s.tau_brake_fr_nm, vx_fr, vy_fr, s.Fz_fr, s.delta_fr_rad,
                  s.Fx_fr, s.Fy_fr, s.sigma_x_fr, s.sigma_y_fr, s.lambda_fr);

    process_wheel(wd_rl_, s.tau_drive_rl_nm, s.tau_brake_rl_nm, vx_rl, vy_rl, s.Fz_rl, 0.0,  // Rear: no steer
                  s.Fx_rl, s.Fy_rl, s.sigma_x_rl, s.sigma_y_rl, s.lambda_rl);

    process_wheel(wd_rr_, s.tau_drive_rr_nm, s.tau_brake_rr_nm, vx_rr, vy_rr, s.Fz_rr, 0.0,  // Rear: no steer
                  s.Fx_rr, s.Fy_rr, s.sigma_x_rr, s.sigma_y_rr, s.lambda_rr);

    // -------------------------------------------------------------------------
    // PERIODIC LOGGING: Wheel forces and dynamics (every 50 steps)
    // -------------------------------------------------------------------------
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {
        LOG_INFO("========== WHEEL DYNAMICS (step %d) ==========", log_counter);
        LOG_INFO("Vehicle state: vx=%.2f m/s, vy=%.2f m/s, yaw_rate=%.3f rad/s",
                 s.v_mps, s.vy_mps, s.yaw_rate_radps);

        LOG_INFO("Drive torques:  FL=%.1f  FR=%.1f  RL=%.1f  RR=%.1f Nm",
                 s.tau_drive_fl_nm, s.tau_drive_fr_nm, s.tau_drive_rl_nm, s.tau_drive_rr_nm);
        LOG_INFO("Brake torques:  FL=%.1f  FR=%.1f  RL=%.1f  RR=%.1f Nm",
                 s.tau_brake_fl_nm, s.tau_brake_fr_nm, s.tau_brake_rl_nm, s.tau_brake_rr_nm);

        LOG_INFO("Normal loads:   FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                 s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr);
        LOG_INFO("Long forces:    FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                 s.Fx_fl, s.Fx_fr, s.Fx_rl, s.Fx_rr);
        LOG_INFO("Lat forces:     FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                 s.Fy_fl, s.Fy_fr, s.Fy_rl, s.Fy_rr);

        LOG_INFO("Slip ratios σx: FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f",
                 s.sigma_x_fl, s.sigma_x_fr, s.sigma_x_rl, s.sigma_x_rr);
        LOG_INFO("Slip angles α:  FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f rad (%.1f° %.1f° %.1f° %.1f°)",
                 s.alpha_fl, s.alpha_fr, s.alpha_rl, s.alpha_rr,
                 s.alpha_fl * 180.0 / M_PI, s.alpha_fr * 180.0 / M_PI,
                 s.alpha_rl * 180.0 / M_PI, s.alpha_rr * 180.0 / M_PI);

        LOG_INFO("Wheel omegas:   FL=%.2f  FR=%.2f  RL=%.2f  RR=%.2f rad/s",
                 s.omega_fl_radps, s.omega_fr_radps, s.omega_rl_radps, s.omega_rr_radps);
        LOG_INFO("Friction λ:     FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f",
                 s.lambda_fl, s.lambda_fr, s.lambda_rl, s.lambda_rr);
        LOG_INFO("============================================");
    }

    // Export wheel speeds to PlantState
    s.omega_fl_radps = wd_fl_.omega_radps();
    s.omega_fr_radps = wd_fr_.omega_radps();
    s.omega_rl_radps = wd_rl_.omega_radps();
    s.omega_rr_radps = wd_rr_.omega_radps();

    // Optional: convert to rps too (your PlantState still has wheel_*_rps fields)
    s.wheel_fl_rps = s.omega_fl_radps / (2.0 * M_PI);
    s.wheel_fr_rps = s.omega_fr_radps / (2.0 * M_PI);
    s.wheel_rl_rps = s.omega_rl_radps / (2.0 * M_PI);
    s.wheel_rr_rps = s.omega_rr_radps / (2.0 * M_PI);
}

} // namespace plant
