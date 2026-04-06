// src/plant/wheel_subsystem/wheel_subsystem.cpp
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/wheel_subsystem/wheel_kinematics.hpp"
#include "plant/wheel_subsystem/load_transfer_model.hpp"
#include "utils/logging.hpp"
#ifdef __ZEPHYR__
LOG_MODULE_DECLARE(xcmg_sim, LOG_LEVEL_INF);
#endif
#include <cmath>
#ifdef __ZEPHYR__
#  include <zephyr/kernel.h>
#else
#  include <chrono>
#endif

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

    // KINEMATIC FALLBACK at low speed (< 3.6 kph = 1 m/s):
    // Below this threshold Dugoff slip ratios explode because wheel omega
    // hasn't matched vx yet → ±200 kN Fx oscillation on startup/stop.
    // Omega is synced to vx; Fx is computed as F = τ/R (no slip calculation)
    // so the vehicle can still accelerate and brake in this regime.
    // Fy stays zero to prevent yaw runaway at low speed.
    // Works for forward and reverse: std::abs() + sign of drive torque.
    const double v_kin_thresh_mps = 1.0;  // 3.6 kph
    if (std::abs(s.v_mps) < v_kin_thresh_mps) {
        const double r = p_.wheel.radius_m;

        // Sync wheel omega to vehicle speed
        const double wheel_omega = s.v_mps / r;
        wd_fl_.set_omega_radps(wheel_omega);
        wd_fr_.set_omega_radps(wheel_omega);
        wd_rl_.set_omega_radps(wheel_omega);
        wd_rr_.set_omega_radps(wheel_omega);
        s.omega_fl_radps = wheel_omega;
        s.omega_fr_radps = wheel_omega;
        s.omega_rl_radps = wheel_omega;
        s.omega_rr_radps = wheel_omega;

        // Kinematic Fx = τ/R; brake opposes direction of motion (or intent)
        const double tq_net = s.tau_drive_rl_nm + s.tau_drive_rr_nm
                            + s.tau_drive_fl_nm + s.tau_drive_fr_nm;
        const int brake_dir = (std::abs(s.v_mps) > p_.wheel.v_eps_mps)
                              ? sgn(s.v_mps)
                              : sgn(tq_net);

        s.Fx_fl = (s.tau_drive_fl_nm - s.tau_brake_fl_nm * brake_dir) / r;
        s.Fx_fr = (s.tau_drive_fr_nm - s.tau_brake_fr_nm * brake_dir) / r;
        s.Fx_rl = (s.tau_drive_rl_nm - s.tau_brake_rl_nm * brake_dir) / r;
        s.Fx_rr = (s.tau_drive_rr_nm - s.tau_brake_rr_nm * brake_dir) / r;

        // No lateral forces at low speed
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
    // CRITICAL: Use centripetal (inertial) lateral acceleration for load transfer,
    // NOT the body-frame a_lat (which approaches 0 in steady cornering).
    // The lateral tire forces create the centripetal force that rolls the body
    // and transfers load. Using previous step's Fy values (explicit Euler).
    // PDF Eq. 43-44: ΔFz_lat = m * ay_centripetal * hCG / w
    const double ay_for_load_transfer =
        (s.Fy_fl + s.Fy_fr + s.Fy_rl + s.Fy_rr) / p_.mass_kg;

    LoadTransferModel::compute(
        p_.mass_kg, s.a_long_mps2, ay_for_load_transfer,
        p_.wheelbase_m, p_.track_m, p_.cg_height_m, p_.cg_to_front_m,
        s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr
    );

    // Convert gear_position to direction: Forward=+1, Reverse=-1, Neutral=0
    int gear_dir = 0;
    switch (s.gear_position) {
        case sim::GearPosition::FORWARD: gear_dir = 1; break;
        case sim::GearPosition::REVERSE: gear_dir = -1; break;
        case sim::GearPosition::NEUTRAL: gear_dir = 0; break;
        default: gear_dir = 0; break;
    }

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
        // Compute tyre forces using Dugoff API (6 args including gear_dir)
        // Forces are in WHEEL frame
        const TyreForces tf = tyre_model_.compute_forces(
            wd.omega_radps(),
            p_.wheel.radius_m,
            Vx_mps,
            Vy_mps,
            Fz_n,
            gear_dir
        );

        // CRITICAL: Transform forces from wheel frame to vehicle frame
        // Wheel frame is rotated by +delta (CCW) from vehicle frame
        // Rotation matrix R(δ) to express wheel-frame vector in vehicle-frame:
        //   [Fx_veh]   [cos(δ)  -sin(δ)]   [Fx_wheel]
        //   [Fy_veh] = [sin(δ)   cos(δ)] * [Fy_wheel]
        const double cos_delta = std::cos(delta_rad);
        const double sin_delta = std::sin(delta_rad);

        out_Fx = tf.Fx * cos_delta - tf.Fy * sin_delta;   // CRITICAL FIX: MINUS sign
        out_Fy = tf.Fx * sin_delta + tf.Fy * cos_delta;   // CRITICAL FIX: PLUS sign
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
    // PERIODIC LOGGING: Wheel forces and dynamics (once every 5 s wall time)
    // -------------------------------------------------------------------------
    {
#ifdef __ZEPHYR__
        static uint32_t last_log_ms = 0;
        uint32_t now_ms = k_uptime_get_32();
        bool do_log = (now_ms - last_log_ms) >= 5000u;
        if (do_log) last_log_ms = now_ms;
#else
        using clk = std::chrono::steady_clock;
        static auto last_log = clk::now();
        auto now = clk::now();
        bool do_log = std::chrono::duration<double>(now - last_log).count() >= 5.0;
        if (do_log) last_log = now;
#endif
        if (do_log) {
            LOG_DEBUG("========== WHEEL DYNAMICS ==========");
            LOG_DEBUG("Vehicle state: vx=%.2f m/s, vy=%.2f m/s, yaw_rate=%.3f rad/s",
                     s.v_mps, s.vy_mps, s.yaw_rate_radps);
            LOG_DEBUG("Drive torques:  FL=%.1f  FR=%.1f  RL=%.1f  RR=%.1f Nm",
                     s.tau_drive_fl_nm, s.tau_drive_fr_nm, s.tau_drive_rl_nm, s.tau_drive_rr_nm);
            LOG_DEBUG("Brake torques:  FL=%.1f  FR=%.1f  RL=%.1f  RR=%.1f Nm",
                     s.tau_brake_fl_nm, s.tau_brake_fr_nm, s.tau_brake_rl_nm, s.tau_brake_rr_nm);
            LOG_DEBUG("Normal loads:   FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                     s.Fz_fl, s.Fz_fr, s.Fz_rl, s.Fz_rr);
            LOG_DEBUG("Long forces:    FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                     s.Fx_fl, s.Fx_fr, s.Fx_rl, s.Fx_rr);
            LOG_DEBUG("Lat forces:     FL=%.0f  FR=%.0f  RL=%.0f  RR=%.0f N",
                     s.Fy_fl, s.Fy_fr, s.Fy_rl, s.Fy_rr);
            LOG_DEBUG("Slip ratios σx: FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f",
                     s.sigma_x_fl, s.sigma_x_fr, s.sigma_x_rl, s.sigma_x_rr);
            LOG_DEBUG("Slip angles α:  FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f rad (%.1f° %.1f° %.1f° %.1f°)",
                     s.alpha_fl, s.alpha_fr, s.alpha_rl, s.alpha_rr,
                     s.alpha_fl * 180.0 / M_PI, s.alpha_fr * 180.0 / M_PI,
                     s.alpha_rl * 180.0 / M_PI, s.alpha_rr * 180.0 / M_PI);
            LOG_DEBUG("Wheel omegas:   FL=%.2f  FR=%.2f  RL=%.2f  RR=%.2f rad/s",
                     s.omega_fl_radps, s.omega_fr_radps, s.omega_rl_radps, s.omega_rr_radps);
            LOG_DEBUG("Friction λ:     FL=%.3f  FR=%.3f  RL=%.3f  RR=%.3f",
                     s.lambda_fl, s.lambda_fr, s.lambda_rl, s.lambda_rr);
            LOG_DEBUG("============================================");
        } // do_log
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
