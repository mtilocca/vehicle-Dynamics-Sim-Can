// src/plant/wheel_subsystem/wheel_subsystem.cpp
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
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

    // Local wheel longitudinal velocity: use vehicle Vx for now (no full body velocities yet)
    const double Vx = s.v_mps;
    const double Vy_fl = 0.0;
    const double Vy_fr = 0.0;
    const double Vy_rl = 0.0;
    const double Vy_rr = 0.0;

    // --- Per-wheel: slip -> tyre forces (Dugoff) -> wheel omega integration ---
    auto process_wheel = [&](WheelDynamics& wd,
                             double tau_drive_nm,
                             double tau_brake_nm,
                             double Vx_mps,
                             double Vy_mps,
                             double Fz_n,
                             double& out_Fx,
                             double& out_Fy,
                             double& out_sigma_x,
                             double& out_sigma_y,
                             double& out_lambda)
    {
        // Compute tyre forces using Dugoff API (5 args, returns TyreForces)
        const TyreForces tf = tyre_model_.compute_forces(
            wd.omega_radps(),
            p_.wheel.radius_m,
            Vx_mps,
            Vy_mps,
            Fz_n
        );

        out_Fx = tf.Fx;
        out_Fy = tf.Fy;
        out_sigma_x = tf.sigma_x;
        out_sigma_y = tf.sigma_y;
        out_lambda = tf.lambda;

        // Make brake oppose current wheel rotation (or Vx if omega ~ 0)
        const double tau_brake_term = brake_opposing_torque(tau_brake_nm, wd.omega_radps(), Vx_mps);

        // Integrate wheel omega with the generated Fx load term
        wd.step(tau_drive_nm, tau_brake_term, out_Fx, dt);
    };

    // If you have Fz already computed elsewhere, use it; otherwise expect it set by your load-transfer model.
    // Here we assume PlantState contains Fz_*.
    process_wheel(wd_fl_, s.tau_drive_fl_nm, s.tau_brake_fl_nm, Vx, Vy_fl, s.Fz_fl,
                  s.Fx_fl, s.Fy_fl, s.sigma_x_fl, s.sigma_y_fl, s.lambda_fl);

    process_wheel(wd_fr_, s.tau_drive_fr_nm, s.tau_brake_fr_nm, Vx, Vy_fr, s.Fz_fr,
                  s.Fx_fr, s.Fy_fr, s.sigma_x_fr, s.sigma_y_fr, s.lambda_fr);

    process_wheel(wd_rl_, s.tau_drive_rl_nm, s.tau_brake_rl_nm, Vx, Vy_rl, s.Fz_rl,
                  s.Fx_rl, s.Fy_rl, s.sigma_x_rl, s.sigma_y_rl, s.lambda_rl);

    process_wheel(wd_rr_, s.tau_drive_rr_nm, s.tau_brake_rr_nm, Vx, Vy_rr, s.Fz_rr,
                  s.Fx_rr, s.Fy_rr, s.sigma_x_rr, s.sigma_y_rr, s.lambda_rr);

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
