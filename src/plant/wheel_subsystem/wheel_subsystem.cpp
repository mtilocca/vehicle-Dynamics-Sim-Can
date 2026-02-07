// src/plant/wheel_subsystem/wheel_subsystem.cpp
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "utils/logging.hpp"

#include <cmath>

namespace plant {

WheelSubsystem::WheelSubsystem(const WheelSubsystemParams& params)
    : p_(params),
      w_fl_(params.wheel),
      w_fr_(params.wheel),
      w_rl_(params.wheel),
      w_rr_(params.wheel),
      tyre_model_(TyreDugoffParams()),
      tyre_params_(TyreDugoffParams())
{
    apply_tyre_params_();
}

void WheelSubsystem::apply_tyre_params_() {
    // Start from existing
    tyre_params_ = tyre_model_.get_params();

    // Override friction from subsystem params
    tyre_params_.mu_peak  = p_.mu_peak;
    tyre_params_.mu_slide = p_.mu_slide;

    tyre_model_.set_params(tyre_params_);
}

void WheelSubsystem::initialize(PlantState& s) {
    LOG_INFO("[WheelSubsystem] Initializing: Iw=%.1f kg·m², R=%.2f m, mode=DYNAMIC",
             p_.wheel.Iw_kgm2, p_.wheel.R_m);

    // Ensure tyre params consistent
    apply_tyre_params_();

    // Initialize wheel angular speeds to match vehicle speed at start
    const double omega0 = (std::abs(p_.wheel.R_m) > 1e-6) ? (s.v_mps / p_.wheel.R_m) : 0.0;

    s.omega_fl_radps = omega0;
    s.omega_fr_radps = omega0;
    s.omega_rl_radps = omega0;
    s.omega_rr_radps = omega0;

    w_fl_.reset(s.omega_fl_radps);
    w_fr_.reset(s.omega_fr_radps);
    w_rl_.reset(s.omega_rl_radps);
    w_rr_.reset(s.omega_rr_radps);

    // Mark dynamic model enabled
    s.dynamic_model_enabled = true;
    s.surface_mu = p_.mu_peak;
    s.surface_mu_slide = p_.mu_slide;
}

void WheelSubsystem::reset(PlantState& s) {
    LOG_INFO("[WheelSubsystem] Resetting wheel state");

    s.omega_fl_radps = 0.0;
    s.omega_fr_radps = 0.0;
    s.omega_rl_radps = 0.0;
    s.omega_rr_radps = 0.0;

    w_fl_.reset(0.0);
    w_fr_.reset(0.0);
    w_rl_.reset(0.0);
    w_rr_.reset(0.0);

    // Clear forces
    s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
    s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
    s.lambda_fl = s.lambda_fr = s.lambda_rl = s.lambda_rr = 1.0;
    s.sigma_x_fl = s.sigma_x_fr = s.sigma_x_rl = s.sigma_x_rr = 0.0;
    s.sigma_y_fl = s.sigma_y_fr = s.sigma_y_rl = s.sigma_y_rr = 0.0;
}

void WheelSubsystem::set_params(const WheelSubsystemParams& params) {
    p_ = params;

    // Rebuild wheel dynamics with new params
    w_fl_.set_params(p_.wheel);
    w_fr_.set_params(p_.wheel);
    w_rl_.set_params(p_.wheel);
    w_rr_.set_params(p_.wheel);

    apply_tyre_params_();

    LOG_INFO("[WheelSubsystem] Parameters updated: mu_peak=%.2f mu_slide=%.2f R=%.2f Iw=%.1f",
             p_.mu_peak, p_.mu_slide, p_.wheel.R_m, p_.wheel.Iw_kgm2);
}

void WheelSubsystem::compute_normal_loads_(
    PlantState& s,
    double a_long,
    double& Fz_fl, double& Fz_fr, double& Fz_rl, double& Fz_rr
) const {
    // Static axle loads
    const double g = 9.81;
    const double W = p_.mass_kg * g;

    // Static front/rear split from cg distances
    const double L = p_.wheelbase_m;
    const double a = p_.cg_to_front_m; // CG -> front
    const double b = p_.cg_to_rear_m;  // CG -> rear

    double Fz_front_static = (b / L) * W;
    double Fz_rear_static  = (a / L) * W;

    // Longitudinal load transfer ΔF = m*a*h/L
    double dF = 0.0;
    if (p_.enable_load_transfer && L > 1e-6) {
        dF = (p_.mass_kg * a_long * p_.cg_height_m) / L;
    }

    // Under acceleration: transfer to rear (front loses load)
    const double Fz_front = Fz_front_static - dF;
    const double Fz_rear  = Fz_rear_static + dF;

    // Split left/right equally (no lateral transfer here)
    Fz_fl = 0.5 * Fz_front;
    Fz_fr = 0.5 * Fz_front;
    Fz_rl = 0.5 * Fz_rear;
    Fz_rr = 0.5 * Fz_rear;

    // Store in state for logging
    s.Fz_fl = Fz_fl; s.Fz_fr = Fz_fr; s.Fz_rl = Fz_rl; s.Fz_rr = Fz_rr;
}

void WheelSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    if (dt <= 0.0) return;

    // If the global model isn't using dynamics, bail (but you set it true in initialize)
    if (!s.dynamic_model_enabled) {
        s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
        s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
        return;
    }

    // --- Wheel dynamics integration (omega)
    // Brake torques are assumed POSITIVE magnitudes.
    // Drive torques can be +/- (forward / reverse).
    //
    // We apply: tau_net = tau_drive - sign(omega)*tau_brake - tau_road
    // tau_road is handled inside WheelDynamics via Fx*R feedback.
    //
    // NOTE: Front wheels are non-driven in your PlantState, but we still allow tau_drive_* if you set them.

    // Estimate longitudinal acceleration for load transfer using previous state
    const double a_guess = s.a_long_mps2;

    // Compute normal loads first
    double Fz_fl, Fz_fr, Fz_rl, Fz_rr;
    compute_normal_loads_(s, a_guess, Fz_fl, Fz_fr, Fz_rl, Fz_rr);

    // Velocities at contact patch (simple: Vx = vehicle v, Vy=0 in body frame)
    const double Vx = s.v_mps;

    // You can extend Vy later using yaw rate + lateral velocity model.
    const double Vy = 0.0;

    auto step_one = [&](WheelDynamics& wd,
                        double& omega_state,
                        double tau_drive,
                        double tau_brake,
                        double delta_rad,
                        double Fz,
                        double& Fx_out,
                        double& Fy_out,
                        double& sx_out,
                        double& sy_out,
                        double& lam_out)
    {
        // 1) Integrate wheel speed using wheel dynamics (internal direction handling)
        omega_state = wd.step(omega_state, Vx, tau_drive, tau_brake, dt);

        // 2) Tire forces from Dugoff (signature must match your header: 5 args)
        const TyreForces tf = tyre_model_.compute_forces(
            omega_state,
            p_.wheel.R_m,
            Vx,
            Vy,
            Fz
        );

        // 3) Rotate tire forces from wheel frame -> vehicle frame using steer angle
        // (Longitudinal along wheel heading)
        const double c = std::cos(delta_rad);
        const double sn = std::sin(delta_rad);

        // Wheel-frame: Fx along wheel forward, Fy to left of wheel
        // Vehicle-frame: x forward, y left
        Fx_out =  tf.Fx * c - tf.Fy * sn;
        Fy_out =  tf.Fx * sn + tf.Fy * c;

        sx_out  = tf.sigma_x;
        sy_out  = tf.sigma_y;
        lam_out = tf.lambda;
    };

    // Front wheels
    step_one(w_fl_, s.omega_fl_radps, s.tau_drive_fl_nm, s.tau_brake_fl_nm, s.delta_fl_rad, Fz_fl,
             s.Fx_fl, s.Fy_fl, s.sigma_x_fl, s.sigma_y_fl, s.lambda_fl);

    step_one(w_fr_, s.omega_fr_radps, s.tau_drive_fr_nm, s.tau_brake_fr_nm, s.delta_fr_rad, Fz_fr,
             s.Fx_fr, s.Fy_fr, s.sigma_x_fr, s.sigma_y_fr, s.lambda_fr);

    // Rear wheels (driven)
    step_one(w_rl_, s.omega_rl_radps, s.tau_drive_rl_nm, s.tau_brake_rl_nm, 0.0, Fz_rl,
             s.Fx_rl, s.Fy_rl, s.sigma_x_rl, s.sigma_y_rl, s.lambda_rl);

    step_one(w_rr_, s.omega_rr_radps, s.tau_drive_rr_nm, s.tau_brake_rr_nm, 0.0, Fz_rr,
             s.Fx_rr, s.Fy_rr, s.sigma_x_rr, s.sigma_y_rr, s.lambda_rr);

    // Surface friction into PlantState for logging
    s.surface_mu = p_.mu_peak;
    s.surface_mu_slide = p_.mu_slide;
}

} // namespace plant
