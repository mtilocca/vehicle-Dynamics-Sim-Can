// src/plant/drive_subsystem/drive_subsystem.cpp
// DriveSubsystem - PhysicsSubsystem wrapper around DrivePlant

#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "utils/logging.hpp"

namespace plant {

DriveSubsystem::DriveSubsystem(const DriveParams& params)
    : drive_(params)
{
}

void DriveSubsystem::initialize(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Initializing: mass=%.0f kg, τ_max=%.0f Nm, P_max=%.0f kW",
             drive_.params().mass_kg,
             drive_.params().motor_torque_max_nm,
             drive_.params().motor_power_max_w / 1000.0);
    LOG_INFO("[DriveSubsystem] Gear ratio=%.1f, η=%.2f, μ=%.2f, L=%.2f m, lf=%.2f m",
             drive_.params().gear_ratio,
             drive_.params().drivetrain_eff,
             drive_.params().mu_surface,
             drive_.params().wheelbase_m,
             drive_.params().cg_to_front_m);
    LOG_INFO("[DriveSubsystem] Brake bias: %.0f%% front / %.0f%% rear",
             drive_.params().brake_bias_front * 100.0,
             (1.0 - drive_.params().brake_bias_front) * 100.0);

    s.v_mps           = 0.0;
    s.a_long_mps2     = 0.0;
    s.motor_torque_nm = 0.0;
    s.brake_force_kN  = 0.0;

    s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
    s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
    s.Fz_fl = s.Fz_fr = s.Fz_rl = s.Fz_rr = 0.0;
}

void DriveSubsystem::reset(PlantState& s) {
    LOG_INFO("[DriveSubsystem] Resetting to zero velocity");
    s.v_mps           = 0.0;
    s.a_long_mps2     = 0.0;
    s.motor_torque_nm = 0.0;
    s.brake_force_kN  = 0.0;
    s.Fx_fl = s.Fx_fr = s.Fx_rl = s.Fx_rr = 0.0;
    s.Fy_fl = s.Fy_fr = s.Fy_rl = s.Fy_rr = 0.0;
}

void DriveSubsystem::pre_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)s; (void)cmd; (void)dt;
}

void DriveSubsystem::step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    drive_.step(s, cmd, dt);
}

void DriveSubsystem::post_step(PlantState& s, const sim::ActuatorCmd& cmd, double dt) {
    (void)s; (void)cmd; (void)dt;
}

void DriveSubsystem::set_params(const DriveParams& params) {
    drive_.params() = params;
}

} // namespace plant
