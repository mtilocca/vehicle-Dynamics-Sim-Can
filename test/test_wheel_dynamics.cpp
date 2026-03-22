// test/test_wheel_dynamics.cpp — GoogleTest migration
// Reference: Vehicle_Dynamics_with_Dugoff_Tire_Model.pdf

#include "plant/tyre_models/tyre_dugoff.hpp"
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace plant;

// ── Shared Dugoff tyre params ─────────────────────────────────────────────────
static TyreDugoffParams make_tyre_params() {
    TyreDugoffParams p;
    p.mu_peak   = 0.72;
    p.Cx_base   = 280000.0;
    p.Cy_base   = 220000.0;
    p.Fz_ref    = 100000.0;
    return p;
}

// ── Shared WheelSubsystem params ──────────────────────────────────────────────
static WheelSubsystemParams make_ws_params(bool dynamic = true) {
    WheelSubsystemParams p;
    p.mass_kg        = 218000.0;
    p.wheelbase_m    = 6.3;
    p.track_m        = 7.2;
    p.cg_height_m    = 3.2;
    p.cg_to_front_m  = 3.78;
    p.wheel.radius_m      = 1.93;
    p.wheel.inertia_kgm2  = 1000.0;
    p.tyre_params    = make_tyre_params();
    p.mu_peak        = 0.72;
    p.mu_slide       = 0.65;
    p.dynamic_mode_enabled = dynamic;
    return p;
}

// ─────────────────────────────────────────────────────────────────────────────

TEST(WheelDynamics, FreeRollingSlipNearZero) {
    TyreDugoff tyre(make_tyre_params());
    const double R = 1.93, Vx = 10.0;
    auto f = tyre.compute_forces(Vx / R, R, Vx, 0.0, 100000.0);
    EXPECT_NEAR(f.sigma_x, 0.0, 0.01);
}

TEST(WheelDynamics, DriveSlipPositive) {
    TyreDugoff tyre(make_tyre_params());
    const double R = 1.93, Vx = 10.0;
    auto f = tyre.compute_forces(Vx / R * 1.1, R, Vx, 0.0, 100000.0);
    EXPECT_GT(f.sigma_x, 0.05) << "drive slip sigma_x should be positive";
}

TEST(WheelDynamics, BrakeSlipNegative) {
    TyreDugoff tyre(make_tyre_params());
    const double R = 1.93, Vx = 10.0;
    auto f = tyre.compute_forces(Vx / R * 0.8, R, Vx, 0.0, 100000.0);
    EXPECT_LT(f.sigma_x, -0.10) << "brake slip sigma_x should be negative";
}

TEST(WheelDynamics, FrictionCircleRespected) {
    TyreDugoff tyre(make_tyre_params());
    const double Fz = 500000.0;
    const double F_max = 0.72 * Fz;
    auto f = tyre.compute_forces(10.0 / 1.93 * 1.5, 1.93, 10.0, 0.5, Fz);
    double F_total = std::sqrt(f.Fx * f.Fx + f.Fy * f.Fy);
    EXPECT_LE(F_total, F_max * 1.01) << "|F| must not exceed μFz";
    EXPECT_LT(f.lambda, 1.0) << "lambda < 1 in saturated regime";
}

TEST(WheelDynamics, IntegrationFreeSpin) {
    WheelDynamicsParams p; p.inertia_kgm2 = 1000.0; p.radius_m = 1.93;
    WheelDynamics wheel(p);
    wheel.set_omega_radps(5.0);

    const double tau = 10000.0, dt = 0.001;
    const double before = wheel.omega_radps();
    wheel.step(tau, 0.0, 0.0, dt);

    const double expected_delta = (tau / p.inertia_kgm2) * dt;
    EXPECT_NEAR(wheel.omega_radps() - before, expected_delta, 0.001);
}

TEST(WheelDynamics, StabilityCriterion) {
    WheelDynamicsParams p; p.inertia_kgm2 = 1000.0; p.radius_m = 1.93;
    WheelDynamics wheel(p);

    const double Cx = 280000.0;
    double dt_max    = wheel.get_stability_dt_max(Cx);
    double expected  = 2.0 * p.inertia_kgm2 / (Cx * p.radius_m * p.radius_m);

    EXPECT_NEAR(dt_max, expected, 0.0001);
    EXPECT_GT(dt_max, 0.001);
    EXPECT_LT(dt_max, 0.003);
}

TEST(WheelDynamics, SubsystemInit) {
    auto params = make_ws_params();
    WheelSubsystem sub(params);
    PlantState s; s.v_mps = 10.0;
    sub.initialize(s);

    const double expected_omega = s.v_mps / params.wheel.radius_m;
    EXPECT_NEAR(s.omega_rl_radps, expected_omega, 0.01);

    const double W = 218000.0 * 9.81;
    const double Wf = W * (params.wheelbase_m - params.cg_to_front_m) / params.wheelbase_m;
    const double Wr = W * params.cg_to_front_m / params.wheelbase_m;
    EXPECT_NEAR(s.Fz_fl + s.Fz_fr, Wf, 1000.0);
    EXPECT_NEAR(s.Fz_rl + s.Fz_rr, Wr, 1000.0);

    EXPECT_EQ(sub.priority(), 105);
    EXPECT_TRUE(s.dynamic_model_enabled);
}

TEST(WheelDynamics, DynamicStepAcceleratesWheel) {
    auto params = make_ws_params();
    params.tyre_params.Cx_base = 280000.0;
    params.tyre_params.Fz_ref  = 100000.0;
    WheelSubsystem sub(params);

    PlantState s; s.v_mps = 10.0;
    s.tau_drive_rl_nm = s.tau_drive_rr_nm = 50000.0;
    sub.initialize(s);

    sim::ActuatorCmd cmd;
    const double before = s.omega_rl_radps;
    for (int i = 0; i < 5; ++i) sub.step(s, cmd, 0.001);

    EXPECT_GT(s.omega_rl_radps, before) << "driven wheel should spin up";
    EXPECT_NE(s.Fx_rl, 0.0);
    EXPECT_GT(s.sigma_x_rl, 0.0);
}

TEST(WheelDynamics, LongitudinalLoadTransfer) {
    auto params = make_ws_params();
    WheelSubsystem sub(params);

    PlantState s_static; s_static.v_mps = 10.0; s_static.a_long_mps2 = 0.0;
    sub.initialize(s_static);
    const double Fzf_static = s_static.Fz_fl + s_static.Fz_fr;
    const double Fzr_static = s_static.Fz_rl + s_static.Fz_rr;

    PlantState s_accel = s_static; s_accel.a_long_mps2 = 2.0;
    sim::ActuatorCmd cmd;
    sub.step(s_accel, cmd, 0.01);
    const double Fzf_accel = s_accel.Fz_fl + s_accel.Fz_fr;
    const double Fzr_accel = s_accel.Fz_rl + s_accel.Fz_rr;

    EXPECT_LT(Fzf_accel, Fzf_static) << "front unloads during acceleration";
    EXPECT_GT(Fzr_accel, Fzr_static) << "rear loads during acceleration";

    // Expected ΔFz = m·a·h/L
    const double expected_dFz = 218000.0 * 2.0 * params.cg_height_m / params.wheelbase_m;
    EXPECT_NEAR(Fzr_accel - Fzr_static, expected_dFz, 2000.0);
}

TEST(WheelDynamics, KinematicModeWheelSyncV) {
    auto params = make_ws_params(false);  // kinematic
    WheelSubsystem sub(params);
    PlantState s; s.v_mps = 15.0;
    sub.initialize(s);
    sim::ActuatorCmd cmd;
    sub.step(s, cmd, 0.01);
    EXPECT_NEAR(s.omega_rl_radps, s.v_mps / params.wheel.radius_m, 0.001);
}

TEST(WheelDynamics, DynamicModeWheelSpinsFaster) {
    auto params = make_ws_params(true);  // dynamic
    WheelSubsystem sub(params);
    PlantState s; s.v_mps = 15.0;
    s.tau_drive_rl_nm = s.tau_drive_rr_nm = 100000.0;
    sub.initialize(s);
    sim::ActuatorCmd cmd;
    for (int i = 0; i < 10; ++i) sub.step(s, cmd, 0.001);

    EXPECT_GT(s.omega_rl_radps, s.v_mps / params.wheel.radius_m)
        << "driven wheel should develop positive slip";
}
