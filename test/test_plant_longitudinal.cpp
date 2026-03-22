// test/test_plant_longitudinal.cpp — GoogleTest migration
//
// Plant integration tests — longitudinal dynamics
//
// Covers:
//   - Kinematic startup (Fix A): F=τ/R below 1 m/s, no Dugoff oscillation
//   - Cruise and braking  (Bug #9): brake CAN factor=1, brake_force_kN > 0
//   - Standstill damping  (Bug #13): vy and yaw_rate decay when stopped

#include "plant/plant_model.hpp"
#include "sim/actuator_cmd.hpp"
#include <gtest/gtest.h>
#include <cmath>

// ── XCMG XDE320 plant factory ─────────────────────────────────────────────────
static plant::PlantModelParams make_xcmg_params() {
    plant::PlantModelParams p;
    p.wheelbase_m   = 6.30;
    p.track_width_m = 7.20;

    p.geometry.cg_height_m      = 3.20;
    p.geometry.cg_to_front_m    = 2.52;
    p.geometry.yaw_inertia_kgm2 = 8.5e6;

    p.drive.mass_kg             = 218000.0;
    p.drive.wheel_radius_m      = 1.93;
    p.drive.drag_c              = 1.85;
    p.drive.roll_c              = 9500.0;
    p.drive.motor_torque_max_nm = 145000.0;
    p.drive.brake_torque_max_nm = 180000.0;
    p.drive.gear_ratio          = 28.0;
    p.drive.drivetrain_eff      = 0.92;
    p.drive.motor_power_max_w   = 2013000.0;
    p.drive.v_stop_eps          = 0.5;
    p.drive.v_max_mps           = 17.78;
    p.drive.brake_bias_front    = 0.40;
    p.drive.regen_eff_active    = 0.65;

    p.battery_params.capacity_kWh           = 1650.0;
    p.battery_params.nominal_voltage_v      = 1200.0;
    p.battery_params.max_charge_power_kW    = 600.0;
    p.battery_params.max_discharge_power_kW = 2400.0;
    p.battery_params.efficiency_charge      = 0.91;
    p.battery_params.efficiency_discharge   = 0.93;
    p.battery_params.min_soc                = 0.18;
    p.battery_params.max_soc                = 0.88;

    p.motor_params.max_power_kW  = 2013.0;
    p.motor_params.max_torque_nm = 145000.0;
    p.motor_params.efficiency    = 0.92;

    p.steer.delta_max_deg  = 35.0;
    p.steer.steer_rate_dps = 15.0;

    p.dynamic_config.enabled    = true;
    p.dynamic_config.surface_mu = 0.72;
    return p;
}

static plant::PlantState make_initial_state(const plant::PlantModelParams& p) {
    plant::PlantState s{};
    s.batt_soc_pct          = 50.0;
    s.batt_v                = p.battery_params.nominal_voltage_v;
    s.dynamic_model_enabled = true;
    s.gear_position         = sim::GearPosition::FORWARD;
    return s;
}

static void run_steps(plant::PlantModel& model, plant::PlantState& s,
                      const sim::ActuatorCmd& cmd, int n, double dt = 0.01) {
    for (int i = 0; i < n; i++) model.step(s, cmd, dt);
}

// ─────────────────────────────────────────────────────────────────────────────

// 1a: Kinematic startup (Fix A)
// Below 1 m/s the kinematic fallback uses F=τ/R (no Dugoff oscillation).
TEST(PlantLongitudinal, KinematicStartup) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd cmd;
    cmd.system_enable       = true;
    cmd.gear_position       = sim::GearPosition::FORWARD;
    cmd.drive_torque_cmd_nm = 72500.0;
    cmd.brake_cmd_pct       = 0.0;
    cmd.steer_cmd_deg       = 0.0;

    // 10 steps (0.1 s) — still in kinematic regime
    run_steps(model, s, cmd, 10);

    EXPECT_GT(s.v_mps, 0.0) << "Vehicle must accelerate from rest";
    EXPECT_EQ(s.Fy_fl, 0.0) << "No lateral Dugoff force in kinematic regime";
    EXPECT_EQ(s.Fy_rl, 0.0) << "No lateral Dugoff force in kinematic regime";

    // 240 more steps (total 2.5 s) — cross 1 m/s and enter Dugoff
    run_steps(model, s, cmd, 240);

    EXPECT_GT(s.v_mps, 1.0) << "Must exceed 1 m/s threshold (Dugoff active): v=" << s.v_mps;
    EXPECT_GT(std::abs(s.Fx_rl), 0.0) << "Dugoff Fx_rl non-zero above threshold";
}

// 1b: Cruise and brake (Bug #9)
// Accelerate to cruise, then brake — brake_force_kN must be positive.
TEST(PlantLongitudinal, CruiseAndBrake) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd accel_cmd;
    accel_cmd.system_enable       = true;
    accel_cmd.gear_position       = sim::GearPosition::FORWARD;
    accel_cmd.drive_torque_cmd_nm = 72500.0;
    accel_cmd.brake_cmd_pct       = 0.0;
    run_steps(model, s, accel_cmd, 300);

    EXPECT_GT(s.v_mps, 1.0) << "Cruise must be reached: v=" << s.v_mps;

    // Coast 1 s
    sim::ActuatorCmd coast_cmd;
    coast_cmd.system_enable       = true;
    coast_cmd.gear_position       = sim::GearPosition::FORWARD;
    coast_cmd.drive_torque_cmd_nm = 0.0;
    coast_cmd.brake_cmd_pct       = 0.0;
    run_steps(model, s, coast_cmd, 100);

    // Brake 40 % for 6 s
    sim::ActuatorCmd brake_cmd;
    brake_cmd.system_enable       = true;
    brake_cmd.gear_position       = sim::GearPosition::FORWARD;
    brake_cmd.drive_torque_cmd_nm = 0.0;
    brake_cmd.brake_cmd_pct       = 40.0;
    run_steps(model, s, brake_cmd, 600);

    // Primary assertion for Bug #9: brake CAN factor=1 → brake force is non-zero.
    EXPECT_GT(s.brake_force_kN, 0.0) << "Brake force must be non-zero: " << s.brake_force_kN << " kN";
}

// 1c: Standstill damping (Bug #13)
// Inject vy and yaw_rate at standstill; both must decay within 2 s.
TEST(PlantLongitudinal, StandstillDamping) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    s.v_mps          = 0.0;
    s.vy_mps         = 1.5;
    s.yaw_rate_radps = 0.2;

    sim::ActuatorCmd cmd;
    cmd.system_enable       = true;
    cmd.gear_position       = sim::GearPosition::FORWARD;
    cmd.drive_torque_cmd_nm = 0.0;
    cmd.brake_cmd_pct       = 0.0;

    // 2 s (vy(t) = 1.5*exp(-2t): need ~1.35 s to reach 0.1 m/s)
    run_steps(model, s, cmd, 200);

    EXPECT_LT(std::abs(s.vy_mps), 0.1)
        << "vy must be damped at standstill: |vy|=" << std::abs(s.vy_mps) << " m/s";
    EXPECT_LT(std::abs(s.yaw_rate_radps), 0.05)
        << "yaw_rate must be damped at standstill: |ψ̇|=" << s.yaw_rate_radps << " rad/s";
}
