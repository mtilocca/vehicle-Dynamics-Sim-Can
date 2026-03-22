// test/test_plant_reverse.cpp — GoogleTest migration
//
// Plant integration tests — reverse gear
//
// Covers:
//   - Reverse acceleration: REVERSE gear + negative torque → v_mps < 0
//   - Reverse steering: yaw rate appears while reversing with steer input
//   - Reverse stop + damp: brake halts the truck; vy decays at standstill (Bug #13/#9)

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

// 3a: Reverse acceleration
// REVERSE gear + negative drive torque must produce v_mps < 0.
TEST(PlantReverse, ReverseAcceleration) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd rev;
    rev.system_enable       = true;
    rev.gear_position       = sim::GearPosition::REVERSE;
    rev.steer_cmd_deg       = 0.0;
    rev.drive_torque_cmd_nm = -72500.0;
    rev.brake_cmd_pct       = 0.0;
    run_steps(model, s, rev, 300);

    EXPECT_LT(s.v_mps, 0.0) << "Vehicle must move backward: v=" << s.v_mps;
    EXPECT_LT(s.v_mps, -0.5) << "Meaningful reverse speed must be reached: v=" << s.v_mps;
}

// 3b: Reverse steering
// A steer input while reversing must produce a non-trivial yaw rate.
TEST(PlantReverse, ReverseSteering) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd rev;
    rev.system_enable       = true;
    rev.gear_position       = sim::GearPosition::REVERSE;
    rev.steer_cmd_deg       = 0.0;
    rev.drive_torque_cmd_nm = -72500.0;
    rev.brake_cmd_pct       = 0.0;
    run_steps(model, s, rev, 300);

    ASSERT_LT(s.v_mps, -0.5) << "Pre-condition: reversing at v=" << s.v_mps;

    sim::ActuatorCmd rev_steer;
    rev_steer.system_enable       = true;
    rev_steer.gear_position       = sim::GearPosition::REVERSE;
    rev_steer.steer_cmd_deg       = 10.0;
    rev_steer.drive_torque_cmd_nm = -36000.0;
    rev_steer.brake_cmd_pct       = 0.0;
    run_steps(model, s, rev_steer, 100);

    EXPECT_GT(std::abs(s.yaw_rate_radps), 0.001)
        << "Steer while reversing must generate yaw_rate: |ψ̇|=" << std::abs(s.yaw_rate_radps);
}

// 3c: Reverse stop and standstill damp (Bugs #9/#13)
// Braking from reverse must reduce |v| (Bug #9) and vy must decay (Bug #13).
TEST(PlantReverse, ReverseStopAndDamp) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd rev;
    rev.system_enable       = true;
    rev.gear_position       = sim::GearPosition::REVERSE;
    rev.steer_cmd_deg       = 0.0;
    rev.drive_torque_cmd_nm = -72500.0;
    rev.brake_cmd_pct       = 0.0;
    run_steps(model, s, rev, 300);

    s.vy_mps = 0.5;  // inject lateral velocity for damping check
    const double v_before_brake = s.v_mps;   // negative (reverse)

    sim::ActuatorCmd brake;
    brake.system_enable       = true;
    brake.gear_position       = sim::GearPosition::REVERSE;
    brake.steer_cmd_deg       = 0.0;
    brake.drive_torque_cmd_nm = 0.0;
    brake.brake_cmd_pct       = 40.0;
    run_steps(model, s, brake, 300);

    EXPECT_LT(std::abs(s.v_mps), std::abs(v_before_brake))
        << "Braking from reverse must reduce |v|: |v|=" << std::abs(s.v_mps)
        << " pre-brake=" << std::abs(v_before_brake);
    EXPECT_LT(std::abs(s.vy_mps), 0.1)
        << "vy must be damped at standstill: |vy|=" << std::abs(s.vy_mps);
    EXPECT_GT(s.brake_force_kN, 0.0)
        << "Brake force must be non-zero in reverse: " << s.brake_force_kN << " kN";
}
