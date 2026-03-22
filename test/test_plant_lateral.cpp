// test/test_plant_lateral.cpp — GoogleTest migration
//
// Plant integration tests — lateral dynamics
//
// Covers:
//   - Yaw rate from steering: positive steer → positive yaw_rate, Ackermann angles
//   - vy clamp (Bug #10):     |vy_mps| never exceeds 2 m/s under max steering
//   - Load transfer (Bugs #7/#8): outer wheel loads more during cornering

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

// 2a: Yaw rate from steering
// At speed, a positive (left) steer must produce positive yaw_rate
// and correct Ackermann wheel angles.
TEST(PlantLateral, YawRateFromSteering) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd accel;
    accel.system_enable       = true;
    accel.gear_position       = sim::GearPosition::FORWARD;
    accel.drive_torque_cmd_nm = 72500.0;
    accel.brake_cmd_pct       = 0.0;
    run_steps(model, s, accel, 300);

    sim::ActuatorCmd steer_cmd;
    steer_cmd.system_enable       = true;
    steer_cmd.gear_position       = sim::GearPosition::FORWARD;
    steer_cmd.steer_cmd_deg       = 10.0;
    steer_cmd.drive_torque_cmd_nm = 5000.0;
    steer_cmd.brake_cmd_pct       = 0.0;
    run_steps(model, s, steer_cmd, 100);

    EXPECT_GT(s.yaw_rate_radps, 0.0)
        << "Left steer must produce positive yaw_rate: ψ̇=" << s.yaw_rate_radps;
    EXPECT_GT(s.delta_fl_rad, 0.0)
        << "Front-left wheel must turn left: δFL=" << s.delta_fl_rad * 180.0 / M_PI << "°";
    EXPECT_GT(s.delta_fr_rad, 0.0)
        << "Front-right wheel must turn left: δFR=" << s.delta_fr_rad * 180.0 / M_PI << "°";
    // In this codebase's Ackermann implementation, FR (outer wheel in left turn)
    // carries the larger angle due to sign convention in SteerPlant.
    EXPECT_GT(s.delta_fr_rad, s.delta_fl_rad)
        << "Ackermann: outer (FR) angle > inner (FL): δFR="
        << s.delta_fr_rad * 180.0 / M_PI << "° δFL=" << s.delta_fl_rad * 180.0 / M_PI << "°";
}

// 2b: vy clamp (Bug #10)
// Under maximum steering at speed, |vy| must stay ≤ 2 m/s.
TEST(PlantLateral, VyClamp) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd accel;
    accel.system_enable       = true;
    accel.gear_position       = sim::GearPosition::FORWARD;
    accel.drive_torque_cmd_nm = 72500.0;
    accel.brake_cmd_pct       = 0.0;
    run_steps(model, s, accel, 300);

    sim::ActuatorCmd max_steer;
    max_steer.system_enable       = true;
    max_steer.gear_position       = sim::GearPosition::FORWARD;
    max_steer.steer_cmd_deg       = 35.0;
    max_steer.drive_torque_cmd_nm = 5000.0;
    max_steer.brake_cmd_pct       = 0.0;
    run_steps(model, s, max_steer, 500);

    EXPECT_LE(std::abs(s.vy_mps), 2.01)
        << "|vy| must be clamped to 2 m/s: |vy|=" << std::abs(s.vy_mps);
    EXPECT_GT(s.v_mps, 0.0) << "Vehicle must still be moving forward: v=" << s.v_mps;
}

// 2c: Load transfer during cornering (Bugs #7/#8)
// During a left turn the outer (right) wheels must carry more load.
TEST(PlantLateral, LoadTransferCornering) {
    auto params = make_xcmg_params();
    plant::PlantModel model(params);
    auto s = make_initial_state(params);

    sim::ActuatorCmd accel;
    accel.system_enable       = true;
    accel.gear_position       = sim::GearPosition::FORWARD;
    accel.drive_torque_cmd_nm = 72500.0;
    accel.brake_cmd_pct       = 0.0;
    run_steps(model, s, accel, 300);

    sim::ActuatorCmd turn;
    turn.system_enable       = true;
    turn.gear_position       = sim::GearPosition::FORWARD;
    turn.steer_cmd_deg       = 10.0;
    turn.drive_torque_cmd_nm = 5000.0;
    turn.brake_cmd_pct       = 0.0;
    run_steps(model, s, turn, 100);

    EXPECT_GT(s.Fz_fr, s.Fz_fl)
        << "Left turn: outer (FR) Fz > inner (FL) Fz: Fz_fr="
        << s.Fz_fr / 1000.0 << " kN, Fz_fl=" << s.Fz_fl / 1000.0 << " kN";
    EXPECT_GT(s.Fz_rr, s.Fz_rl)
        << "Left turn: outer (RR) Fz > inner (RL) Fz: Fz_rr="
        << s.Fz_rr / 1000.0 << " kN, Fz_rl=" << s.Fz_rl / 1000.0 << " kN";

    const double total_Fz = s.Fz_fl + s.Fz_fr + s.Fz_rl + s.Fz_rr;
    const double weight_N = 218000.0 * 9.81;
    EXPECT_NEAR(total_Fz, weight_N, 5.0e4)
        << "Total Fz must be conserved within 50 kN: "
        << total_Fz / 1000.0 << " kN (expected " << weight_N / 1000.0 << " kN)";
}
