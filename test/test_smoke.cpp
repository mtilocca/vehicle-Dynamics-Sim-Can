// test/test_smoke.cpp — Smoke tests
//
// Three high-level checks that the key subsystems wire together correctly:
//   1. PlantModel cold-start: construct + step once, no crash
//   2. CAN encode/decode round-trip: ACTUATOR_CMD_1 signals
//   3. sim_main binary: spawn with --fast --duration 1, assert exit 0

#include "plant/plant_model.hpp"
#include "sim/actuator_cmd.hpp"
#include "can/can_map.hpp"
#include "can/can_codec.hpp"
#include <gtest/gtest.h>
#include <linux/can.h>
#include <cmath>
#include <cstdlib>
#include <string>

// ── Shared XCMG factory (same as plant test files) ───────────────────────────
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

// ─────────────────────────────────────────────────────────────────────────────

// Smoke 1: PlantModel cold-start
TEST(Smoke, PlantModelBoots) {
    ASSERT_NO_THROW({
        plant::PlantModelParams p = make_xcmg_params();
        plant::PlantModel model(p);

        plant::PlantState s{};
        s.batt_soc_pct          = 50.0;
        s.batt_v                = p.battery_params.nominal_voltage_v;
        s.dynamic_model_enabled = true;
        s.gear_position         = sim::GearPosition::FORWARD;

        sim::ActuatorCmd cmd{};
        cmd.system_enable       = true;
        cmd.gear_position       = sim::GearPosition::FORWARD;
        cmd.drive_torque_cmd_nm = 72500.0;

        model.step(s, cmd, 0.01);
    });
}

// Smoke 2: CAN encode/decode round-trip (ACTUATOR_CMD_1 @ 0x100)
TEST(Smoke, CanRoundTrip) {
    can::CanMap map;
    ASSERT_TRUE(map.load(CAN_MAP_PATH))
        << "Could not load CAN map: " CAN_MAP_PATH;

    // J1939 ID: Priority=6, PGN=0xEF00 (PDU1), SA=0x21, DA=0xF0 → 0x18EFF021
    const auto* def = map.find_rx_frame(CAN_EFF_FLAG | 0x18EFF021u);  // ACTUATOR_CMD_1
    ASSERT_NE(def, nullptr) << "ACTUATOR_CMD_1 (J1939 0x18EFF021) not found in CAN map";

    // Encode known values
    can::SignalMap input{
        {"drive_torque_cmd_nm", 72500.0},
        {"brake_cmd_pct",       40.0},
        {"steer_cmd_deg",       10.0},
        {"gear_position",       1.0},    // FORWARD
        {"system_enable",       1.0},
    };
    struct can_frame frame{};
    can::CanCodec::encode_from_map(*def, input, frame);

    // Decode and compare (within 1 LSB quantization)
    auto decoded = can::CanCodec::decode_to_map(*def, frame);
    EXPECT_NEAR(decoded.at("drive_torque_cmd_nm"), 72500.0, 10.0);   // factor=10
    EXPECT_NEAR(decoded.at("brake_cmd_pct"),       40.0,    1.0);
    EXPECT_NEAR(decoded.at("steer_cmd_deg"),        10.0,   0.2);
    EXPECT_NEAR(decoded.at("gear_position"),         1.0,   0.5);
    EXPECT_NEAR(decoded.at("system_enable"),         1.0,   0.5);
}

// Smoke 3: sim_main binary runs cleanly for 1 simulated second
TEST(Smoke, SimMainRunsCleanly) {
    std::string cmd = std::string(SIM_MAIN_PATH)
        + " --fast --duration 1"
        + " --log-level off"
        + " --no-can-tx"
        + " --can-map " + CAN_MAP_PATH;
    int ret = std::system(cmd.c_str());
    EXPECT_EQ(ret, 0) << "sim_main exited with non-zero code: " << ret;
}
