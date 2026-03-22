// test/test_plant_state_packer.cpp — GoogleTest migration
/**
 * Validation Test: PlantStatePacker Refactor
 *
 * Verifies that the visitor-based PlantStatePacker produces correct output.
 *
 * Test Strategy:
 *   1. Create a PlantState with known values
 *   2. Pack each CAN frame using the implementation
 *   3. Verify all expected signals are present with correct values
 */

#include "plant/plant_main/plant_state.hpp"
#include "plant/plant_main/plant_state_visitor.hpp"
#include "sim/plant_state_packer.hpp"
#include "can/can_map.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <string>

// Helper: relative tolerance check (0.01%)
static bool is_close(double actual, double expected, double tolerance = 0.0001) {
    if (std::abs(expected) < 1e-9) return std::abs(actual) < tolerance;
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

static plant::PlantState create_test_state() {
    plant::PlantState s;
    s.t_s             = 10.5;
    s.x_m             = 123.45;
    s.y_m             = 67.89;
    s.yaw_rad         = 0.785398;   // 45 degrees
    s.v_mps           = 25.0;
    s.a_long_mps2     = 2.5;
    s.steer_virtual_rad = 0.174533; // 10 degrees
    s.steer_rate_radps  = 0.1;
    s.delta_fl_rad    = 0.19;
    s.delta_fr_rad    = 0.16;
    s.batt_soc_pct    = 75.5;
    s.batt_v          = 385.0;
    s.batt_i          = 150.0;
    s.batt_temp_c     = 28.5;
    s.motor_torque_nm = 1200.0;
    s.motor_power_kW  = 80.5;
    s.regen_power_kW  = 0.0;
    s.brake_force_kN  = 0.0;
    s.wheel_fl_rps    = 25.2;
    s.wheel_fr_rps    = 25.3;
    s.wheel_rl_rps    = 25.1;
    s.wheel_rr_rps    = 25.0;
    s.status_flags    = 0;
    return s;
}

// ── Fixture: loads CAN map once per test ──────────────────────────────────────
class PlantStatePackerTest : public ::testing::Test {
protected:
    can::CanMap map;

    void SetUp() override {
        ASSERT_TRUE(map.load(TEST_SOURCE_DIR "/config/can_map.csv"))
            << "Could not load CAN map from: " TEST_SOURCE_DIR "/config/can_map.csv";
    }
};

// ─────────────────────────────────────────────────────────────────────────────

TEST_F(PlantStatePackerTest, VisitorEnumeratesFields) {
    plant::PlantState state = create_test_state();

    int field_count = 0;
    bool found_speed  = false;
    bool found_soc    = false;
    bool found_torque = false;

    auto visitor = plant::make_visitor([&](const char* name, double value) {
        ++field_count;
        std::string n(name);
        if (n == "vehicle_speed_mps") {
            found_speed = true;
            EXPECT_TRUE(is_close(value, 25.0)) << "vehicle_speed_mps value incorrect: " << value;
        } else if (n == "batt_soc_pct") {
            found_soc = true;
            EXPECT_TRUE(is_close(value, 75.5)) << "batt_soc_pct value incorrect: " << value;
        } else if (n == "motor_torque_nm") {
            found_torque = true;
            EXPECT_TRUE(is_close(value, 1200.0)) << "motor_torque_nm value incorrect: " << value;
        }
    });

    state.accept_fields(visitor);

    EXPECT_GE(field_count, 20) << "Too few fields enumerated: " << field_count;
    EXPECT_TRUE(found_speed)  << "vehicle_speed_mps not found in visitor output";
    EXPECT_TRUE(found_soc)    << "batt_soc_pct not found in visitor output";
    EXPECT_TRUE(found_torque) << "motor_torque_nm not found in visitor output";
}

TEST_F(PlantStatePackerTest, AllPlantFramesPack) {
    plant::PlantState state = create_test_state();

    const uint32_t plant_frame_ids[] = {
        0x300,  // VEHICLE_STATE_1
        0x310,  // MOTOR_STATE_1
        0x320,  // BRAKE_STATE
        0x330,  // POSITION_STATE
        0x331,  // ORIENTATION_STATE
        0x340,  // DRIVETRAIN_STATE
        0x3F0   // DIAGNOSTIC_STATE
    };

    for (uint32_t frame_id : plant_frame_ids) {
        const can::FrameDef* frame_def = map.find_tx_frame(frame_id);
        ASSERT_NE(frame_def, nullptr)
            << "Frame 0x" << std::hex << frame_id << " not found in map";

        can::SignalMap signals = sim::PlantStatePacker::pack(state, *frame_def);
        EXPECT_FALSE(signals.empty())
            << "Frame 0x" << std::hex << frame_id << " produced empty signal map";

        // All signals in the frame definition must be present
        for (const auto& sig_def : frame_def->signals) {
            EXPECT_NE(signals.find(sig_def.signal_name), signals.end())
                << "Signal '" << sig_def.signal_name
                << "' missing from frame 0x" << std::hex << frame_id;
        }
    }
}

TEST_F(PlantStatePackerTest, MotorRpmDerivedCorrectly) {
    plant::PlantState state = create_test_state();
    const can::FrameDef* motor_frame = map.find_tx_frame(0x310);
    ASSERT_NE(motor_frame, nullptr);

    can::SignalMap signals = sim::PlantStatePacker::pack(state, *motor_frame);
    auto it = signals.find("motor_speed_rpm");
    ASSERT_NE(it, signals.end()) << "motor_speed_rpm not found in MOTOR_STATE_1";

    // Expected: (v / r) * gear_ratio * 60 / (2π)
    double expected_rpm = (25.0 / 0.33) * 9.0 * 60.0 / (2.0 * M_PI);
    EXPECT_TRUE(is_close(it->second, expected_rpm, 0.01))
        << "motor_speed_rpm incorrect: got " << it->second << ", expected " << expected_rpm;
}

TEST_F(PlantStatePackerTest, YawConvertedToDegrees) {
    plant::PlantState state = create_test_state();
    const can::FrameDef* orient_frame = map.find_tx_frame(0x331);
    ASSERT_NE(orient_frame, nullptr);

    can::SignalMap signals = sim::PlantStatePacker::pack(state, *orient_frame);
    auto it = signals.find("yaw_deg");
    ASSERT_NE(it, signals.end()) << "yaw_deg not found in ORIENTATION_STATE";

    double expected_deg = state.yaw_rad * 180.0 / M_PI;
    EXPECT_TRUE(is_close(it->second, expected_deg))
        << "yaw_deg incorrect: got " << it->second << ", expected " << expected_deg;
}

TEST_F(PlantStatePackerTest, BatteryPowerCalculated) {
    plant::PlantState state = create_test_state();
    const can::FrameDef* batt_frame = map.find_tx_frame(0x230);
    ASSERT_NE(batt_frame, nullptr);

    can::SignalMap signals = sim::PlantStatePacker::pack(state, *batt_frame);
    auto it = signals.find("batt_power_kw");
    ASSERT_NE(it, signals.end()) << "batt_power_kw not found in BATT_STATE";

    // Expected: V * I / 1000 = 385 * 150 / 1000 = 57.75 kW
    double expected_power = (state.batt_v * state.batt_i) / 1000.0;
    EXPECT_TRUE(is_close(it->second, expected_power))
        << "batt_power_kw incorrect: got " << it->second << ", expected " << expected_power;
}
