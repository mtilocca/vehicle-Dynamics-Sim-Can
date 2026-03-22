// test/test_sensor_packer.cpp — GoogleTest migration

#include "can/sensor_state_packer.hpp"
#include "sensors/sensor_out.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>

// Little-endian byte extraction helpers
static uint16_t u16(const uint8_t* d, int off) {
    return static_cast<uint16_t>((d[off + 1] << 8) | d[off]);
}
static int16_t s16(const uint8_t* d, int off) {
    return static_cast<int16_t>(u16(d, off));
}

TEST(SensorPacker, BatteryVoltage) {
    sensors::SensorOut sens{};
    sens.batt_v_meas = 385.7;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = u16(data, 0) * 0.1;
    EXPECT_NEAR(decoded, 385.7, 0.15);
}

TEST(SensorPacker, BatteryCurrentNegative) {
    sensors::SensorOut sens{};
    sens.batt_i_meas = -150.3;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = s16(data, 2) * 0.1;
    EXPECT_NEAR(decoded, -150.3, 0.15);
}

TEST(SensorPacker, BatteryCurrentPositive) {
    sensors::SensorOut sens{};
    sens.batt_i_meas = 75.6;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = s16(data, 2) * 0.1;
    EXPECT_NEAR(decoded, 75.6, 0.15);
}

TEST(SensorPacker, BatterySOC) {
    sensors::SensorOut sens{};
    sens.batt_soc_meas = 75.5;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = data[4] * 0.5;
    EXPECT_NEAR(decoded, 75.5, 0.6);
}

TEST(SensorPacker, BatteryTempPositive) {
    sensors::SensorOut sens{};
    sens.batt_temp_meas = 25.0;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = data[5] + (-40.0);
    EXPECT_NEAR(decoded, 25.0, 1.1);
}

TEST(SensorPacker, BatteryTempNegative) {
    sensors::SensorOut sens{};
    sens.batt_temp_meas = -10.0;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded = data[5] + (-40.0);
    EXPECT_NEAR(decoded, -10.0, 1.1);
}

TEST(SensorPacker, BatteryPower) {
    sensors::SensorOut sens{};
    sens.batt_v_meas = 385.0;
    sens.batt_i_meas = 150.0;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    double decoded  = s16(data, 6) * 0.1;
    double expected = (sens.batt_v_meas * sens.batt_i_meas) / 1000.0;
    EXPECT_NEAR(decoded, expected, 0.15);
}

TEST(SensorPacker, WheelSpeeds) {
    sensors::SensorOut sens{};
    sens.wheel_fl_rps_meas = 25.3;
    sens.wheel_fr_rps_meas = 25.1;
    sens.wheel_rl_rps_meas = 24.9;
    sens.wheel_rr_rps_meas = 25.0;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_wheel_speeds(sens, data);

    EXPECT_NEAR(s16(data, 0) * 0.01, 25.3, 0.015);
    EXPECT_NEAR(s16(data, 2) * 0.01, 25.1, 0.015);
    EXPECT_NEAR(s16(data, 4) * 0.01, 24.9, 0.015);
    EXPECT_NEAR(s16(data, 6) * 0.01, 25.0, 0.015);
}

TEST(SensorPacker, WheelSpeedsReverse) {
    sensors::SensorOut sens{};
    sens.wheel_fl_rps_meas = -10.5;
    sens.wheel_fr_rps_meas = -10.3;
    sens.wheel_rl_rps_meas = -10.6;
    sens.wheel_rr_rps_meas = -10.4;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_wheel_speeds(sens, data);

    EXPECT_NEAR(s16(data, 0) * 0.01, -10.5, 0.015);
}

TEST(SensorPacker, BatteryFrameIntegrity) {
    sensors::SensorOut sens{};
    sens.batt_v_meas   = 400.0;
    sens.batt_i_meas   = 200.0;
    sens.batt_soc_meas = 85.0;
    sens.batt_temp_meas = 30.0;

    uint8_t data[8]{};
    can::SensorStatePacker::pack_battery(sens, data);

    EXPECT_NEAR(u16(data, 0) * 0.1,       400.0, 0.15);   // voltage
    EXPECT_NEAR(s16(data, 2) * 0.1,       200.0, 0.15);   // current
    EXPECT_NEAR(data[4]      * 0.5,        85.0, 0.6 );   // SOC
    EXPECT_NEAR(data[5]      + (-40.0),    30.0, 1.1 );   // temp
    EXPECT_NEAR(s16(data, 6) * 0.1,        80.0, 0.15);   // power (400V*200A/1000)
}
