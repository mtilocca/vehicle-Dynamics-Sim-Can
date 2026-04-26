// test_actuator_cmd_decode.cpp — unit tests for ACTUATOR_CMD_1 decode logic.
#include "zephyr/src/can/actuator_cmd_codec.hpp"
#include <gtest/gtest.h>

using namespace can_codec;
using sim::GearPosition;

// Helper: build an ACTUATOR_CMD_1 payload buffer from physical values.
static void fill_cmd_data(uint8_t d[8], bool enable, GearPosition gear,
                          double steer_deg, double torque_nm,
                          double brake_pct, uint16_t seq = 0)
{
    // Byte 0: enable [0], gear [1:2], mode [3:4]
    d[0] = static_cast<uint8_t>(enable ? 0x01u : 0x00u)
         | static_cast<uint8_t>(static_cast<uint8_t>(gear) << 1);

    // Bytes 1-2: steer (factor 0.1), signed i16
    int16_t steer_raw = static_cast<int16_t>(
        clamp_d(steer_deg / 0.1, -32768.0, 32767.0));
    d[1] = static_cast<uint8_t>( steer_raw        & 0xFF);
    d[2] = static_cast<uint8_t>((steer_raw >> 8)  & 0xFF);

    // Bytes 3-4: torque (factor 10.0), signed i16
    int16_t torque_raw = static_cast<int16_t>(
        clamp_d(torque_nm / 10.0, -32768.0, 32767.0));
    d[3] = static_cast<uint8_t>( torque_raw        & 0xFF);
    d[4] = static_cast<uint8_t>((torque_raw >> 8)  & 0xFF);

    // Byte 5: brake (factor 1.0)
    d[5] = static_cast<uint8_t>(clamp_d(brake_pct, 0.0, 100.0));

    // Bytes 6-7: sequence number
    d[6] = static_cast<uint8_t>( seq        & 0xFF);
    d[7] = static_cast<uint8_t>((seq >> 8)  & 0xFF);
}

// ── system_enable ─────────────────────────────────────────────────────────────

TEST(ActuatorDecode, EnableTrue)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_TRUE(c.system_enable);
}

TEST(ActuatorDecode, EnableFalse)
{
    uint8_t d[8]{};
    fill_cmd_data(d, false, GearPosition::FORWARD, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_FALSE(c.system_enable);
}

// ── gear position ─────────────────────────────────────────────────────────────

TEST(ActuatorDecode, GearNeutral)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::NEUTRAL, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_EQ(c.gear_position, GearPosition::NEUTRAL);
}

TEST(ActuatorDecode, GearForward)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_EQ(c.gear_position, GearPosition::FORWARD);
}

TEST(ActuatorDecode, GearReverse)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::REVERSE, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_EQ(c.gear_position, GearPosition::REVERSE);
}

TEST(ActuatorDecode, GearReservedDefaultsToNeutral)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::RESERVED, 0.0, 0.0, 0.0);
    // RESERVED is value 3; decoder should fall back to NEUTRAL
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_EQ(c.gear_position, GearPosition::NEUTRAL);
}

// ── steer ─────────────────────────────────────────────────────────────────────

TEST(ActuatorDecode, SteerPositive)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 30.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_NEAR(c.steer_cmd_deg, 30.0, 0.05);
}

TEST(ActuatorDecode, SteerNegative)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, -22.5, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_NEAR(c.steer_cmd_deg, -22.5, 0.05);
}

TEST(ActuatorDecode, SteerClampsAtMax)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 500.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.steer_cmd_deg, 45.0);
}

TEST(ActuatorDecode, SteerClampsAtMin)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, -500.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.steer_cmd_deg, -45.0);
}

// ── torque ────────────────────────────────────────────────────────────────────

TEST(ActuatorDecode, TorquePositive)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 50000.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_NEAR(c.drive_torque_cmd_nm, 50000.0, 5.0);  // factor 10 → ±5 Nm
}

TEST(ActuatorDecode, TorqueClampsAtMax)
{
    // max raw i16 * 10 = 327670 Nm, physically clamped to 145000 Nm
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 200000.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.drive_torque_cmd_nm, 145000.0);
}

TEST(ActuatorDecode, TorqueClampsNegativeToZero)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, -1000.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.drive_torque_cmd_nm, 0.0);
}

// ── brake ─────────────────────────────────────────────────────────────────────

TEST(ActuatorDecode, Brake50Pct)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 0.0, 50.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.brake_cmd_pct, 50.0);
}

TEST(ActuatorDecode, BrakeFullStop)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::NEUTRAL, 0.0, 0.0, 100.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.brake_cmd_pct, 100.0);
}

TEST(ActuatorDecode, BrakeZero)
{
    uint8_t d[8]{};
    fill_cmd_data(d, true, GearPosition::FORWARD, 0.0, 0.0, 0.0);
    sim::ActuatorCmd c{};
    decode_actuator_cmd(d, c);
    EXPECT_DOUBLE_EQ(c.brake_cmd_pct, 0.0);
}
