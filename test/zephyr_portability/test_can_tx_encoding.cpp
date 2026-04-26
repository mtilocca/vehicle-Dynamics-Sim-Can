// test_can_tx_encoding.cpp — unit tests for the CAN TX bit-packing helpers.
#include "zephyr/src/can/can_tx_codec.hpp"
#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

using namespace can_codec;

// ── pack_i16 / pack_u16 ───────────────────────────────────────────────────────

TEST(PackI16, LittleEndianPositive)
{
    uint8_t d[8]{};
    pack_i16(d, 0, 0x0102);
    EXPECT_EQ(d[0], 0x02u);  // LSB first
    EXPECT_EQ(d[1], 0x01u);
}

TEST(PackI16, LittleEndianNegative)
{
    uint8_t d[8]{};
    pack_i16(d, 2, -1);
    EXPECT_EQ(d[2], 0xFFu);
    EXPECT_EQ(d[3], 0xFFu);
}

TEST(PackI16, DoesNotClobberOtherBytes)
{
    uint8_t d[8];
    memset(d, 0xAA, sizeof(d));
    pack_i16(d, 4, 0x1234);
    EXPECT_EQ(d[0], 0xAAu);
    EXPECT_EQ(d[4], 0x34u);
    EXPECT_EQ(d[5], 0x12u);
    EXPECT_EQ(d[6], 0xAAu);
}

TEST(PackU16, Zero)
{
    uint8_t d[8]{};
    pack_u16(d, 0, 0);
    EXPECT_EQ(d[0], 0u);
    EXPECT_EQ(d[1], 0u);
}

TEST(PackU16, MaxValue)
{
    uint8_t d[8]{};
    pack_u16(d, 0, 0xFFFFu);
    EXPECT_EQ(d[0], 0xFFu);
    EXPECT_EQ(d[1], 0xFFu);
}

// ── enc_i16 ───────────────────────────────────────────────────────────────────

TEST(EncI16, BasicScaling)
{
    // steer_cmd: factor 0.1 → 10.0 deg → raw 100
    EXPECT_EQ(enc_i16(10.0, 0.1), 100);
}

TEST(EncI16, NegativeScaling)
{
    EXPECT_EQ(enc_i16(-20.0, 0.1), -200);
}

TEST(EncI16, ClampsAtMax)
{
    EXPECT_EQ(enc_i16(1e9, 0.1), 32767);
}

TEST(EncI16, ClampsAtMin)
{
    EXPECT_EQ(enc_i16(-1e9, 0.1), -32768);
}

TEST(EncI16, Zero)
{
    EXPECT_EQ(enc_i16(0.0, 0.1), 0);
}

// ── enc_u16 ───────────────────────────────────────────────────────────────────

TEST(EncU16, BasicScaling)
{
    // speed in 0.01 m/s steps: 25.0 m/s → raw 2500
    EXPECT_EQ(enc_u16(25.0, 0.01), 2500u);
}

TEST(EncU16, ClampsNegativeToZero)
{
    EXPECT_EQ(enc_u16(-1.0, 0.01), 0u);
}

TEST(EncU16, ClampsAtMax)
{
    EXPECT_EQ(enc_u16(1e9, 1.0), 65535u);
}

// ── enc_u32 ───────────────────────────────────────────────────────────────────

TEST(EncU32, LargeValue)
{
    // torque 50000 Nm, factor 10 → raw 5000
    EXPECT_EQ(enc_u32(50000.0, 10.0), 5000u);
}

TEST(EncU32, ClampNegative)
{
    EXPECT_EQ(enc_u32(-1.0, 1.0), 0u);
}

// ── round-trip pack+unpack ────────────────────────────────────────────────────

TEST(RoundTrip, I16SteerAngle)
{
    uint8_t d[8]{};
    double steer_phys = 35.2;
    pack_i16(d, 0, enc_i16(steer_phys, 0.1));
    int16_t raw = static_cast<int16_t>((uint16_t)d[0] | ((uint16_t)d[1] << 8));
    double decoded = raw * 0.1;
    EXPECT_NEAR(decoded, steer_phys, 0.05);  // within half a LSB
}

TEST(RoundTrip, NegativeSteer)
{
    uint8_t d[8]{};
    double steer_phys = -22.7;
    pack_i16(d, 0, enc_i16(steer_phys, 0.1));
    int16_t raw = static_cast<int16_t>((uint16_t)d[0] | ((uint16_t)d[1] << 8));
    double decoded = raw * 0.1;
    EXPECT_NEAR(decoded, steer_phys, 0.05);
}
