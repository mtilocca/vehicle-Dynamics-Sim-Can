// test/test_can_codec.cpp — GoogleTest migration

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include "can/can_frame_compat.hpp"

// Relative-tolerance helper (factor/offset values need this)
static bool is_close(double actual, double expected, double tolerance = 0.001) {
    if (std::abs(expected) < 1e-9) return std::abs(actual) < tolerance;
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

// Helper: build a single-signal FrameDef
static can::FrameDef make_frame(const std::string& name,
                                 int start_bit, int bit_length,
                                 bool is_signed, double factor, double offset,
                                 double min_val = -1e9, double max_val = 1e9) {
    can::FrameDef frame;
    frame.frame_id   = 0x100;
    frame.frame_name = "TEST_FRAME";
    frame.dlc        = 8;

    can::SignalRule sig;
    sig.signal_name   = name;
    sig.start_bit     = start_bit;
    sig.bit_length    = bit_length;
    sig.is_signed     = is_signed;
    sig.factor        = factor;
    sig.offset        = offset;
    sig.min           = min_val;
    sig.max           = max_val;
    sig.default_value = 0.0;
    frame.signals.push_back(sig);
    return frame;
}

TEST(CanCodec, Unsigned8bit) {
    auto frame = make_frame("u8", 0, 8, false, 1.0, 0.0);

    can::SignalMap vals{{"u8", 123.0}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);

    EXPECT_EQ(msg.data[0], 123u);

    auto dec = can::CanCodec::decode_to_map(frame, msg);
    EXPECT_TRUE(is_close(dec["u8"], 123.0)) << "decoded=" << dec["u8"];
}

TEST(CanCodec, Signed16bitPositive) {
    auto frame = make_frame("s16", 0, 16, true, 1.0, 0.0);

    can::SignalMap vals{{"s16", 1000.0}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_TRUE(is_close(dec["s16"], 1000.0)) << "decoded=" << dec["s16"];
}

TEST(CanCodec, Signed16bitNegative) {
    auto frame = make_frame("s16", 0, 16, true, 1.0, 0.0);

    can::SignalMap vals{{"s16", -500.0}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_TRUE(is_close(dec["s16"], -500.0)) << "decoded=" << dec["s16"];
}

TEST(CanCodec, FactorOffsetTemperature) {
    // offset=-40, factor=1 → 25°C stored as raw=65
    auto frame = make_frame("temperature", 0, 8, false, 1.0, -40.0);

    can::SignalMap vals{{"temperature", 25.0}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);

    EXPECT_EQ(msg.data[0], 65u);

    auto dec = can::CanCodec::decode_to_map(frame, msg);
    EXPECT_TRUE(is_close(dec["temperature"], 25.0)) << "decoded=" << dec["temperature"];
}

TEST(CanCodec, FactorOffsetVoltage) {
    auto frame = make_frame("voltage", 0, 16, false, 0.1, 0.0);

    can::SignalMap vals{{"voltage", 385.7}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_NEAR(dec["voltage"], 385.7, 0.15);
}

TEST(CanCodec, MultiByteOffset) {
    // Signal at start_bit=16, 16 bits
    auto frame = make_frame("sig", 16, 16, false, 1.0, 0.0);

    can::SignalMap vals{{"sig", static_cast<double>(0xABCD)}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);

    uint16_t raw = static_cast<uint16_t>((msg.data[3] << 8) | msg.data[2]);
    EXPECT_EQ(raw, static_cast<uint16_t>(0xABCD));

    auto dec = can::CanCodec::decode_to_map(frame, msg);
    EXPECT_TRUE(is_close(dec["sig"], 0xABCD)) << "decoded=" << dec["sig"];
}

TEST(CanCodec, MaxUnsigned) {
    auto frame = make_frame("u8max", 0, 8, false, 1.0, 0.0);

    can::SignalMap vals{{"u8max", 255.0}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);

    EXPECT_EQ(msg.data[0], 255u);
    auto dec = can::CanCodec::decode_to_map(frame, msg);
    EXPECT_TRUE(is_close(dec["u8max"], 255.0));
}

TEST(CanCodec, SignedOverflowClampsToMax) {
    auto frame = make_frame("s8", 0, 8, true, 1.0, 0.0, -128.0, 127.0);

    can::SignalMap vals{{"s8", 200.0}};  // beyond +127
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_LE(dec["s8"], 127.0) << "overflow should clamp to 127";
}

TEST(CanCodec, SignedUnderflowClampsToMin) {
    auto frame = make_frame("s8", 0, 8, true, 1.0, 0.0, -128.0, 127.0);

    can::SignalMap vals{{"s8", -200.0}};  // below -128
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_GE(dec["s8"], -128.0) << "underflow should clamp to -128";
}

TEST(CanCodec, MultipleSignalsRoundTrip) {
    can::FrameDef frame;
    frame.frame_id   = 0x200;
    frame.frame_name = "MULTI";
    frame.dlc        = 8;

    auto make_sig = [](const std::string& n, int sb, int bl, bool sgn,
                       double f, double o, double mn, double mx) {
        can::SignalRule s;
        s.signal_name = n; s.start_bit = sb; s.bit_length = bl;
        s.is_signed = sgn; s.factor = f; s.offset = o;
        s.min = mn; s.max = mx; s.default_value = 0.0;
        return s;
    };
    frame.signals.push_back(make_sig("sig1", 0,  16, false, 0.1, 0.0,    0,  1000));
    frame.signals.push_back(make_sig("sig2", 16, 16, true,  0.1, 0.0, -200,   200));
    frame.signals.push_back(make_sig("sig3", 32,  8, false, 0.5, 0.0,    0,   100));

    can::SignalMap vals{{"sig1", 385.7}, {"sig2", -123.4}, {"sig3", 75.5}};
    struct can_frame msg{};
    can::CanCodec::encode_from_map(frame, vals, msg);
    auto dec = can::CanCodec::decode_to_map(frame, msg);

    EXPECT_NEAR(dec["sig1"], 385.7, 0.15);
    EXPECT_NEAR(dec["sig2"], -123.4, 0.15);
    EXPECT_NEAR(dec["sig3"], 75.5, 0.6);
}
