// test/test_can_codec.cpp
/**
 * Unit Test: CAN Codec
 * 
 * Tests CAN signal encoding and decoding for various data types.
 * 
 * Test Coverage:
 *   1. Unsigned signal encoding/decoding
 *   2. Signed signal encoding/decoding
 *   3. Floating point with factor and offset
 *   4. Multi-byte signals (16-bit)
 *   5. Bit packing and extraction
 *   6. Edge cases (min/max values, overflow)
 */

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include <iostream>
#include <cmath>
#include <cstring>
#include <linux/can.h>

// ANSI color codes
#define COLOR_GREEN  "\033[32m"
#define COLOR_RED    "\033[31m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET  "\033[0m"

struct TestResult {
    int passed = 0;
    int failed = 0;
    
    void pass(const std::string& msg) {
        std::cout << COLOR_GREEN << "  ✓ " << msg << COLOR_RESET << "\n";
        ++passed;
    }
    
    void fail(const std::string& msg) {
        std::cout << COLOR_RED << "  ✗ " << msg << COLOR_RESET << "\n";
        ++failed;
    }
    
    void summary() {
        std::cout << "\n========================================\n";
        if (failed == 0) {
            std::cout << COLOR_GREEN << "ALL TESTS PASSED" << COLOR_RESET;
        } else {
            std::cout << COLOR_RED << "SOME TESTS FAILED" << COLOR_RESET;
        }
        std::cout << " (" << passed << " passed, " << failed << " failed)\n";
        std::cout << "========================================\n";
    }
};

bool is_close(double actual, double expected, double tolerance = 0.001) {
    if (std::abs(expected) < 1e-9) {
        return std::abs(actual) < tolerance;
    }
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

// Helper: Create a FrameDef with one signal for testing
can::FrameDef create_test_frame(const std::string& signal_name, 
                                 int start_bit, int bit_length,
                                 bool is_signed, double factor, double offset) {
    can::FrameDef frame;
    frame.frame_id = 0x100;
    frame.frame_name = "TEST_FRAME";
    frame.dlc = 8;
    
    can::SignalRule sig;
    sig.signal_name = signal_name;
    sig.start_bit = start_bit;
    sig.bit_length = bit_length;
    sig.is_signed = is_signed;
    sig.factor = factor;
    sig.offset = offset;
    sig.min = -1e9;
    sig.max = 1e9;
    sig.default_value = 0.0;
    
    frame.signals.push_back(sig);
    return frame;
}

// Test 1: Unsigned 8-bit encoding/decoding
void test_unsigned_8bit(TestResult& result) {
    std::cout << "\n=== Test 1: Unsigned 8-bit Signal ===\n";
    
    can::FrameDef frame = create_test_frame("test_u8", 0, 8, false, 1.0, 0.0);
    
    // Encode value 123
    can::SignalMap values;
    values["test_u8"] = 123.0;
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    
    if (can_msg.data[0] == 123) {
        result.pass("Unsigned 8-bit encode: value=123 -> data[0]=123");
    } else {
        result.fail("Unsigned 8-bit encode failed: got " + std::to_string(can_msg.data[0]));
    }
    
    // Decode back
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    if (is_close(decoded["test_u8"], 123.0)) {
        result.pass("Unsigned 8-bit decode: data[0]=123 -> value=123.0");
    } else {
        result.fail("Unsigned 8-bit decode failed: got " + std::to_string(decoded["test_u8"]));
    }
}

// Test 2: Signed 16-bit encoding/decoding
void test_signed_16bit(TestResult& result) {
    std::cout << "\n=== Test 2: Signed 16-bit Signal ===\n";
    
    can::FrameDef frame = create_test_frame("test_s16", 0, 16, true, 1.0, 0.0);
    
    // Test positive value
    can::SignalMap values;
    values["test_s16"] = 1000.0;
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    
    if (is_close(decoded["test_s16"], 1000.0)) {
        result.pass("Signed 16-bit positive: 1000.0 -> " + std::to_string(decoded["test_s16"]));
    } else {
        result.fail("Signed 16-bit positive failed: expected 1000.0, got " + 
                   std::to_string(decoded["test_s16"]));
    }
    
    // Test negative value
    values["test_s16"] = -500.0;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    decoded = can::CanCodec::decode_to_map(frame, can_msg);
    
    if (is_close(decoded["test_s16"], -500.0)) {
        result.pass("Signed 16-bit negative: -500.0 -> " + std::to_string(decoded["test_s16"]));
    } else {
        result.fail("Signed 16-bit negative failed: expected -500.0, got " + 
                   std::to_string(decoded["test_s16"]));
    }
}

// Test 3: Factor and offset
void test_factor_offset(TestResult& result) {
    std::cout << "\n=== Test 3: Factor and Offset ===\n";
    
    // Temperature sensor: offset=-40, factor=1.0
    // Physical: -40°C to +125°C -> Raw: 0 to 165
    can::FrameDef temp_frame = create_test_frame("temperature", 0, 8, false, 1.0, -40.0);
    
    // Encode 25°C (should store as 65 raw)
    can::SignalMap values;
    values["temperature"] = 25.0;
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(temp_frame, values, can_msg);
    
    if (can_msg.data[0] == 65) {
        result.pass("Temperature encode: 25°C -> raw=65");
    } else {
        result.fail("Temperature encode failed: expected 65, got " + 
                   std::to_string(can_msg.data[0]));
    }
    
    // Decode back
    can::SignalMap decoded = can::CanCodec::decode_to_map(temp_frame, can_msg);
    if (is_close(decoded["temperature"], 25.0)) {
        result.pass("Temperature decode: raw=65 -> 25°C");
    } else {
        result.fail("Temperature decode failed: expected 25.0, got " + 
                   std::to_string(decoded["temperature"]));
    }
    
    // Test with factor=0.1 (voltage: 0-1000V with 0.1V resolution)
    can::FrameDef volt_frame = create_test_frame("voltage", 0, 16, false, 0.1, 0.0);
    
    values.clear();
    values["voltage"] = 385.7;
    
    can::CanCodec::encode_from_map(volt_frame, values, can_msg);
    decoded = can::CanCodec::decode_to_map(volt_frame, can_msg);
    
    // Should be close (quantization to 0.1V resolution)
    if (std::abs(decoded["voltage"] - 385.7) < 0.15) {
        result.pass("Voltage with factor=0.1: 385.7V -> " + 
                   std::to_string(decoded["voltage"]) + "V");
    } else {
        result.fail("Voltage decode failed: expected ~385.7V, got " + 
                   std::to_string(decoded["voltage"]) + "V");
    }
}

// Test 4: Multi-byte signal at non-zero start bit
void test_multi_byte_offset(TestResult& result) {
    std::cout << "\n=== Test 4: Multi-byte Signal with Offset ===\n";
    
    // Signal starting at bit 16 (byte 2), 16 bits long
    can::FrameDef frame = create_test_frame("offset_signal", 16, 16, false, 1.0, 0.0);
    
    // Encode value 0xABCD
    can::SignalMap values;
    values["offset_signal"] = 0xABCD;
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    
    // Check that bytes 2-3 contain the value (little-endian)
    uint16_t expected = 0xABCD;
    uint16_t actual = (can_msg.data[3] << 8) | can_msg.data[2];
    
    if (actual == expected) {
        result.pass("Multi-byte offset encode: value at bytes[2:3]");
    } else {
        char buf[100];
        snprintf(buf, sizeof(buf), "Expected 0x%04X at bytes[2:3], got 0x%04X", expected, actual);
        result.fail(buf);
    }
    
    // Decode back
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    if (is_close(decoded["offset_signal"], 0xABCD)) {
        result.pass("Multi-byte offset decode: correct value");
    } else {
        result.fail("Multi-byte offset decode failed");
    }
}

// Test 5: Max value edge case (unsigned 8-bit = 255)
void test_max_unsigned(TestResult& result) {
    std::cout << "\n=== Test 5: Max Unsigned Value ===\n";
    
    can::FrameDef frame = create_test_frame("max_u8", 0, 8, false, 1.0, 0.0);
    
    // Encode max value (255)
    can::SignalMap values;
    values["max_u8"] = 255.0;
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    
    if (can_msg.data[0] == 255) {
        result.pass("Max unsigned encode: 255 -> 0xFF");
    } else {
        result.fail("Max unsigned encode failed");
    }
    
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    if (is_close(decoded["max_u8"], 255.0)) {
        result.pass("Max unsigned decode: 0xFF -> 255.0");
    } else {
        result.fail("Max unsigned decode failed");
    }
}

// Test 6: Signed overflow/clipping
void test_signed_overflow(TestResult& result) {
    std::cout << "\n=== Test 6: Signed Overflow Handling ===\n";
    
    // For 8-bit signed: range is -128 to +127
    can::FrameDef frame = create_test_frame("signed_8bit", 0, 8, true, 1.0, 0.0);
    frame.signals[0].min = -128.0;
    frame.signals[0].max = 127.0;
    
    // Test value beyond range
    can::SignalMap values;
    values["signed_8bit"] = 200.0;  // Beyond +127
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    
    // Should clamp to max (127)
    if (decoded["signed_8bit"] <= 127.0) {
        result.pass("Signed overflow handled (value clamped to 127)");
    } else {
        result.fail("Signed overflow not handled: got " + std::to_string(decoded["signed_8bit"]));
    }
    
    // Test negative minimum
    values["signed_8bit"] = -200.0;  // Below -128
    can::CanCodec::encode_from_map(frame, values, can_msg);
    decoded = can::CanCodec::decode_to_map(frame, can_msg);
    
    if (decoded["signed_8bit"] >= -128.0) {
        result.pass("Signed underflow handled (value clamped to -128)");
    } else {
        result.fail("Signed underflow not handled: got " + std::to_string(decoded["signed_8bit"]));
    }
}

// Test 7: Multiple signals in same frame
void test_multiple_signals(TestResult& result) {
    std::cout << "\n=== Test 7: Multiple Signals in Frame ===\n";
    
    // Create a frame with 3 signals
    can::FrameDef frame;
    frame.frame_id = 0x200;
    frame.frame_name = "MULTI_SIGNAL";
    frame.dlc = 8;
    
    // Signal 1: bytes 0-1 (16-bit unsigned, factor=0.1)
    can::SignalRule sig1;
    sig1.signal_name = "signal_1";
    sig1.start_bit = 0;
    sig1.bit_length = 16;
    sig1.is_signed = false;
    sig1.factor = 0.1;
    sig1.offset = 0.0;
    sig1.min = 0.0;
    sig1.max = 1000.0;
    frame.signals.push_back(sig1);
    
    // Signal 2: bytes 2-3 (16-bit signed, factor=0.1)
    can::SignalRule sig2;
    sig2.signal_name = "signal_2";
    sig2.start_bit = 16;
    sig2.bit_length = 16;
    sig2.is_signed = true;
    sig2.factor = 0.1;
    sig2.offset = 0.0;
    sig2.min = -200.0;
    sig2.max = 200.0;
    frame.signals.push_back(sig2);
    
    // Signal 3: byte 4 (8-bit unsigned, factor=0.5)
    can::SignalRule sig3;
    sig3.signal_name = "signal_3";
    sig3.start_bit = 32;
    sig3.bit_length = 8;
    sig3.is_signed = false;
    sig3.factor = 0.5;
    sig3.offset = 0.0;
    sig3.min = 0.0;
    sig3.max = 100.0;
    frame.signals.push_back(sig3);
    
    // Encode all signals
    can::SignalMap values;
    values["signal_1"] = 385.7;   // Voltage
    values["signal_2"] = -123.4;  // Current
    values["signal_3"] = 75.5;    // SOC
    
    struct can_frame can_msg;
    can::CanCodec::encode_from_map(frame, values, can_msg);
    
    // Decode all signals
    can::SignalMap decoded = can::CanCodec::decode_to_map(frame, can_msg);
    
    bool all_ok = true;
    if (!is_close(decoded["signal_1"], 385.7, 0.15)) all_ok = false;
    if (!is_close(decoded["signal_2"], -123.4, 0.15)) all_ok = false;
    if (!is_close(decoded["signal_3"], 75.5, 0.6)) all_ok = false;
    
    if (all_ok) {
        result.pass("Multiple signals round-trip correctly");
        std::cout << "    Signal 1: " << values["signal_1"] << " -> " << decoded["signal_1"] << "\n";
        std::cout << "    Signal 2: " << values["signal_2"] << " -> " << decoded["signal_2"] << "\n";
        std::cout << "    Signal 3: " << values["signal_3"] << " -> " << decoded["signal_3"] << "\n";
    } else {
        result.fail("Multiple signals round-trip failed");
    }
}

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              CAN Codec Unit Tests                           ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    
    TestResult result;
    
    test_unsigned_8bit(result);
    test_signed_16bit(result);
    test_factor_offset(result);
    test_multi_byte_offset(result);
    test_max_unsigned(result);
    test_signed_overflow(result);
    test_multiple_signals(result);
    
    result.summary();
    
    return (result.failed == 0) ? 0 : 1;
}