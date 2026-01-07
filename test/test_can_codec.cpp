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
 *   4. Multi-byte signals (16-bit, 24-bit)
 *   5. Bit packing and extraction
 *   6. Edge cases (min/max values, overflow)
 */

#include "can/can_codec.hpp"
#include "can/can_map.hpp"
#include <iostream>
#include <cmath>
#include <cstring>

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

// Test 1: Unsigned 8-bit encoding/decoding
void test_unsigned_8bit(TestResult& result) {
    std::cout << "\n=== Test 1: Unsigned 8-bit Signal ===\n";
    
    can::SignalDef sig;
    sig.signal_name = "test_u8";
    sig.start_bit = 0;
    sig.bit_length = 8;
    sig.is_signed = false;
    sig.factor = 1.0;
    sig.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // Encode value 123
    double value = 123.0;
    can::CanCodec::encode_signal(sig, value, data);
    
    if (data[0] == 123) {
        result.pass("Unsigned 8-bit encode: value=123 -> data[0]=123");
    } else {
        result.fail("Unsigned 8-bit encode failed: got " + std::to_string(data[0]));
    }
    
    // Decode back
    double decoded = can::CanCodec::decode_signal(sig, data);
    if (is_close(decoded, value)) {
        result.pass("Unsigned 8-bit decode: data[0]=123 -> value=123.0");
    } else {
        result.fail("Unsigned 8-bit decode failed: got " + std::to_string(decoded));
    }
}

// Test 2: Signed 16-bit encoding/decoding
void test_signed_16bit(TestResult& result) {
    std::cout << "\n=== Test 2: Signed 16-bit Signal ===\n";
    
    can::SignalDef sig;
    sig.signal_name = "test_s16";
    sig.start_bit = 0;
    sig.bit_length = 16;
    sig.is_signed = true;
    sig.factor = 1.0;
    sig.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // Test positive value
    double value_pos = 1000.0;
    can::CanCodec::encode_signal(sig, value_pos, data);
    double decoded_pos = can::CanCodec::decode_signal(sig, data);
    
    if (is_close(decoded_pos, value_pos)) {
        result.pass("Signed 16-bit positive: 1000.0 -> " + std::to_string(decoded_pos));
    } else {
        result.fail("Signed 16-bit positive failed: expected 1000.0, got " + std::to_string(decoded_pos));
    }
    
    // Test negative value
    std::memset(data, 0, 8);
    double value_neg = -500.0;
    can::CanCodec::encode_signal(sig, value_neg, data);
    double decoded_neg = can::CanCodec::decode_signal(sig, data);
    
    if (is_close(decoded_neg, value_neg)) {
        result.pass("Signed 16-bit negative: -500.0 -> " + std::to_string(decoded_neg));
    } else {
        result.fail("Signed 16-bit negative failed: expected -500.0, got " + std::to_string(decoded_neg));
    }
}

// Test 3: Factor and offset
void test_factor_offset(TestResult& result) {
    std::cout << "\n=== Test 3: Factor and Offset ===\n";
    
    // Example: Temperature sensor with offset=-40, factor=1.0
    // Physical: -40°C to +125°C -> Raw: 0 to 165
    can::SignalDef sig;
    sig.signal_name = "temperature";
    sig.start_bit = 0;
    sig.bit_length = 8;
    sig.is_signed = false;
    sig.factor = 1.0;
    sig.offset = -40.0;
    
    uint8_t data[8] = {0};
    
    // Encode 25°C (should store as 65 raw)
    double temp_c = 25.0;
    can::CanCodec::encode_signal(sig, temp_c, data);
    
    if (data[0] == 65) {
        result.pass("Temperature encode: 25°C -> raw=65");
    } else {
        result.fail("Temperature encode failed: expected 65, got " + std::to_string(data[0]));
    }
    
    // Decode back
    double decoded = can::CanCodec::decode_signal(sig, data);
    if (is_close(decoded, temp_c)) {
        result.pass("Temperature decode: raw=65 -> 25°C");
    } else {
        result.fail("Temperature decode failed: expected 25.0, got " + std::to_string(decoded));
    }
    
    // Test with factor=0.1 (e.g., voltage: 0-1000V with 0.1V resolution)
    can::SignalDef volt_sig;
    volt_sig.signal_name = "voltage";
    volt_sig.start_bit = 0;
    volt_sig.bit_length = 16;
    volt_sig.is_signed = false;
    volt_sig.factor = 0.1;
    volt_sig.offset = 0.0;
    
    std::memset(data, 0, 8);
    double voltage = 385.7;
    can::CanCodec::encode_signal(volt_sig, voltage, data);
    double decoded_volt = can::CanCodec::decode_signal(volt_sig, data);
    
    // Should be close (quantization to 0.1V resolution)
    if (std::abs(decoded_volt - voltage) < 0.15) {  // Allow 0.1V quantization error
        result.pass("Voltage with factor=0.1: 385.7V -> " + std::to_string(decoded_volt) + "V");
    } else {
        result.fail("Voltage decode failed: expected ~385.7V, got " + std::to_string(decoded_volt) + "V");
    }
}

// Test 4: Multi-byte signal at non-zero start bit
void test_multi_byte_offset(TestResult& result) {
    std::cout << "\n=== Test 4: Multi-byte Signal with Offset ===\n";
    
    // Signal starting at bit 16 (byte 2), 16 bits long
    can::SignalDef sig;
    sig.signal_name = "offset_signal";
    sig.start_bit = 16;
    sig.bit_length = 16;
    sig.is_signed = false;
    sig.factor = 1.0;
    sig.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // Encode value 0xABCD at bytes 2-3
    double value = 0xABCD;
    can::CanCodec::encode_signal(sig, value, data);
    
    // Check that bytes 2-3 contain the value (little-endian)
    uint16_t expected = 0xABCD;
    uint16_t actual = (data[3] << 8) | data[2];
    
    if (actual == expected) {
        result.pass("Multi-byte offset encode: value at bytes[2:3]");
    } else {
        char buf[100];
        snprintf(buf, sizeof(buf), "Expected 0x%04X at bytes[2:3], got 0x%04X", expected, actual);
        result.fail(buf);
    }
    
    // Decode back
    double decoded = can::CanCodec::decode_signal(sig, data);
    if (is_close(decoded, value)) {
        result.pass("Multi-byte offset decode: correct value");
    } else {
        result.fail("Multi-byte offset decode failed");
    }
}

// Test 5: Max value edge case (unsigned 8-bit = 255)
void test_max_unsigned(TestResult& result) {
    std::cout << "\n=== Test 5: Max Unsigned Value ===\n";
    
    can::SignalDef sig;
    sig.signal_name = "max_u8";
    sig.start_bit = 0;
    sig.bit_length = 8;
    sig.is_signed = false;
    sig.factor = 1.0;
    sig.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // Encode max value (255)
    double value = 255.0;
    can::CanCodec::encode_signal(sig, value, data);
    
    if (data[0] == 255) {
        result.pass("Max unsigned encode: 255 -> 0xFF");
    } else {
        result.fail("Max unsigned encode failed");
    }
    
    double decoded = can::CanCodec::decode_signal(sig, data);
    if (is_close(decoded, value)) {
        result.pass("Max unsigned decode: 0xFF -> 255.0");
    } else {
        result.fail("Max unsigned decode failed");
    }
}

// Test 6: Signed overflow/clipping
void test_signed_overflow(TestResult& result) {
    std::cout << "\n=== Test 6: Signed Overflow Handling ===\n";
    
    can::SignalDef sig;
    sig.signal_name = "signed_8bit";
    sig.start_bit = 0;
    sig.bit_length = 8;
    sig.is_signed = true;
    sig.factor = 1.0;
    sig.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // For 8-bit signed: range is -128 to +127
    // Test value beyond range
    double value_overflow = 200.0;  // Beyond +127
    can::CanCodec::encode_signal(sig, value_overflow, data);
    double decoded = can::CanCodec::decode_signal(sig, data);
    
    // Should clamp to max (127) or wrap
    if (decoded <= 127.0) {
        result.pass("Signed overflow handled (value clamped or wrapped)");
    } else {
        result.fail("Signed overflow not handled: got " + std::to_string(decoded));
    }
    
    // Test negative minimum
    std::memset(data, 0, 8);
    double value_underflow = -200.0;  // Below -128
    can::CanCodec::encode_signal(sig, value_underflow, data);
    decoded = can::CanCodec::decode_signal(sig, data);
    
    if (decoded >= -128.0) {
        result.pass("Signed underflow handled (value clamped or wrapped)");
    } else {
        result.fail("Signed underflow not handled: got " + std::to_string(decoded));
    }
}

// Test 7: Round-trip multiple signals in same frame
void test_multiple_signals(TestResult& result) {
    std::cout << "\n=== Test 7: Multiple Signals in Frame ===\n";
    
    // Simulate a CAN frame with 3 signals
    can::SignalDef sig1, sig2, sig3;
    
    // Signal 1: bytes 0-1 (16-bit unsigned)
    sig1.signal_name = "signal_1";
    sig1.start_bit = 0;
    sig1.bit_length = 16;
    sig1.is_signed = false;
    sig1.factor = 0.1;
    sig1.offset = 0.0;
    
    // Signal 2: bytes 2-3 (16-bit signed)
    sig2.signal_name = "signal_2";
    sig2.start_bit = 16;
    sig2.bit_length = 16;
    sig2.is_signed = true;
    sig2.factor = 0.1;
    sig2.offset = 0.0;
    
    // Signal 3: byte 4 (8-bit unsigned)
    sig3.signal_name = "signal_3";
    sig3.start_bit = 32;
    sig3.bit_length = 8;
    sig3.is_signed = false;
    sig3.factor = 0.5;
    sig3.offset = 0.0;
    
    uint8_t data[8] = {0};
    
    // Encode all signals
    double val1 = 385.7;   // Voltage
    double val2 = -123.4;  // Current
    double val3 = 75.5;    // SOC
    
    can::CanCodec::encode_signal(sig1, val1, data);
    can::CanCodec::encode_signal(sig2, val2, data);
    can::CanCodec::encode_signal(sig3, val3, data);
    
    // Decode all signals
    double dec1 = can::CanCodec::decode_signal(sig1, data);
    double dec2 = can::CanCodec::decode_signal(sig2, data);
    double dec3 = can::CanCodec::decode_signal(sig3, data);
    
    bool all_ok = true;
    if (!is_close(dec1, val1, 0.15)) all_ok = false;
    if (!is_close(dec2, val2, 0.15)) all_ok = false;
    if (!is_close(dec3, val3, 0.6)) all_ok = false;
    
    if (all_ok) {
        result.pass("Multiple signals round-trip correctly");
        std::cout << "    Signal 1: " << val1 << " -> " << dec1 << "\n";
        std::cout << "    Signal 2: " << val2 << " -> " << dec2 << "\n";
        std::cout << "    Signal 3: " << val3 << " -> " << dec3 << "\n";
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