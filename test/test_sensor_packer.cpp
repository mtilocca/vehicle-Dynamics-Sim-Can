// test/test_sensor_packer.cpp
/**
 * Unit Test: Sensor State Packer
 * 
 * Tests sensor data packing into CAN frames.
 * 
 * Test Coverage:
 *   1. Battery sensor packing (voltage, current, SOC, temp, power)
 *   2. Wheel speed sensor packing (4 wheels)
 *   3. Signal value ranges and quantization
 *   4. Derived signals (power calculation)
 *   5. Data integrity (no overflow/corruption)
 */

#include "can/sensor_state_packer.hpp"
#include "sensors/sensor_out.hpp"
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

// Helper: Extract uint16 from little-endian bytes
uint16_t extract_u16(const uint8_t* data, int offset) {
    return (data[offset + 1] << 8) | data[offset];
}

// Helper: Extract int16 from little-endian bytes
int16_t extract_s16(const uint8_t* data, int offset) {
    uint16_t raw = extract_u16(data, offset);
    return static_cast<int16_t>(raw);
}

// Test 1: Battery voltage packing
void test_battery_voltage(TestResult& result) {
    std::cout << "\n=== Test 1: Battery Voltage Packing ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 385.7;  // 385.7V
    sens.batt_i_meas = 0.0;
    sens.batt_soc_meas = 0.0;
    sens.batt_temp_meas = 0.0;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // Voltage: start_bit=0, 16-bit unsigned, factor=0.1
    // Expected raw: 385.7 / 0.1 = 3857
    uint16_t raw_voltage = extract_u16(data, 0);
    double decoded_voltage = raw_voltage * 0.1;
    
    if (is_close(decoded_voltage, 385.7, 0.15)) {
        result.pass("Battery voltage: 385.7V -> raw=" + std::to_string(raw_voltage) + 
                   " -> " + std::to_string(decoded_voltage) + "V");
    } else {
        result.fail("Battery voltage mismatch: expected ~385.7V, got " + 
                   std::to_string(decoded_voltage) + "V");
    }
}

// Test 2: Battery current (signed) packing
void test_battery_current(TestResult& result) {
    std::cout << "\n=== Test 2: Battery Current Packing ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 0.0;
    sens.batt_i_meas = -150.3;  // Negative = discharging
    sens.batt_soc_meas = 0.0;
    sens.batt_temp_meas = 0.0;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // Current: start_bit=16, 16-bit signed, factor=0.1
    // Expected raw: -150.3 / 0.1 = -1503
    int16_t raw_current = extract_s16(data, 2);
    double decoded_current = raw_current * 0.1;
    
    if (is_close(decoded_current, -150.3, 0.15)) {
        result.pass("Battery current: -150.3A -> raw=" + std::to_string(raw_current) + 
                   " -> " + std::to_string(decoded_current) + "A");
    } else {
        result.fail("Battery current mismatch: expected ~-150.3A, got " + 
                   std::to_string(decoded_current) + "A");
    }
    
    // Test positive current (charging)
    std::memset(data, 0, 8);
    sens.batt_i_meas = 75.6;
    can::SensorStatePacker::pack_battery(sens, data);
    
    raw_current = extract_s16(data, 2);
    decoded_current = raw_current * 0.1;
    
    if (is_close(decoded_current, 75.6, 0.15)) {
        result.pass("Battery current (charging): 75.6A -> " + std::to_string(decoded_current) + "A");
    } else {
        result.fail("Battery current (charging) mismatch");
    }
}

// Test 3: Battery SOC packing
void test_battery_soc(TestResult& result) {
    std::cout << "\n=== Test 3: Battery SOC Packing ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 0.0;
    sens.batt_i_meas = 0.0;
    sens.batt_soc_meas = 75.5;  // 75.5%
    sens.batt_temp_meas = 0.0;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // SOC: start_bit=32 (byte 4), 8-bit unsigned, factor=0.5
    // Expected raw: 75.5 / 0.5 = 151
    uint8_t raw_soc = data[4];
    double decoded_soc = raw_soc * 0.5;
    
    if (is_close(decoded_soc, 75.5, 0.6)) {
        result.pass("Battery SOC: 75.5% -> raw=" + std::to_string(raw_soc) + 
                   " -> " + std::to_string(decoded_soc) + "%");
    } else {
        result.fail("Battery SOC mismatch: expected ~75.5%, got " + 
                   std::to_string(decoded_soc) + "%");
    }
}

// Test 4: Battery temperature with offset
void test_battery_temperature(TestResult& result) {
    std::cout << "\n=== Test 4: Battery Temperature Packing ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 0.0;
    sens.batt_i_meas = 0.0;
    sens.batt_soc_meas = 0.0;
    sens.batt_temp_meas = 25.0;  // 25°C
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // Temp: start_bit=40 (byte 5), 8-bit unsigned, factor=1, offset=-40
    // Physical range: -40°C to +125°C
    // Expected raw: 25 - (-40) = 65
    uint8_t raw_temp = data[5];
    double decoded_temp = raw_temp + (-40.0);
    
    if (is_close(decoded_temp, 25.0, 1.1)) {
        result.pass("Battery temp: 25°C -> raw=" + std::to_string(raw_temp) + 
                   " -> " + std::to_string(decoded_temp) + "°C");
    } else {
        result.fail("Battery temp mismatch: expected ~25°C, got " + 
                   std::to_string(decoded_temp) + "°C");
    }
    
    // Test negative temperature
    std::memset(data, 0, 8);
    sens.batt_temp_meas = -10.0;  // -10°C
    can::SensorStatePacker::pack_battery(sens, data);
    
    raw_temp = data[5];
    decoded_temp = raw_temp + (-40.0);
    
    if (is_close(decoded_temp, -10.0, 1.1)) {
        result.pass("Battery temp (cold): -10°C -> raw=" + std::to_string(raw_temp) + 
                   " -> " + std::to_string(decoded_temp) + "°C");
    } else {
        result.fail("Battery temp (cold) mismatch");
    }
}

// Test 5: Battery power calculation (derived signal)
void test_battery_power(TestResult& result) {
    std::cout << "\n=== Test 5: Battery Power Calculation ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 385.0;   // 385V
    sens.batt_i_meas = 150.0;   // 150A
    sens.batt_soc_meas = 0.0;
    sens.batt_temp_meas = 0.0;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // Power: start_bit=48 (bytes 6-7), 16-bit signed, factor=0.1
    // Expected: 385V * 150A / 1000 = 57.75 kW
    // Raw: 57.75 / 0.1 = 577.5 ≈ 578
    int16_t raw_power = extract_s16(data, 6);
    double decoded_power = raw_power * 0.1;
    double expected_power = (sens.batt_v_meas * sens.batt_i_meas) / 1000.0;
    
    if (is_close(decoded_power, expected_power, 0.15)) {
        result.pass("Battery power: V=" + std::to_string(sens.batt_v_meas) + 
                   "V, I=" + std::to_string(sens.batt_i_meas) + 
                   "A -> P=" + std::to_string(decoded_power) + "kW");
    } else {
        result.fail("Battery power mismatch: expected ~" + std::to_string(expected_power) + 
                   "kW, got " + std::to_string(decoded_power) + "kW");
    }
}

// Test 6: Wheel speeds packing (4 wheels)
void test_wheel_speeds(TestResult& result) {
    std::cout << "\n=== Test 6: Wheel Speeds Packing ===\n";
    
    sensors::SensorOut sens;
    sens.wheel_fl_rps_meas = 25.3;   // Front-left
    sens.wheel_fr_rps_meas = 25.1;   // Front-right
    sens.wheel_rl_rps_meas = 24.9;   // Rear-left
    sens.wheel_rr_rps_meas = 25.0;   // Rear-right
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_wheel_speeds(sens, data);
    
    // All wheels: factor=0.01, signed 16-bit
    int16_t raw_fl = extract_s16(data, 0);
    int16_t raw_fr = extract_s16(data, 2);
    int16_t raw_rl = extract_s16(data, 4);
    int16_t raw_rr = extract_s16(data, 6);
    
    double dec_fl = raw_fl * 0.01;
    double dec_fr = raw_fr * 0.01;
    double dec_rl = raw_rl * 0.01;
    double dec_rr = raw_rr * 0.01;
    
    bool all_ok = true;
    if (!is_close(dec_fl, 25.3, 0.015)) all_ok = false;
    if (!is_close(dec_fr, 25.1, 0.015)) all_ok = false;
    if (!is_close(dec_rl, 24.9, 0.015)) all_ok = false;
    if (!is_close(dec_rr, 25.0, 0.015)) all_ok = false;
    
    if (all_ok) {
        result.pass("Wheel speeds packed correctly:");
        std::cout << "    FL: " << sens.wheel_fl_rps_meas << " -> " << dec_fl << " rad/s\n";
        std::cout << "    FR: " << sens.wheel_fr_rps_meas << " -> " << dec_fr << " rad/s\n";
        std::cout << "    RL: " << sens.wheel_rl_rps_meas << " -> " << dec_rl << " rad/s\n";
        std::cout << "    RR: " << sens.wheel_rr_rps_meas << " -> " << dec_rr << " rad/s\n";
    } else {
        result.fail("Wheel speeds packing failed");
    }
}

// Test 7: Negative wheel speeds (reverse)
void test_wheel_speeds_reverse(TestResult& result) {
    std::cout << "\n=== Test 7: Wheel Speeds (Reverse) ===\n";
    
    sensors::SensorOut sens;
    sens.wheel_fl_rps_meas = -10.5;
    sens.wheel_fr_rps_meas = -10.3;
    sens.wheel_rl_rps_meas = -10.6;
    sens.wheel_rr_rps_meas = -10.4;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_wheel_speeds(sens, data);
    
    int16_t raw_fl = extract_s16(data, 0);
    double dec_fl = raw_fl * 0.01;
    
    if (is_close(dec_fl, -10.5, 0.015)) {
        result.pass("Reverse wheel speed: -10.5 rad/s -> " + std::to_string(dec_fl) + " rad/s");
    } else {
        result.fail("Reverse wheel speed mismatch: expected ~-10.5, got " + 
                   std::to_string(dec_fl));
    }
}

// Test 8: Full battery frame integrity
void test_battery_frame_integrity(TestResult& result) {
    std::cout << "\n=== Test 8: Battery Frame Integrity ===\n";
    
    sensors::SensorOut sens;
    sens.batt_v_meas = 400.0;
    sens.batt_i_meas = 200.0;
    sens.batt_soc_meas = 85.0;
    sens.batt_temp_meas = 30.0;
    
    uint8_t data[8] = {0};
    can::SensorStatePacker::pack_battery(sens, data);
    
    // Decode all signals
    uint16_t raw_v = extract_u16(data, 0);
    int16_t raw_i = extract_s16(data, 2);
    uint8_t raw_soc = data[4];
    uint8_t raw_temp = data[5];
    int16_t raw_power = extract_s16(data, 6);
    
    double voltage = raw_v * 0.1;
    double current = raw_i * 0.1;
    double soc = raw_soc * 0.5;
    double temp = raw_temp + (-40.0);
    double power = raw_power * 0.1;
    
    bool integrity_ok = true;
    if (!is_close(voltage, 400.0, 0.15)) integrity_ok = false;
    if (!is_close(current, 200.0, 0.15)) integrity_ok = false;
    if (!is_close(soc, 85.0, 0.6)) integrity_ok = false;
    if (!is_close(temp, 30.0, 1.1)) integrity_ok = false;
    if (!is_close(power, 80.0, 0.15)) integrity_ok = false;  // 400V * 200A / 1000 = 80kW
    
    if (integrity_ok) {
        result.pass("All battery signals packed without corruption");
    } else {
        result.fail("Battery frame integrity check failed");
    }
}

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           Sensor State Packer Unit Tests                    ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    
    TestResult result;
    
    test_battery_voltage(result);
    test_battery_current(result);
    test_battery_soc(result);
    test_battery_temperature(result);
    test_battery_power(result);
    test_wheel_speeds(result);
    test_wheel_speeds_reverse(result);
    test_battery_frame_integrity(result);
    
    result.summary();
    
    return (result.failed == 0) ? 0 : 1;
}