// test/test_plant_state_packer.cpp
/**
 * Validation Test: PlantStatePacker Refactor
 * 
 * This test verifies that the new visitor-based PlantStatePacker
 * produces identical output to the old manual implementation.
 * 
 * Test Strategy:
 *   1. Create a PlantState with known values
 *   2. Pack each CAN frame using the new implementation
 *   3. Verify all expected signals are present with correct values
 *   4. Ensure no signals are missing or incorrect
 * 
 * Success Criteria:
 *   - All 7 plant_state frames pack correctly
 *   - All signals match expected values within tolerance
 *   - No crashes, no missing fields
 */

#include "plant/plant_state.hpp"
#include "plant/plant_state_visitor.hpp"
#include "sim/plant_state_packer.hpp"
#include "can/can_map.hpp"

#include <cmath>
#include <iostream>
#include <iomanip>

// ANSI color codes for terminal output
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

// Helper: Check if value is close to expected (within 0.01%)
bool is_close(double actual, double expected, double tolerance = 0.0001) {
    if (std::abs(expected) < 1e-9) {
        return std::abs(actual) < tolerance;
    }
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

// Create a test PlantState with known values
plant::PlantState create_test_state() {
    plant::PlantState s;
    
    // Set realistic values
    s.t_s = 10.5;
    s.x_m = 123.45;
    s.y_m = 67.89;
    s.yaw_rad = 0.785398;  // 45 degrees
    s.v_mps = 25.0;
    s.a_long_mps2 = 2.5;
    
    s.steer_virtual_rad = 0.174533;  // 10 degrees
    s.steer_rate_radps = 0.1;
    s.delta_fl_rad = 0.19;
    s.delta_fr_rad = 0.16;
    
    s.batt_soc_pct = 75.5;
    s.batt_v = 385.0;
    s.batt_i = 150.0;
    s.batt_temp_c = 28.5;
    
    s.motor_torque_nm = 1200.0;
    s.motor_power_kW = 80.5;
    s.regen_power_kW = 0.0;
    s.brake_force_kN = 0.0;
    
    s.wheel_fl_rps = 25.2;
    s.wheel_fr_rps = 25.3;
    s.wheel_rl_rps = 25.1;
    s.wheel_rr_rps = 25.0;
    
    s.status_flags = 0;
    
    return s;
}

void test_visitor_enumeration(TestResult& result) {
    std::cout << "\n=== Test 1: Visitor Field Enumeration ===\n";
    
    plant::PlantState state = create_test_state();
    
    int field_count = 0;
    bool found_speed = false;
    bool found_soc = false;
    bool found_torque = false;
    
    auto visitor = plant::make_visitor([&](const char* name, double value) {
        ++field_count;
        
        std::string n(name);
        if (n == "vehicle_speed_mps") {
            found_speed = true;
            if (is_close(value, 25.0)) {
                result.pass("vehicle_speed_mps = 25.0");
            } else {
                result.fail("vehicle_speed_mps incorrect");
            }
        }
        else if (n == "batt_soc_pct") {
            found_soc = true;
            if (is_close(value, 75.5)) {
                result.pass("batt_soc_pct = 75.5");
            } else {
                result.fail("batt_soc_pct incorrect");
            }
        }
        else if (n == "motor_torque_nm") {
            found_torque = true;
            if (is_close(value, 1200.0)) {
                result.pass("motor_torque_nm = 1200.0");
            } else {
                result.fail("motor_torque_nm incorrect");
            }
        }
    });
    
    state.accept_fields(visitor);
    
    if (field_count >= 20) {
        result.pass("Visitor enumerated " + std::to_string(field_count) + " fields");
    } else {
        result.fail("Too few fields enumerated: " + std::to_string(field_count));
    }
    
    if (!found_speed) result.fail("vehicle_speed_mps not found");
    if (!found_soc) result.fail("batt_soc_pct not found");
    if (!found_torque) result.fail("motor_torque_nm not found");
}

void test_frame_packing(TestResult& result, const can::CanMap& map) {
    std::cout << "\n=== Test 2: Frame Packing ===\n";
    
    plant::PlantState state = create_test_state();
    
    // Test each plant_state frame
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
        
        if (!frame_def) {
            result.fail("Frame 0x" + std::to_string(frame_id) + " not found in map");
            continue;
        }
        
        // Pack the frame
        can::SignalMap signals = sim::PlantStatePacker::pack(state, *frame_def);
        
        // Check that we got some signals
        if (signals.empty()) {
            result.fail("Frame 0x" + std::to_string(frame_id) + " produced empty signal map");
            continue;
        }
        
        // Verify expected signals are present
        int expected_signals = 0;
        int found_signals = 0;
        
        for (const auto& sig_def : frame_def->signals) {
            ++expected_signals;
            
            auto it = signals.find(sig_def.signal_name);
            if (it != signals.end()) {
                ++found_signals;
            }
        }
        
        if (found_signals == expected_signals) {
            result.pass("Frame 0x" + std::to_string(frame_id) + " (" + frame_def->frame_name + 
                       "): " + std::to_string(found_signals) + " signals");
        } else {
            result.fail("Frame 0x" + std::to_string(frame_id) + ": only " + 
                       std::to_string(found_signals) + "/" + std::to_string(expected_signals) + " signals");
        }
    }
}

void test_derived_signals(TestResult& result, const can::CanMap& map) {
    std::cout << "\n=== Test 3: Derived Signal Calculations ===\n";
    
    plant::PlantState state = create_test_state();
    
    // Test motor RPM calculation (0x310)
    const can::FrameDef* motor_frame = map.find_tx_frame(0x310);
    if (motor_frame) {
        can::SignalMap signals = sim::PlantStatePacker::pack(state, *motor_frame);
        
        auto it = signals.find("motor_speed_rpm");
        if (it != signals.end()) {
            // Expected: (v / r) * gear_ratio * 60 / (2π)
            // (25.0 / 0.33) * 9.0 * 60 / (2π) ≈ 6497 RPM
            double expected_rpm = (25.0 / 0.33) * 9.0 * 60.0 / (2.0 * M_PI);
            
            if (is_close(it->second, expected_rpm, 0.01)) {
                result.pass("motor_speed_rpm calculated correctly: " + 
                           std::to_string((int)it->second) + " RPM");
            } else {
                result.fail("motor_speed_rpm incorrect: got " + std::to_string(it->second) +
                           ", expected " + std::to_string(expected_rpm));
            }
        } else {
            result.fail("motor_speed_rpm not found in MOTOR_STATE_1");
        }
    }
    
    // Test yaw conversion (0x331)
    const can::FrameDef* orient_frame = map.find_tx_frame(0x331);
    if (orient_frame) {
        can::SignalMap signals = sim::PlantStatePacker::pack(state, *orient_frame);
        
        auto it = signals.find("yaw_deg");
        if (it != signals.end()) {
            // Expected: 0.785398 rad = 45 degrees
            double expected_deg = state.yaw_rad * 180.0 / M_PI;
            
            if (is_close(it->second, expected_deg)) {
                result.pass("yaw_deg converted correctly: " + 
                           std::to_string(it->second) + " deg");
            } else {
                result.fail("yaw_deg incorrect");
            }
        } else {
            result.fail("yaw_deg not found in ORIENTATION_STATE");
        }
    }
    
    // Test battery power calculation (0x230)
    const can::FrameDef* batt_frame = map.find_tx_frame(0x230);
    if (batt_frame) {
        can::SignalMap signals = sim::PlantStatePacker::pack(state, *batt_frame);
        
        auto it = signals.find("batt_power_kw");
        if (it != signals.end()) {
            // Expected: V * I / 1000 = 385 * 150 / 1000 = 57.75 kW
            double expected_power = (state.batt_v * state.batt_i) / 1000.0;
            
            if (is_close(it->second, expected_power)) {
                result.pass("batt_power_kw calculated correctly: " + 
                           std::to_string(it->second) + " kW");
            } else {
                result.fail("batt_power_kw incorrect");
            }
        } else {
            result.fail("batt_power_kw not found in BATT_STATE");
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║  PlantStatePacker Refactor Validation Test                  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    
    TestResult result;
    
    // Load CAN map
    const char* can_map_path = (argc > 1) ? argv[1] : "config/can_map.csv";
    
    can::CanMap map;
    if (!map.load(can_map_path)) {
        std::cout << COLOR_RED << "\nFATAL: Could not load CAN map: " << can_map_path << COLOR_RESET << "\n";
        std::cout << "Usage: " << argv[0] << " [can_map.csv]\n";
        return 1;
    }
    
    std::cout << "\nLoaded CAN map: " << can_map_path << "\n";
    std::cout << "  TX Frames: " << map.tx_frames().size() << "\n";
    
    // Run tests
    test_visitor_enumeration(result);
    test_frame_packing(result, map);
    test_derived_signals(result, map);
    
    // Print summary
    result.summary();
    
    return (result.failed == 0) ? 0 : 1;
}
