// test/test_vehicle_config.cpp
/**
 * Unit Test: VehicleConfig
 * 
 * Tests YAML loading, validation, and default configuration.
 * 
 * Test Coverage:
 *   1. Default configuration generation
 *   2. Valid YAML loading
 *   3. Invalid YAML handling
 *   4. Parameter validation (mass, power, battery, etc.)
 *   5. Missing file fallback to defaults
 */

#include "config/vehicle_config.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

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

// Helper: Check if value is close to expected
bool is_close(double actual, double expected, double tolerance = 0.0001) {
    if (std::abs(expected) < 1e-9) {
        return std::abs(actual) < tolerance;
    }
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

// Test 1: Default configuration
void test_default_config(TestResult& result) {
    std::cout << "\n=== Test 1: Default Configuration ===\n";
    
    config::VehicleConfig cfg = config::VehicleConfig::get_default();
    
    // Check that defaults are reasonable
    if (cfg.params.drive.mass_kg > 0.0) {
        result.pass("Default mass_kg is positive: " + std::to_string(cfg.params.drive.mass_kg) + " kg");
    } else {
        result.fail("Default mass_kg is invalid");
    }
    
    if (cfg.params.drive.motor_power_max_w > 0.0) {
        result.pass("Default motor power is positive: " + 
                   std::to_string(cfg.params.drive.motor_power_max_w / 1000.0) + " kW");
    } else {
        result.fail("Default motor power is invalid");
    }
    
    if (cfg.params.battery_params.capacity_kWh > 0.0) {
        result.pass("Default battery capacity is positive: " + 
                   std::to_string(cfg.params.battery_params.capacity_kWh) + " kWh");
    } else {
        result.fail("Default battery capacity is invalid");
    }
    
    if (cfg.params.wheelbase_m > 0.0 && cfg.params.track_width_m > 0.0) {
        result.pass("Default geometry is valid (wheelbase=" + 
                   std::to_string(cfg.params.wheelbase_m) + "m, track=" + 
                   std::to_string(cfg.params.track_width_m) + "m)");
    } else {
        result.fail("Default geometry is invalid");
    }
}

// Test 2: Valid YAML loading
void test_valid_yaml(TestResult& result) {
    std::cout << "\n=== Test 2: Valid YAML Loading ===\n";
    
    // Create a temporary valid YAML file
    const char* temp_yaml = "/tmp/test_vehicle_valid.yaml";
    std::ofstream yaml_file(temp_yaml);
    yaml_file << R"(
vehicle:
  name: "Test Vehicle"
  description: "Unit test vehicle"
  manufacturer: "TestCo"
  year: 2025
  
  geometry:
    mass_kg: 2000.0
    wheelbase_m: 3.0
    track_width_m: 1.7
    wheel_radius_m: 0.35
  
  drivetrain:
    motor_power_max_w: 200000.0
    motor_torque_max_nm: 2500.0
    gear_ratio: 10.0
    drivetrain_eff: 0.95
  
  battery:
    capacity_kWh: 100.0
    nominal_voltage: 400.0
    initial_soc: 0.80
    max_power_kw: 150.0
    efficiency_charge: 0.95
    efficiency_discharge: 0.95
    min_soc: 0.05
    max_soc: 0.95
  
  resistance:
    drag_coefficient: 0.30
    rolling_resistance: 35.0
  
  limits:
    v_max_mps: 50.0
    v_stop_eps: 0.3
)";
    yaml_file.close();
    
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);
        
        // Verify loaded values
        if (cfg.name == "Test Vehicle") {
            result.pass("Vehicle name loaded correctly: " + cfg.name);
        } else {
            result.fail("Vehicle name mismatch");
        }
        
        if (is_close(cfg.params.drive.mass_kg, 2000.0)) {
            result.pass("Mass loaded correctly: " + std::to_string(cfg.params.drive.mass_kg) + " kg");
        } else {
            result.fail("Mass mismatch");
        }
        
        if (is_close(cfg.params.drive.motor_power_max_w, 200000.0)) {
            result.pass("Motor power loaded correctly: " + 
                       std::to_string(cfg.params.drive.motor_power_max_w / 1000.0) + " kW");
        } else {
            result.fail("Motor power mismatch");
        }
        
        if (is_close(cfg.params.battery_params.capacity_kWh, 100.0)) {
            result.pass("Battery capacity loaded correctly: " + 
                       std::to_string(cfg.params.battery_params.capacity_kWh) + " kWh");
        } else {
            result.fail("Battery capacity mismatch");
        }
        
        if (is_close(cfg.params.wheelbase_m, 3.0)) {
            result.pass("Wheelbase loaded correctly: " + std::to_string(cfg.params.wheelbase_m) + " m");
        } else {
            result.fail("Wheelbase mismatch");
        }
        
    } catch (const std::exception& e) {
        result.fail(std::string("Exception during load: ") + e.what());
    }
    
    // Clean up
    std::remove(temp_yaml);
}

// Test 3: Missing file fallback
void test_missing_file(TestResult& result) {
    std::cout << "\n=== Test 3: Missing File Fallback ===\n";
    
    // Try to load a non-existent file
    const char* missing_file = "/tmp/nonexistent_vehicle_config.yaml";
    
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(missing_file);
        
        // Should fall back to defaults
        if (cfg.params.drive.mass_kg > 0.0 && cfg.params.battery_params.capacity_kWh > 0.0) {
            result.pass("Missing file correctly fell back to defaults");
        } else {
            result.fail("Fallback defaults are invalid");
        }
        
    } catch (const std::exception& e) {
        result.fail(std::string("Unexpected exception: ") + e.what());
    }
}

// Test 4: Invalid YAML - negative mass
void test_invalid_mass(TestResult& result) {
    std::cout << "\n=== Test 4: Validation - Negative Mass ===\n";
    
    const char* temp_yaml = "/tmp/test_vehicle_invalid_mass.yaml";
    std::ofstream yaml_file(temp_yaml);
    yaml_file << R"(
vehicle:
  geometry:
    mass_kg: -1000.0
    wheelbase_m: 3.0
    track_width_m: 1.7
    wheel_radius_m: 0.35
  drivetrain:
    motor_power_max_w: 200000.0
  battery:
    capacity_kWh: 100.0
)";
    yaml_file.close();
    
    bool caught_exception = false;
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);
    } catch (const std::exception& e) {
        caught_exception = true;
        std::string msg = e.what();
        if (msg.find("mass") != std::string::npos || msg.find("Invalid") != std::string::npos) {
            result.pass("Correctly rejected negative mass");
        } else {
            result.fail("Exception thrown but wrong message: " + msg);
        }
    }
    
    if (!caught_exception) {
        result.fail("Should have thrown exception for negative mass");
    }
    
    std::remove(temp_yaml);
}

// Test 5: Invalid YAML - invalid SOC range
void test_invalid_soc_range(TestResult& result) {
    std::cout << "\n=== Test 5: Validation - Invalid SOC Range ===\n";
    
    const char* temp_yaml = "/tmp/test_vehicle_invalid_soc.yaml";
    std::ofstream yaml_file(temp_yaml);
    yaml_file << R"(
vehicle:
  geometry:
    mass_kg: 2000.0
    wheelbase_m: 3.0
  drivetrain:
    motor_power_max_w: 200000.0
  battery:
    capacity_kWh: 100.0
    min_soc: 0.80
    max_soc: 0.60
)";
    yaml_file.close();
    
    bool caught_exception = false;
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);
    } catch (const std::exception& e) {
        caught_exception = true;
        std::string msg = e.what();
        if (msg.find("SOC") != std::string::npos || msg.find("Invalid") != std::string::npos) {
            result.pass("Correctly rejected invalid SOC range (min > max)");
        } else {
            result.fail("Exception thrown but wrong message: " + msg);
        }
    }
    
    if (!caught_exception) {
        result.fail("Should have thrown exception for invalid SOC range");
    }
    
    std::remove(temp_yaml);
}

// Test 6: Invalid YAML - zero motor power
void test_invalid_motor_power(TestResult& result) {
    std::cout << "\n=== Test 6: Validation - Zero Motor Power ===\n";
    
    const char* temp_yaml = "/tmp/test_vehicle_zero_power.yaml";
    std::ofstream yaml_file(temp_yaml);
    yaml_file << R"(
vehicle:
  geometry:
    mass_kg: 2000.0
  drivetrain:
    motor_power_max_w: 0.0
  battery:
    capacity_kWh: 100.0
)";
    yaml_file.close();
    
    bool caught_exception = false;
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);
    } catch (const std::exception& e) {
        caught_exception = true;
        std::string msg = e.what();
        if (msg.find("motor_power") != std::string::npos || msg.find("Invalid") != std::string::npos) {
            result.pass("Correctly rejected zero motor power");
        } else {
            result.fail("Exception thrown but wrong message: " + msg);
        }
    }
    
    if (!caught_exception) {
        result.fail("Should have thrown exception for zero motor power");
    }
    
    std::remove(temp_yaml);
}

int main() {
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            VehicleConfig Unit Tests                         ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n";
    
    TestResult result;
    
    test_default_config(result);
    test_valid_yaml(result);
    test_missing_file(result);
    test_invalid_mass(result);
    test_invalid_soc_range(result);
    test_invalid_motor_power(result);
    
    result.summary();
    
    return (result.failed == 0) ? 0 : 1;
}