// test/test_vehicle_config.cpp
/**
 * Unit Test: VehicleConfig
 *
 * Tests YAML loading, validation, and default configuration.
 *
 * Test Coverage:
 *   1. Default configuration generation
 *   2. Valid YAML loading
 *   3. Missing file fallback to defaults
 *   4. Validation — negative mass
 *   5. Validation — invalid mu_surface
 *   6. Validation — zero motor power
 */

#include "config/vehicle_config.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

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

bool is_close(double actual, double expected, double tolerance = 0.0001) {
    if (std::abs(expected) < 1e-9) return std::abs(actual) < tolerance;
    return std::abs(actual - expected) / std::abs(expected) < tolerance;
}

// Test 1: Default configuration
void test_default_config(TestResult& result) {
    std::cout << "\n=== Test 1: Default Configuration ===\n";

    config::VehicleConfig cfg = config::VehicleConfig::get_default();

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

    if (cfg.params.drive.mu_surface > 0.0 && cfg.params.drive.mu_surface <= 2.0) {
        result.pass("Default mu_surface is valid: " + std::to_string(cfg.params.drive.mu_surface));
    } else {
        result.fail("Default mu_surface is invalid");
    }

    if (cfg.params.drive.Cy_front_Npm > 0.0 && cfg.params.drive.Cy_rear_Npm > 0.0) {
        result.pass("Default cornering stiffness is positive");
    } else {
        result.fail("Default cornering stiffness invalid");
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
    efficiency: 0.95

  brakes:
    brake_torque_max_nm: 5000.0
    brake_bias_front: 0.40

  resistance:
    drag_coefficient: 2.5
    rolling_resistance: 1500.0

  limits:
    v_max_mps: 50.0
    v_stop_eps: 0.3

  dynamics:
    mu_surface: 0.85
    Cy_front_Npm: 2000000.0
    Cy_rear_Npm: 1800000.0
)";
    yaml_file.close();

    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);

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

        if (is_close(cfg.params.drive.mu_surface, 0.85)) {
            result.pass("mu_surface loaded correctly: " + std::to_string(cfg.params.drive.mu_surface));
        } else {
            result.fail("mu_surface mismatch");
        }

        if (is_close(cfg.params.wheelbase_m, 3.0)) {
            result.pass("Wheelbase loaded correctly: " + std::to_string(cfg.params.wheelbase_m) + " m");
        } else {
            result.fail("Wheelbase mismatch");
        }

    } catch (const std::exception& e) {
        result.fail(std::string("Exception during load: ") + e.what());
    }

    std::remove(temp_yaml);
}

// Test 3: Missing file fallback
void test_missing_file(TestResult& result) {
    std::cout << "\n=== Test 3: Missing File Fallback ===\n";

    const char* missing_file = "/tmp/nonexistent_vehicle_config.yaml";

    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(missing_file);

        if (cfg.params.drive.mass_kg > 0.0 && cfg.params.drive.motor_power_max_w > 0.0) {
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
    wheel_radius_m: 0.35
  drivetrain:
    motor_power_max_w: 200000.0
    motor_torque_max_nm: 2500.0
    gear_ratio: 10.0
    efficiency: 0.95
  limits:
    v_max_mps: 30.0
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

    if (!caught_exception) result.fail("Should have thrown exception for negative mass");

    std::remove(temp_yaml);
}

// Test 5: Invalid YAML - invalid mu_surface (zero)
void test_invalid_mu_surface(TestResult& result) {
    std::cout << "\n=== Test 5: Validation - Invalid mu_surface ===\n";

    const char* temp_yaml = "/tmp/test_vehicle_invalid_mu.yaml";
    std::ofstream yaml_file(temp_yaml);
    yaml_file << R"(
vehicle:
  geometry:
    mass_kg: 2000.0
    wheelbase_m: 3.0
    wheel_radius_m: 0.35
  drivetrain:
    motor_power_max_w: 200000.0
    motor_torque_max_nm: 2500.0
    gear_ratio: 10.0
    efficiency: 0.95
  limits:
    v_max_mps: 30.0
  dynamics:
    mu_surface: 0.0
    Cy_front_Npm: 2000000.0
    Cy_rear_Npm: 1800000.0
)";
    yaml_file.close();

    bool caught_exception = false;
    try {
        config::VehicleConfig cfg = config::VehicleConfig::load(temp_yaml);
    } catch (const std::exception& e) {
        caught_exception = true;
        std::string msg = e.what();
        if (msg.find("mu_surface") != std::string::npos || msg.find("Invalid") != std::string::npos) {
            result.pass("Correctly rejected zero mu_surface");
        } else {
            result.fail("Exception thrown but wrong message: " + msg);
        }
    }

    if (!caught_exception) result.fail("Should have thrown exception for zero mu_surface");

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
    wheelbase_m: 3.0
    wheel_radius_m: 0.35
  drivetrain:
    motor_power_max_w: 0.0
    motor_torque_max_nm: 2500.0
    gear_ratio: 10.0
    efficiency: 0.95
  limits:
    v_max_mps: 30.0
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

    if (!caught_exception) result.fail("Should have thrown exception for zero motor power");

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
    test_invalid_mu_surface(result);
    test_invalid_motor_power(result);

    result.summary();

    return (result.failed == 0) ? 0 : 1;
}
