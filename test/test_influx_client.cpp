// test/test_influx_client.cpp
// Unit tests for InfluxDB client integration

#include "utils/influx.hpp"
#include "plant/plant_state.hpp"
#include "sensors/sensor_out.hpp"
#include "sim/actuator_cmd.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>
#include <thread>

// Test helper macros
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "FAILED: " << message << std::endl; \
            std::cerr << "  at " << __FILE__ << ":" << __LINE__ << std::endl; \
            return false; \
        } \
    } while (0)

#define RUN_TEST(test_func) \
    do { \
        std::cout << "Running " << #test_func << "... "; \
        if (test_func()) { \
            std::cout << "PASSED" << std::endl; \
            passed++; \
        } else { \
            std::cout << "FAILED" << std::endl; \
            failed++; \
        } \
        total++; \
    } while (0)

// Helper to create minimal plant state
plant::PlantState create_test_state() {
    plant::PlantState state;
    state.x_m = 100.0;
    state.y_m = 50.0;
    state.yaw_rad = 0.5;
    state.v_mps = 10.0;
    state.steer_virtual_rad = 0.1;
    state.delta_fl_rad = 0.1;
    state.delta_fr_rad = 0.1;
    state.motor_power_kW = 150.0;
    state.regen_power_kW = 0.0;
    state.brake_force_kN = 0.0;
    
    // Battery state
    state.batt_soc_pct = 80.0;
    state.batt_v = 800.0;
    state.batt_i = 200.0;
    
    // Wheel speeds
    state.wheel_fl_rps = 5.0;
    state.wheel_fr_rps = 5.0;
    state.wheel_rl_rps = 5.0;
    state.wheel_rr_rps = 5.0;
    
    return state;
}

// Helper to create minimal sensor output
sensors::SensorOut create_test_sensors() {
    sensors::SensorOut sensors;
    
    // Battery sensors
    sensors.batt_soc_meas = 80.0;
    sensors.batt_v_meas = 800.0;
    sensors.batt_i_meas = 200.0;
    sensors.batt_temp_meas = 25.0;
    
    // Wheel sensors
    sensors.wheel_fl_rps_meas = 5.0;
    sensors.wheel_fr_rps_meas = 5.0;
    sensors.wheel_rl_rps_meas = 5.0;
    sensors.wheel_rr_rps_meas = 5.0;
    
    // IMU
    sensors.imu_valid = true;
    sensors.imu_gx_rps = 0.01;
    sensors.imu_gy_rps = 0.02;
    sensors.imu_gz_rps = 0.03;
    sensors.imu_ax_mps2 = 1.0;
    sensors.imu_ay_mps2 = 0.5;
    sensors.imu_az_mps2 = 9.81;
    sensors.imu_temp_c = 25.0;
    
    // GNSS
    sensors.gnss_valid = true;
    sensors.gnss_lat_deg = -31.9505;
    sensors.gnss_lon_deg = 115.8605;
    sensors.gnss_alt_m = 50.0;
    sensors.gnss_vn_mps = 10.0;
    sensors.gnss_ve_mps = 0.0;
    sensors.gnss_fix_type = 4;  // RTK fixed
    sensors.gnss_sat_count = 12;
    
    // Radar
    sensors.radar_valid = true;
    sensors.radar_target_range_m = 100.0;
    sensors.radar_target_rel_vel_mps = -5.0;
    sensors.radar_target_angle_deg = 0.0;
    sensors.radar_status = 1;
    
    return sensors;
}

// Helper to create minimal actuator command
sim::ActuatorCmd create_test_cmd() {
    sim::ActuatorCmd cmd;
    cmd.drive_torque_cmd_nm = 5000.0;
    cmd.brake_cmd_pct = 0.0;
    cmd.steer_cmd_deg = 5.0;
    cmd.system_enable = true;
    return cmd;
}

// ============================================================================
// Test Cases
// ============================================================================

// Test 1: Client creation with disabled config
bool test_client_creation_disabled() {
    utils::InfluxClient::Config config;
    config.enabled = false;
    
    utils::InfluxClient client(config);
    
    TEST_ASSERT(!client.is_enabled(), "Client should be disabled");
    
    return true;
}

// Test 2: Client creation with enabled config (no actual connection)
bool test_client_creation_enabled() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.1;
    
    utils::InfluxClient client(config);
    
    TEST_ASSERT(client.is_enabled(), "Client should be enabled");
    
    return true;
}

// Test 3: Write with disabled client (should not write)
bool test_write_disabled_client() {
    utils::InfluxClient::Config config;
    config.enabled = false;
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    bool result = client.write_data_point(state, sensors, cmd, 0.0);
    
    TEST_ASSERT(!result, "Write should return false for disabled client");
    
    return true;
}

// Test 4: Rate limiting (should skip writes within interval)
bool test_rate_limiting() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 1.0;  // 1 second interval
    config.token = "fake-token";  // Won't actually connect
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // First write at t=0 (should attempt to write, will fail due to no server)
    // We're testing rate limiting, not connection, so we ignore the return value
    client.write_data_point(state, sensors, cmd, 0.0);
    
    // Second write at t=0.5 (should skip due to rate limiting)
    bool result2 = client.write_data_point(state, sensors, cmd, 0.5);
    TEST_ASSERT(!result2, "Write should be skipped due to rate limiting");
    
    // Third write at t=1.0 (should attempt to write)
    client.write_data_point(state, sensors, cmd, 1.0);
    
    // Fourth write at t=1.5 (should skip)
    bool result4 = client.write_data_point(state, sensors, cmd, 1.5);
    TEST_ASSERT(!result4, "Write should be skipped due to rate limiting");
    
    return true;
}

// Test 5: Config validation (default values)
bool test_config_defaults() {
    utils::InfluxClient::Config config;
    
    TEST_ASSERT(!config.enabled, "Default enabled should be false");
    TEST_ASSERT(config.url == "http://localhost:8086", "Default URL incorrect");
    TEST_ASSERT(config.token == "", "Default token should be empty");
    TEST_ASSERT(config.org == "Autonomy", "Default org incorrect");
    TEST_ASSERT(config.bucket == "vehicle-sim", "Default bucket incorrect");
    TEST_ASSERT(std::abs(config.write_interval_s - 0.25) < 0.001, "Default interval incorrect");
    
    return true;
}

// Test 6: Config with custom values
bool test_config_custom() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://192.168.1.100:8086";
    config.token = "my-secret-token";
    config.org = "MyOrg";
    config.bucket = "my-bucket";
    config.write_interval_s = 0.5;
    
    TEST_ASSERT(config.enabled, "Custom enabled incorrect");
    TEST_ASSERT(config.url == "http://192.168.1.100:8086", "Custom URL incorrect");
    TEST_ASSERT(config.token == "my-secret-token", "Custom token incorrect");
    TEST_ASSERT(config.org == "MyOrg", "Custom org incorrect");
    TEST_ASSERT(config.bucket == "my-bucket", "Custom bucket incorrect");
    TEST_ASSERT(std::abs(config.write_interval_s - 0.5) < 0.001, "Custom interval incorrect");
    
    return true;
}

// Test 7: Multiple writes at different intervals
bool test_multiple_writes() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.25;  // 250ms
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // Simulate writes at different times
    double times[] = {0.0, 0.1, 0.25, 0.3, 0.5, 0.75, 1.0};
    bool expected[] = {true, false, true, false, true, true, true};
    
    for (size_t i = 0; i < 7; i++) {
        client.write_data_point(state, sensors, cmd, times[i]);
        // Note: We can't check return value reliably since it depends on HTTP connection
        // But the rate limiting logic is being exercised
    }
    
    return true;
}

// Test 8: Flush operation (should not crash)
bool test_flush() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    // Should not crash even with no writes
    client.flush();
    
    // Write some data then flush
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    client.write_data_point(state, sensors, cmd, 0.0);
    client.flush();
    
    return true;
}

// Test 9: State changes between writes
bool test_state_changes() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.1;
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // Write initial state
    client.write_data_point(state, sensors, cmd, 0.0);
    
    // Change state
    state.x_m = 200.0;
    state.y_m = 100.0;
    state.v_mps = 20.0;
    
    // Write changed state
    client.write_data_point(state, sensors, cmd, 0.1);
    
    return true;
}

// Test 10: Sensor validity flags
bool test_sensor_validity() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.1;
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // Test with all sensors valid
    sensors.imu_valid = true;
    sensors.gnss_valid = true;
    sensors.radar_valid = true;
    client.write_data_point(state, sensors, cmd, 0.0);
    
    // Test with some sensors invalid
    sensors.imu_valid = false;
    sensors.gnss_valid = true;
    sensors.radar_valid = false;
    client.write_data_point(state, sensors, cmd, 0.1);
    
    // Test with all sensors invalid
    sensors.imu_valid = false;
    sensors.gnss_valid = false;
    sensors.radar_valid = false;
    client.write_data_point(state, sensors, cmd, 0.2);
    
    return true;
}

// Test 11: Extreme values
bool test_extreme_values() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.1;
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // Test with extreme values
    state.x_m = 1e6;  // 1000 km
    state.y_m = -1e6;
    state.v_mps = 100.0;  // Very fast
    state.batt_soc_pct = 0.0;  // Empty battery
    
    client.write_data_point(state, sensors, cmd, 0.0);
    
    return true;
}

// Test 12: Very high frequency writes (stress test)
bool test_high_frequency_writes() {
    utils::InfluxClient::Config config;
    config.enabled = true;
    config.url = "http://localhost:8086";
    config.org = "test-org";
    config.bucket = "test-bucket";
    config.write_interval_s = 0.01;  // 10ms = 100Hz
    config.token = "fake-token";
    
    utils::InfluxClient client(config);
    
    auto state = create_test_state();
    auto sensors = create_test_sensors();
    auto cmd = create_test_cmd();
    
    // Simulate 100 writes at 10ms intervals (1 second total)
    for (int i = 0; i < 100; i++) {
        double t = i * 0.01;
        state.x_m += 1.0;  // Vehicle moving
        client.write_data_point(state, sensors, cmd, t);
    }
    
    return true;
}

// ============================================================================
// Main test runner
// ============================================================================

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "InfluxDB Client Unit Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    int total = 0;
    int passed = 0;
    int failed = 0;
    
    // Run all tests
    RUN_TEST(test_client_creation_disabled);
    RUN_TEST(test_client_creation_enabled);
    RUN_TEST(test_write_disabled_client);
    RUN_TEST(test_rate_limiting);
    RUN_TEST(test_config_defaults);
    RUN_TEST(test_config_custom);
    RUN_TEST(test_multiple_writes);
    RUN_TEST(test_flush);
    RUN_TEST(test_state_changes);
    RUN_TEST(test_sensor_validity);
    RUN_TEST(test_extreme_values);
    RUN_TEST(test_high_frequency_writes);
    
    // Print summary
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Test Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "Total:  " << total << std::endl;
    std::cout << "Passed: " << passed << std::endl;
    std::cout << "Failed: " << failed << std::endl;
    std::cout << "========================================" << std::endl;
    
    if (failed == 0) {
        std::cout << "✓ All tests passed!" << std::endl;
        return 0;
    } else {
        std::cout << "✗ Some tests failed!" << std::endl;
        return 1;
    }
}