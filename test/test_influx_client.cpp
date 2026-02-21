// test/test_influx_client.cpp
// Unit tests for InfluxDB client (simplified: vehicle truth only, no sensor modules).

#include "utils/influx.hpp"
#include "plant/plant_main/plant_state.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "FAILED: " << message << "\n"; \
            std::cerr << "  at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

#define RUN_TEST(test_func) \
    do { \
        std::cout << "Running " << #test_func << "... "; \
        if (test_func()) { \
            std::cout << "PASSED\n"; \
            passed++; \
        } else { \
            std::cout << "FAILED\n"; \
            failed++; \
        } \
        total++; \
    } while (0)

static plant::PlantState create_test_state() {
    plant::PlantState s;
    s.x_m = 100.0;
    s.y_m = 50.0;
    s.yaw_rad = 0.5;
    s.v_mps = 10.0;
    s.vy_mps = 0.5;
    s.yaw_rate_radps = 0.1;
    s.a_long_mps2 = 1.0;
    s.a_lat_mps2 = 0.2;
    s.steer_virtual_rad = 0.1;
    s.delta_fl_rad = 0.11;
    s.delta_fr_rad = 0.09;
    s.motor_torque_nm = 5000.0;
    s.brake_force_kN = 0.0;
    s.wheel_fl_rps = 5.0;
    s.wheel_fr_rps = 5.0;
    s.wheel_rl_rps = 5.0;
    s.wheel_rr_rps = 5.0;
    return s;
}

// Test 1: Disabled client — is_enabled() = false
static bool test_client_disabled() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = false;
    utils::InfluxClient client(cfg);
    TEST_ASSERT(!client.is_enabled(), "disabled client should report !is_enabled()");
    return true;
}

// Test 2: Enabled client — is_enabled() = true
static bool test_client_enabled() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy";
    cfg.bucket = "test-bucket";
    utils::InfluxClient client(cfg);
    TEST_ASSERT(client.is_enabled(), "enabled client should report is_enabled()");
    return true;
}

// Test 3: Write to disabled client returns false
static bool test_write_disabled() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = false;
    utils::InfluxClient client(cfg);
    auto s = create_test_state();
    bool result = client.write_vehicle_truth(s, 0.0);
    TEST_ASSERT(!result, "write to disabled client should return false");
    return true;
}

// Test 4: Rate limiting (second write within interval is skipped)
static bool test_rate_limiting() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy";
    cfg.bucket = "test-bucket";
    cfg.write_interval_s = 1.0;   // 1 s interval
    utils::InfluxClient client(cfg);

    auto s = create_test_state();
    // First write (will attempt HTTP — expected to fail without server, ignore result)
    client.write_vehicle_truth(s, 0.0);

    // 0.5 s later — within interval, should skip
    bool r2 = client.write_vehicle_truth(s, 0.5);
    TEST_ASSERT(!r2, "write within interval should be skipped (rate limited)");

    // 1.0 s later — at interval boundary, should attempt
    client.write_vehicle_truth(s, 1.0);

    // 1.3 s — still within next interval
    bool r4 = client.write_vehicle_truth(s, 1.3);
    TEST_ASSERT(!r4, "write within second interval should be skipped");

    return true;
}

// Test 5: Default config values
static bool test_config_defaults() {
    utils::InfluxClient::Config cfg;
    TEST_ASSERT(!cfg.enabled, "default enabled = false");
    TEST_ASSERT(cfg.url == "http://localhost:8086", "default URL");
    TEST_ASSERT(cfg.token == "", "default token empty");
    TEST_ASSERT(cfg.org == "Autonomy", "default org");
    TEST_ASSERT(cfg.bucket == "vehicle-sim", "default bucket");
    TEST_ASSERT(std::abs(cfg.write_interval_s - 0.25) < 0.001, "default interval 250 ms");
    return true;
}

// Test 6: Custom config values
static bool test_config_custom() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url = "http://192.168.1.1:8086";
    cfg.token = "mytoken";
    cfg.org = "MyOrg";
    cfg.bucket = "my-bucket";
    cfg.write_interval_s = 0.5;
    TEST_ASSERT(cfg.url == "http://192.168.1.1:8086", "custom URL");
    TEST_ASSERT(cfg.token == "mytoken", "custom token");
    TEST_ASSERT(cfg.org == "MyOrg", "custom org");
    TEST_ASSERT(cfg.bucket == "my-bucket", "custom bucket");
    TEST_ASSERT(std::abs(cfg.write_interval_s - 0.5) < 0.001, "custom interval");
    return true;
}

// Test 7: flush() does not crash
static bool test_flush() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy";
    cfg.bucket = "test";
    utils::InfluxClient client(cfg);
    client.flush();   // no-op, must not crash
    auto s = create_test_state();
    client.write_vehicle_truth(s, 0.0);
    client.flush();
    return true;
}

// Test 8: State changes between writes (exercise code paths)
static bool test_state_changes() {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy";
    cfg.bucket = "test";
    cfg.write_interval_s = 0.1;
    utils::InfluxClient client(cfg);

    auto s = create_test_state();
    client.write_vehicle_truth(s, 0.0);

    s.x_m = 200.0;
    s.v_mps = 20.0;
    client.write_vehicle_truth(s, 0.1);

    return true;
}

int main() {
    std::cout << "========================================\n";
    std::cout << "InfluxDB Client Unit Tests\n";
    std::cout << "========================================\n\n";

    int total = 0, passed = 0, failed = 0;

    RUN_TEST(test_client_disabled);
    RUN_TEST(test_client_enabled);
    RUN_TEST(test_write_disabled);
    RUN_TEST(test_rate_limiting);
    RUN_TEST(test_config_defaults);
    RUN_TEST(test_config_custom);
    RUN_TEST(test_flush);
    RUN_TEST(test_state_changes);

    std::cout << "\n========================================\n";
    std::cout << "Total: " << total << "  Passed: " << passed << "  Failed: " << failed << "\n";
    std::cout << "========================================\n";

    if (failed == 0) {
        std::cout << "All tests passed!\n";
        return 0;
    } else {
        std::cout << "Some tests FAILED!\n";
        return 1;
    }
}
