// test/test_steer_dynamics.cpp
//
// Unit tests for enhanced steering system
// Tests: Ackermann geometry, actuator dynamics, aligning torque feedback

#include <iostream>
#include <string>
#include <cmath>

#include "plant/steer_subsystem/steer_plant.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

using namespace plant;

// ============================================================================
// Test Framework
// ============================================================================

const char* GREEN = "\033[32m";
const char* RED = "\033[31m";
const char* YELLOW = "\033[33m";
const char* RESET = "\033[0m";

struct TestResult {
    int passed = 0;
    int failed = 0;
    
    void pass(const std::string& msg) {
        std::cout << "  " << GREEN << "✓ " << RESET << msg << "\n";
        passed++;
    }
    
    void fail(const std::string& msg) {
        std::cout << "  " << RED << "✗ " << RESET << msg << "\n";
        failed++;
    }
    
    bool check(bool condition, const std::string& msg) {
        if (condition) { pass(msg); return true; }
        else { fail(msg); return false; }
    }
};

bool is_close(double a, double b, double tol = 0.01) {
    return std::abs(a - b) < tol;
}

double deg2rad(double d) { return d * M_PI / 180.0; }
double rad2deg(double r) { return r * 180.0 / M_PI; }

// ============================================================================
// Test 1: Ackermann Geometry - Straight Ahead
// ============================================================================

void test_ackermann_straight(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 1: Ackermann Straight Ahead ===" << RESET << "\n";
    
    SteerParams params;
    params.wheelbase_m = 6.3;
    params.track_width_m = 4.0;
    params.ackermann_factor = 1.0;
    params.actuator_dynamics_enabled = false;
    
    SteerPlant steer(params);
    
    PlantState s;
    s.v_mps = 10.0;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 0.0;  // Straight ahead
    
    steer.step(s, cmd, 0.01);
    
    r.check(std::abs(s.steer_virtual_rad) < 1e-6,
            "Virtual steering = 0 rad");
    
    r.check(std::abs(s.delta_fl_rad) < 1e-6,
            "FL wheel angle = 0 rad");
    
    r.check(std::abs(s.delta_fr_rad) < 1e-6,
            "FR wheel angle = 0 rad");
}

// ============================================================================
// Test 2: Ackermann Geometry - Right Turn
// ============================================================================

void test_ackermann_right_turn(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 2: Ackermann Right Turn ===" << RESET << "\n";
    
    SteerParams params;
    params.wheelbase_m = 6.3;
    params.track_width_m = 4.0;
    params.delta_max_deg = 30.0;
    params.ackermann_factor = 1.0;
    params.actuator_dynamics_enabled = false;  // Instant response
    params.steer_rate_dps = 1000.0;  // Very fast rate (effectively instant)
    
    SteerPlant steer(params);
    
    PlantState s;
    s.v_mps = 5.0;  // Low speed (no speed limit)
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 20.0;  // 20° right turn
    
    // Run a few steps to reach commanded angle
    for (int i = 0; i < 10; i++) {
        steer.step(s, cmd, 0.01);
    }
    
    // For right turn (δ > 0):
    //   - FR is inner wheel → steers MORE
    //   - FL is outer wheel → steers LESS
    
    r.check(s.steer_virtual_rad > 0.0,
            "Virtual steering > 0 (right turn)");
    
    r.check(s.delta_fr_rad > s.delta_fl_rad,
            "FR (inner) > FL (outer): FR=" + std::to_string(rad2deg(s.delta_fr_rad)) +
            "°, FL=" + std::to_string(rad2deg(s.delta_fl_rad)) + "°");
    
    // Verify Ackermann geometry: cot(δ_outer) - cot(δ_inner) = W/L
    double cot_diff = 1.0/std::tan(s.delta_fl_rad) - 1.0/std::tan(s.delta_fr_rad);
    double expected_cot_diff = params.track_width_m / params.wheelbase_m;
    
    r.check(is_close(cot_diff, expected_cot_diff, 0.01),
            "Ackermann relation: cot(δ_outer)-cot(δ_inner)=" + 
            std::to_string(cot_diff) + " ≈ W/L=" + std::to_string(expected_cot_diff));
}

// ============================================================================
// Test 3: Speed-Dependent Steering Limit
// ============================================================================

void test_speed_dependent_limit(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 3: Speed-Dependent Steering Limit ===" << RESET << "\n";
    
    SteerParams params;
    params.delta_max_deg = 30.0;
    params.v_steer_limit_start_mps = 8.0;
    params.v_steer_limit_end_mps = 25.0;
    params.steer_limit_ratio_highv = 0.40;
    params.actuator_dynamics_enabled = false;
    params.steer_rate_dps = 1000.0;
    
    SteerPlant steer(params);
    
    // Test at low speed (no limit)
    PlantState s_low;
    s_low.v_mps = 5.0;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 30.0;  // Max steering
    
    for (int i = 0; i < 20; i++) steer.step(s_low, cmd, 0.01);
    
    double delta_low_deg = rad2deg(s_low.steer_virtual_rad);
    r.check(is_close(delta_low_deg, 30.0, 1.0),
            "Low speed: δ=" + std::to_string(delta_low_deg) + "° ≈ 30° (no limit)");
    
    // Test at high speed (limited)
    steer.reset();
    PlantState s_high;
    s_high.v_mps = 30.0;  // Above v_steer_limit_end
    
    for (int i = 0; i < 20; i++) steer.step(s_high, cmd, 0.01);
    
    double delta_high_deg = rad2deg(s_high.steer_virtual_rad);
    double expected_high = 30.0 * params.steer_limit_ratio_highv;  // 12°
    
    r.check(is_close(delta_high_deg, expected_high, 1.0),
            "High speed: δ=" + std::to_string(delta_high_deg) + 
            "° ≈ " + std::to_string(expected_high) + "° (limited to 40%)");
}

// ============================================================================
// Test 4: Rate Limiting (Kinematic Mode)
// ============================================================================

void test_rate_limiting(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 4: Rate Limiting ===" << RESET << "\n";
    
    SteerParams params;
    params.steer_rate_dps = 45.0;  // 45°/s max rate
    params.actuator_dynamics_enabled = false;
    
    SteerPlant steer(params);
    
    PlantState s;
    s.v_mps = 5.0;
    s.steer_virtual_rad = 0.0;  // Start at zero
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 30.0;  // Step command to 30°
    
    // One step at 10ms
    steer.step(s, cmd, 0.01);
    
    // Expected: rate limited to 45°/s × 0.01s = 0.45°
    double delta_deg = rad2deg(s.steer_virtual_rad);
    double expected_deg = 0.45;
    
    r.check(is_close(delta_deg, expected_deg, 0.1),
            "Rate limited: δ=" + std::to_string(delta_deg) + 
            "° ≈ " + std::to_string(expected_deg) + "° (45°/s × 10ms)");
    
    // Check rate
    double rate_dps = rad2deg(s.steer_rate_radps);
    r.check(is_close(rate_dps, 45.0, 1.0),
            "Steering rate ≈ 45°/s");
}

// ============================================================================
// Test 5: Actuator Dynamics (2nd Order System)
// ============================================================================

void test_actuator_dynamics(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 5: Actuator Dynamics ===" << RESET << "\n";
    
    SteerParams params;
    params.actuator_dynamics_enabled = true;
    params.actuator_time_constant_s = 0.15;  // 150ms time constant
    params.actuator_damping_ratio = 0.8;
    params.steer_rate_dps = 100.0;  // High rate limit (not the constraint)
    
    SteerPlant steer(params);
    
    PlantState s;
    s.v_mps = 5.0;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;  // Step command to 10°
    
    // Run for 500ms (should reach ~95% of final value for 2nd order)
    for (int i = 0; i < 50; i++) {
        steer.step(s, cmd, 0.01);
    }
    
    double delta_500ms_deg = rad2deg(s.steer_virtual_rad);
    
    // For 2nd order system with τ=0.15, 2ζτ=0.24
    // After 500ms ≈ 3.3τ, should be >90% of final value
    r.check(delta_500ms_deg > 8.0 && delta_500ms_deg < 11.0,
            "After 500ms: δ=" + std::to_string(delta_500ms_deg) + 
            "° (expecting ~9-10° with dynamics)");
    
    // Run for another 500ms (should be very close to command)
    for (int i = 0; i < 50; i++) {
        steer.step(s, cmd, 0.01);
    }
    
    double delta_1s_deg = rad2deg(s.steer_virtual_rad);
    r.check(is_close(delta_1s_deg, 10.0, 0.5),
            "After 1s: δ=" + std::to_string(delta_1s_deg) + "° ≈ 10° (settled)");
}

// ============================================================================
// Test 6: Actuator Dynamics - Damped Response (No Overshoot)
// ============================================================================

void test_actuator_damped(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 6: Damped Response ===" << RESET << "\n";
    
    SteerParams params;
    params.actuator_dynamics_enabled = true;
    params.actuator_time_constant_s = 0.10;
    params.actuator_damping_ratio = 1.0;  // Critically damped (no overshoot)
    params.steer_rate_dps = 200.0;
    
    SteerPlant steer(params);
    
    PlantState s;
    s.v_mps = 5.0;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 15.0;
    
    double max_delta = 0.0;
    
    // Run for 1 second
    for (int i = 0; i < 100; i++) {
        steer.step(s, cmd, 0.01);
        max_delta = std::max(max_delta, rad2deg(s.steer_virtual_rad));
    }
    
    // Critically damped should not overshoot
    r.check(max_delta <= 15.5,  // Small tolerance for numerical error
            "No overshoot: max=" + std::to_string(max_delta) + "° ≤ 15.5°");
}

// ============================================================================
// Test 7: SteerSubsystem Priority
// ============================================================================

void test_steer_subsystem_priority(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 7: SteerSubsystem Priority ===" << RESET << "\n";
    
    SteerParams params;
    SteerSubsystem subsystem(params);
    
    r.check(subsystem.priority() == 50,
            "Priority = 50 (Actuators category)");
    
    r.check(std::string(subsystem.name()) == "Steer",
            "Name = 'Steer'");
}

// ============================================================================
// Test 8: Ackermann Factor (Partial vs Full)
// ============================================================================

void test_ackermann_factor(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 8: Ackermann Factor ===" << RESET << "\n";
    
    SteerParams params_full;
    params_full.wheelbase_m = 6.3;
    params_full.track_width_m = 4.0;
    params_full.ackermann_factor = 1.0;  // Full Ackermann
    params_full.actuator_dynamics_enabled = false;
    params_full.steer_rate_dps = 1000.0;
    
    SteerParams params_partial;
    params_partial.wheelbase_m = 6.3;
    params_partial.track_width_m = 4.0;
    params_partial.ackermann_factor = 0.5;  // 50% Ackermann
    params_partial.actuator_dynamics_enabled = false;
    params_partial.steer_rate_dps = 1000.0;
    
    SteerPlant steer_full(params_full);
    SteerPlant steer_partial(params_partial);
    
    PlantState s_full, s_partial;
    s_full.v_mps = s_partial.v_mps = 5.0;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 20.0;
    
    for (int i = 0; i < 20; i++) {
        steer_full.step(s_full, cmd, 0.01);
        steer_partial.step(s_partial, cmd, 0.01);
    }
    
    // Full Ackermann: δ_fl ≠ δ_fr
    double diff_full = std::abs(rad2deg(s_full.delta_fr_rad) - rad2deg(s_full.delta_fl_rad));
    
    // Partial Ackermann: δ_fl and δ_fr should be closer together
    double diff_partial = std::abs(rad2deg(s_partial.delta_fr_rad) - rad2deg(s_partial.delta_fl_rad));
    
    r.check(diff_full > diff_partial,
            "Full Ackermann has larger FL/FR difference: " +
            std::to_string(diff_full) + "° > " + std::to_string(diff_partial) + "°");
    
    // At ack=0.5, difference should be about half
    r.check(is_close(diff_partial, diff_full * 0.5, 0.5),
            "Partial Ackermann (0.5) has ~half the difference");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            Steering System Unit Tests                         ║\n";
    std::cout << "║  Tests: Ackermann, Dynamics, Rate Limiting                    ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";
    
    TestResult r;
    
    test_ackermann_straight(r);
    test_ackermann_right_turn(r);
    test_speed_dependent_limit(r);
    test_rate_limiting(r);
    test_actuator_dynamics(r);
    test_actuator_damped(r);
    test_steer_subsystem_priority(r);
    test_ackermann_factor(r);
    
    std::cout << "\n════════════════════════════════════════════\n";
    if (r.failed == 0) {
        std::cout << GREEN << "ALL TESTS PASSED";
    } else {
        std::cout << RED << "SOME TESTS FAILED";
    }
    std::cout << " (" << r.passed << " passed, " << r.failed << " failed)" << RESET << "\n";
    std::cout << "════════════════════════════════════════════\n";
    
    return r.failed > 0 ? 1 : 0;
}