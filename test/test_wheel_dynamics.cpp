// test/test_wheel_dynamics.cpp
/**
 * Unit Test: WheelDynamics and WheelSubsystem
 * 
 * Tests wheel rotational dynamics, slip ratio calculation, and
 * integration with the Dugoff tire model.
 * 
 * Test Coverage:
 *   1. Slip ratio calculation (zero slip, drive slip, brake slip)
 *   2. Wheel integration (acceleration, deceleration, zero-crossing)
 *   3. Stability criterion validation
 *   4. Tire force feedback (closed-loop behavior)
 *   5. WheelSubsystem initialization and mode switching
 */

#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>

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

// ============================================================================
// Test 1: Slip Ratio Calculation
// ============================================================================

void test_slip_ratio_calculation(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 1: Slip Ratio Calculation ===" << COLOR_RESET << "\n";
    
    plant::WheelDynamicsParams params;
    params.radius_m = 1.93;
    params.v_eps_mps = 0.1;
    
    plant::WheelDynamics wheel(params);
    
    // Test 1a: Zero slip (wheel speed matches vehicle speed)
    double v_vehicle = 10.0;  // m/s
    wheel.set_omega_radps(v_vehicle / params.radius_m);  // ω = V/R
    double sigma = wheel.compute_slip_ratio(v_vehicle);
    
    if (std::abs(sigma) < 0.001) {
        result.pass("Zero slip: σ = " + std::to_string(sigma) + " (expected ~0)");
    } else {
        result.fail("Zero slip: σ = " + std::to_string(sigma) + " (expected ~0)");
    }
    
    // Test 1b: Positive slip (wheel spinning faster - acceleration)
    wheel.set_omega_radps(v_vehicle / params.radius_m * 1.2);  // 20% faster
    sigma = wheel.compute_slip_ratio(v_vehicle);
    
    if (sigma > 0.15 && sigma < 0.25) {
        result.pass("Drive slip: σ = " + std::to_string(sigma) + " (expected ~0.17-0.2)");
    } else {
        result.fail("Drive slip: σ = " + std::to_string(sigma) + " (expected ~0.17-0.2)");
    }
    
    // Test 1c: Negative slip (wheel spinning slower - braking)
    wheel.set_omega_radps(v_vehicle / params.radius_m * 0.8);  // 20% slower
    sigma = wheel.compute_slip_ratio(v_vehicle);
    
    if (sigma < -0.15 && sigma > -0.25) {
        result.pass("Brake slip: σ = " + std::to_string(sigma) + " (expected ~-0.2)");
    } else {
        result.fail("Brake slip: σ = " + std::to_string(sigma) + " (expected ~-0.2)");
    }
    
    // Test 1d: Wheel lock (wheel stopped, vehicle moving)
    wheel.set_omega_radps(0.0);
    sigma = wheel.compute_slip_ratio(v_vehicle);
    
    if (sigma < -0.95) {
        result.pass("Wheel lock: σ = " + std::to_string(sigma) + " (expected ~ -1.0)");
    } else {
        result.fail("Wheel lock: σ = " + std::to_string(sigma) + " (expected ~ -1.0)");
    }
    
    // Test 1e: Low speed protection (denominator clamping)
    wheel.set_omega_radps(0.05);  // Very slow wheel
    sigma = wheel.compute_slip_ratio(0.05);  // Very slow vehicle
    
    if (std::isfinite(sigma)) {
        result.pass("Low speed: σ = " + std::to_string(sigma) + " (finite, no division by zero)");
    } else {
        result.fail("Low speed: σ = " + std::to_string(sigma) + " (division by zero!)");
    }
}

// ============================================================================
// Test 2: Wheel Integration
// ============================================================================

void test_wheel_integration(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 2: Wheel Integration ===" << COLOR_RESET << "\n";
    
    plant::WheelDynamicsParams params;
    params.inertia_kgm2 = 1000.0;  // kg·m²
    params.radius_m = 1.93;
    params.omega_max_radps = 50.0;
    params.omega_min_radps = -10.0;
    
    plant::WheelDynamics wheel(params);
    
    // Test 2a: Acceleration from rest with drive torque
    wheel.set_omega_radps(0.0);
    double tau_drive = 50000.0;  // 50 kNm drive torque
    double fx = 0.0;  // No tire force initially
    double dt = 0.001;  // 1ms timestep
    
    wheel.step(tau_drive, 0.0, fx, dt);
    
    // Expected: ω̇ = τ/I = 50000/1000 = 50 rad/s²
    // After 1ms: ω = 0.05 rad/s
    if (wheel.omega_radps() > 0.04 && wheel.omega_radps() < 0.06) {
        result.pass("Acceleration: ω = " + std::to_string(wheel.omega_radps()) + " rad/s (expected ~0.05)");
    } else {
        result.fail("Acceleration: ω = " + std::to_string(wheel.omega_radps()) + " rad/s (expected ~0.05)");
    }
    
    // Test 2b: Deceleration with brake torque
    wheel.set_omega_radps(10.0);  // 10 rad/s
    double tau_brake = 20000.0;   // 20 kNm brake torque
    
    wheel.step(0.0, tau_brake, fx, dt);
    
    // Expected: ω̇ = -τ/I = -20 rad/s² (brake opposes motion)
    // After 1ms: ω = 10 - 0.02 = 9.98 rad/s
    if (wheel.omega_radps() < 10.0 && wheel.omega_radps() > 9.9) {
        result.pass("Deceleration: ω = " + std::to_string(wheel.omega_radps()) + " rad/s");
    } else {
        result.fail("Deceleration: ω = " + std::to_string(wheel.omega_radps()) + " rad/s (expected ~9.98)");
    }
    
    // Test 2c: Zero-crossing protection
    wheel.set_omega_radps(0.005);  // Very small positive velocity
    wheel.step(0.0, 10000.0, 0.0, dt);  // Brake torque
    
    // Should snap to zero instead of going negative
    if (std::abs(wheel.omega_radps()) < 0.01) {
        result.pass("Zero-crossing: ω = " + std::to_string(wheel.omega_radps()) + " (snapped to ~0)");
    } else {
        result.fail("Zero-crossing: ω = " + std::to_string(wheel.omega_radps()) + " (should be ~0)");
    }
    
    // Test 2d: Max speed clamping
    wheel.set_omega_radps(49.0);
    wheel.step(100000.0, 0.0, 0.0, 0.1);  // Large torque, long timestep
    
    if (wheel.omega_radps() <= params.omega_max_radps) {
        result.pass("Max speed clamp: ω = " + std::to_string(wheel.omega_radps()) + " (≤ " + std::to_string(params.omega_max_radps) + ")");
    } else {
        result.fail("Max speed clamp: ω = " + std::to_string(wheel.omega_radps()) + " (exceeded max!)");
    }
}

// ============================================================================
// Test 3: Tire Force Feedback
// ============================================================================

void test_tire_force_feedback(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 3: Tire Force Feedback ===" << COLOR_RESET << "\n";
    
    plant::WheelDynamicsParams params;
    params.inertia_kgm2 = 1000.0;
    params.radius_m = 1.93;
    
    plant::WheelDynamics wheel(params);
    
    // Test: With tire force, acceleration should be reduced
    // Iw·ω̇ = τ_drive - Fx·R
    // Higher Fx → lower ω̇
    
    wheel.set_omega_radps(5.0);
    double tau_drive = 50000.0;  // 50 kNm
    double dt = 0.001;
    
    // Case A: No tire force
    double omega_before = wheel.omega_radps();
    wheel.step(tau_drive, 0.0, 0.0, dt);
    double accel_no_force = (wheel.omega_radps() - omega_before) / dt;
    
    // Case B: With tire force
    wheel.set_omega_radps(5.0);
    omega_before = wheel.omega_radps();
    double fx = 20000.0;  // 20 kN tire force
    wheel.step(tau_drive, 0.0, fx, dt);
    double accel_with_force = (wheel.omega_radps() - omega_before) / dt;
    
    if (accel_with_force < accel_no_force) {
        result.pass("Tire feedback: accel with Fx (" + std::to_string(accel_with_force) + 
                   ") < accel without (" + std::to_string(accel_no_force) + ")");
    } else {
        result.fail("Tire feedback: Fx should reduce acceleration!");
    }
    
    // Verify the reduction amount
    // Expected reduction: Fx·R / Iw = 20000 * 1.93 / 1000 = 38.6 rad/s²
    double expected_diff = fx * params.radius_m / params.inertia_kgm2;
    double actual_diff = accel_no_force - accel_with_force;
    
    if (is_close(actual_diff, expected_diff, 0.01)) {
        result.pass("Feedback magnitude: Δaccel = " + std::to_string(actual_diff) + 
                   " (expected " + std::to_string(expected_diff) + ")");
    } else {
        result.fail("Feedback magnitude: Δaccel = " + std::to_string(actual_diff) + 
                   " (expected " + std::to_string(expected_diff) + ")");
    }
}

// ============================================================================
// Test 4: Stability Criterion
// ============================================================================

void test_stability_criterion(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 4: Stability Criterion ===" << COLOR_RESET << "\n";
    
    plant::WheelDynamicsParams params;
    params.inertia_kgm2 = 1000.0;  // kg·m²
    params.radius_m = 1.93;        // m
    
    plant::WheelDynamics wheel(params);
    
    // For XCMG truck: Cx = 280000 N/slip
    double cx = 280000.0;
    double dt_max = wheel.get_stability_dt_max(cx);
    
    // Expected: dt_max = 2 * 1000 / (280000 * 1.93²) ≈ 1.92 ms
    double expected_dt = 2.0 * params.inertia_kgm2 / (cx * params.radius_m * params.radius_m);
    
    std::cout << "  Calculated dt_max: " << (dt_max * 1000.0) << " ms\n";
    std::cout << "  Expected dt_max:   " << (expected_dt * 1000.0) << " ms\n";
    
    if (is_close(dt_max, expected_dt, 0.01)) {
        result.pass("Stability criterion: dt_max = " + std::to_string(dt_max * 1000.0) + " ms");
    } else {
        result.fail("Stability criterion mismatch!");
    }
    
    // Verify it's approximately 1.9ms
    if (dt_max > 0.0015 && dt_max < 0.0025) {
        result.pass("dt_max in expected range (1.5-2.5 ms)");
    } else {
        result.fail("dt_max outside expected range!");
    }
}

// ============================================================================
// Test 5: WheelSubsystem Initialization
// ============================================================================

void test_wheel_subsystem_init(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 5: WheelSubsystem Initialization ===" << COLOR_RESET << "\n";
    
    plant::WheelSubsystemParams params;
    params.mass_kg = 220000.0;  // 220 tons
    params.wheel.radius_m = 1.93;
    params.dynamic_mode_enabled = false;  // Start in kinematic mode
    
    // Set tire params using existing TyreDugoffParams
    params.tyre_params.Cx_base = 280000.0;
    params.tyre_params.Cy_base = 220000.0;
    params.tyre_params.Fz_ref = 800000.0;
    params.tyre_params.mu_peak = 0.72;
    
    plant::WheelSubsystem subsystem(params);
    plant::PlantState state;
    state.v_mps = 5.0;  // 5 m/s
    
    // Initialize
    subsystem.initialize(state);
    
    // Check wheel speeds initialized from velocity
    double expected_omega = 5.0 / 1.93;
    
    if (is_close(state.omega_rl_radps, expected_omega, 0.01)) {
        result.pass("Wheel init from velocity: ω = " + std::to_string(state.omega_rl_radps) + 
                   " rad/s (expected " + std::to_string(expected_omega) + ")");
    } else {
        result.fail("Wheel init failed!");
    }
    
    // Check priority
    if (subsystem.priority() == 105) {
        result.pass("Priority = 105 (after Drive=100)");
    } else {
        result.fail("Priority = " + std::to_string(subsystem.priority()) + " (expected 105)");
    }
    
    // Check name
    if (std::string(subsystem.name()) == "Wheel") {
        result.pass("Subsystem name = 'Wheel'");
    } else {
        result.fail("Subsystem name = '" + std::string(subsystem.name()) + "' (expected 'Wheel')");
    }
}

// ============================================================================
// Test 6: Kinematic vs Dynamic Mode
// ============================================================================

void test_kinematic_vs_dynamic_mode(TestResult& result) {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 6: Kinematic vs Dynamic Mode ===" << COLOR_RESET << "\n";
    
    plant::WheelSubsystemParams params;
    params.mass_kg = 220000.0;
    params.wheel.radius_m = 1.93;
    params.wheel.inertia_kgm2 = 1000.0;
    
    // Set tire params using existing TyreDugoffParams
    params.tyre_params.Cx_base = 280000.0;
    params.tyre_params.Cy_base = 220000.0;
    params.tyre_params.Fz_ref = 800000.0;
    params.tyre_params.mu_peak = 0.72;
    
    sim::ActuatorCmd cmd;
    cmd.system_enable = true;
    
    // Test kinematic mode
    params.dynamic_mode_enabled = false;
    plant::WheelSubsystem kin_subsystem(params);
    plant::PlantState kin_state;
    kin_state.v_mps = 10.0;
    kin_state.tau_drive_rl_nm = 50000.0;  // Drive torque (ignored in kinematic)
    
    kin_subsystem.initialize(kin_state);
    kin_subsystem.step(kin_state, cmd, 0.001);
    
    // In kinematic mode, slip should be ~0
    if (std::abs(kin_state.sigma_x_rl) < 0.01) {
        result.pass("Kinematic mode: σ_rl = " + std::to_string(kin_state.sigma_x_rl) + " (~0)");
    } else {
        result.fail("Kinematic mode: σ_rl = " + std::to_string(kin_state.sigma_x_rl) + " (expected ~0)");
    }
    
    // Test dynamic mode
    params.dynamic_mode_enabled = true;
    plant::WheelSubsystem dyn_subsystem(params);
    plant::PlantState dyn_state;
    dyn_state.v_mps = 10.0;
    dyn_state.tau_drive_rl_nm = 100000.0;  // High drive torque
    dyn_state.Fz_rl = 500000.0;  // Normal load
    
    dyn_subsystem.initialize(dyn_state);
    
    // Run a few timesteps to build up slip
    for (int i = 0; i < 50; i++) {
        dyn_subsystem.step(dyn_state, cmd, 0.001);
    }
    
    // In dynamic mode with high torque, slip should be non-zero
    if (std::abs(dyn_state.sigma_x_rl) > 0.01) {
        result.pass("Dynamic mode: σ_rl = " + std::to_string(dyn_state.sigma_x_rl) + " (non-zero)");
    } else {
        result.fail("Dynamic mode: σ_rl = " + std::to_string(dyn_state.sigma_x_rl) + " (expected non-zero)");
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << COLOR_YELLOW << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║           WheelDynamics & WheelSubsystem Tests             ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝" << COLOR_RESET << "\n";
    
    TestResult result;
    
    test_slip_ratio_calculation(result);
    test_wheel_integration(result);
    test_tire_force_feedback(result);
    test_stability_criterion(result);
    test_wheel_subsystem_init(result);
    test_kinematic_vs_dynamic_mode(result);
    
    result.summary();
    
    return (result.failed == 0) ? 0 : 1;
}