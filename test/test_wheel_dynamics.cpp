// test/test_wheel_dynamics.cpp
//
// Unit tests for wheel dynamics subsystem
// Tests: TyreDugoff, WheelDynamics, WheelSubsystem
//
// Reference: Vehicle_Dynamics_with_Dugoff_Tire_Model.pdf

#include <iostream>
#include <string>
#include <cmath>

// Include the components under test
#include "plant/tyre_models/tyre_dugoff.hpp"
#include "plant/wheel_subsystem/wheel_dynamics.hpp"
#include "plant/wheel_subsystem/wheel_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"

using namespace plant;

// ============================================================================
// Test Framework (minimal)
// ============================================================================

static const char* GREEN  = "\033[32m";
static const char* RED    = "\033[31m";
static const char* YELLOW = "\033[33m";
static const char* RESET  = "\033[0m";

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
        fail(msg);
        return false;
    }
};

static bool is_close(double a, double b, double tol = 0.01) {
    return std::abs(a - b) < tol;
}

// ============================================================================
// Test 1: TyreDugoff Slip Ratio Computation
// ============================================================================

static void test_tyre_dugoff_slip(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 1: TyreDugoff Slip Ratio ===" << RESET << "\n";

    TyreDugoffParams params;
    params.mu_peak = 0.72;
    params.Cx_base = 280000.0;
    params.Fz_ref = 100000.0;
    TyreDugoff tyre(params);

    // Free rolling: ω·R = Vx → σx ≈ 0
    const double R = 1.93;
    const double Vx = 10.0;
    const double omega_free = Vx / R;

    TyreForces f1 = tyre.compute_forces(omega_free, R, Vx, 0.0, 100000.0);
    r.check(std::abs(f1.sigma_x) < 0.01,
            "Free rolling: σx=" + std::to_string(f1.sigma_x) + " ≈ 0");

    // Drive slip: ω·R > Vx → σx > 0
    const double omega_spin = (Vx / R) * 1.1;
    TyreForces f2 = tyre.compute_forces(omega_spin, R, Vx, 0.0, 100000.0);
    r.check(f2.sigma_x > 0.05,
            "Drive slip: σx=" + std::to_string(f2.sigma_x) + " > 0");

    // Brake slip: ω·R < Vx → σx < 0
    const double omega_lock = (Vx / R) * 0.8;
    TyreForces f3 = tyre.compute_forces(omega_lock, R, Vx, 0.0, 100000.0);
    r.check(f3.sigma_x < -0.10,
            "Brake slip: σx=" + std::to_string(f3.sigma_x) + " < 0");
}

// ============================================================================
// Test 2: TyreDugoff Friction Circle
// ============================================================================

static void test_tyre_dugoff_friction_circle(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 2: Friction Circle Constraint ===" << RESET << "\n";

    TyreDugoffParams params;
    params.mu_peak = 0.72;
    params.Cx_base = 280000.0;
    params.Cy_base = 220000.0;
    params.Fz_ref = 100000.0;
    TyreDugoff tyre(params);

    const double Fz = 500000.0;
    const double F_max = params.mu_peak * Fz;

    const double R = 1.93;
    const double Vx = 10.0;
    const double omega_high_slip = (Vx / R) * 1.5;

    TyreForces f = tyre.compute_forces(omega_high_slip, R, Vx, 0.5, Fz);
    const double F_total = std::sqrt(f.Fx * f.Fx + f.Fy * f.Fy);

    r.check(F_total <= F_max * 1.01,
            "Friction circle: |F|=" + std::to_string(F_total / 1000.0) +
            " kN ≤ μFz=" + std::to_string(F_max / 1000.0) + " kN");

    r.check(f.lambda < 1.0,
            "Saturated regime: λ=" + std::to_string(f.lambda) + " < 1");
}

// ============================================================================
// Test 3: WheelDynamics Integration
// ============================================================================

static void test_wheel_dynamics_integration(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 3: Wheel Dynamics Integration ===" << RESET << "\n";

    WheelDynamicsParams params;
    params.inertia_kgm2 = 1000.0;
    params.radius_m = 1.93;

    WheelDynamics wheel(params);
    wheel.set_omega_radps(5.0);

    const double tau_drive = 10000.0;
    const double dt = 0.001;

    const double omega_before = wheel.omega_radps();
    wheel.step(tau_drive, 0.0, 0.0, dt);
    const double omega_after = wheel.omega_radps();

    const double delta_omega = omega_after - omega_before;
    const double expected_delta = (tau_drive / params.inertia_kgm2) * dt;

    r.check(is_close(delta_omega, expected_delta, 0.001),
            "Free spin: Δω=" + std::to_string(delta_omega) +
            " rad/s (expected " + std::to_string(expected_delta) + ")");
}

// ============================================================================
// Test 4: Wheel Dynamics Stability Criterion
// ============================================================================

static void test_wheel_stability_criterion(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 4: Stability Criterion ===" << RESET << "\n";

    WheelDynamicsParams params;
    params.inertia_kgm2 = 1000.0;
    params.radius_m = 1.93;

    WheelDynamics wheel(params);

    const double Cx = 280000.0;
    const double dt_max = wheel.get_stability_dt_max(Cx);

    const double expected = 2.0 * params.inertia_kgm2 / (Cx * params.radius_m * params.radius_m);

    r.check(is_close(dt_max, expected, 0.0001),
            "dt_max=" + std::to_string(dt_max * 1000.0) +
            " ms (expected " + std::to_string(expected * 1000.0) + " ms)");

    r.check(dt_max > 0.001 && dt_max < 0.003,
            "Stability requires dt < ~2ms for these parameters");
}

// ============================================================================
// Test 5: WheelSubsystem Initialization
// ============================================================================

static void test_wheel_subsystem_init(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 5: WheelSubsystem Initialization ===" << RESET << "\n";

    // Your current WheelSubsystemParams does NOT include mass_kg or cg_to_front_m.
    // Keep the physical values in the test for expected-load calculations.
    const double mass_kg = 218000.0;
    const double wheelbase_m = 6.3;
    const double cg_to_front_m = 3.78; // 60% rear bias
    const double g = 9.81;

    WheelSubsystemParams params;
    params.mass_kg = mass_kg;
    params.wheelbase_m = wheelbase_m;
    params.track_m = 7.2;
    params.cg_height_m = 3.2;
    params.cg_to_front_m = cg_to_front_m;
    params.wheel.radius_m = 1.93;
    params.wheel.inertia_kgm2 = 1000.0;
    params.tyre_params.mu_peak = 0.72;
    params.mu_peak = 0.72;
    params.mu_slide = 0.65;
    params.dynamic_mode_enabled = true;

    WheelSubsystem subsystem(params);

    PlantState s;
    s.v_mps = 10.0;

    subsystem.initialize(s);

    // Check wheel speed initialized from velocity
    const double expected_omega = s.v_mps / params.wheel.radius_m;
    r.check(is_close(s.omega_rl_radps, expected_omega, 0.01),
            "omega_rl_radps=" + std::to_string(s.omega_rl_radps) +
            " rad/s (from v=" + std::to_string(s.v_mps) + " m/s)");

    // Expected static normal loads (simple 2-axle split)
    const double W = mass_kg * g;
    const double Wf_expected = W * (wheelbase_m - cg_to_front_m) / wheelbase_m;
    const double Wr_expected = W * (cg_to_front_m) / wheelbase_m;

    const double Fz_front_actual = s.Fz_fl + s.Fz_fr;
    const double Fz_rear_actual  = s.Fz_rl + s.Fz_rr;

    r.check(is_close(Fz_front_actual, Wf_expected, 1000.0),
            "Front axle load: " + std::to_string(Fz_front_actual / 1000.0) +
            " kN (expected " + std::to_string(Wf_expected / 1000.0) + " kN)");

    r.check(is_close(Fz_rear_actual, Wr_expected, 1000.0),
            "Rear axle load: " + std::to_string(Fz_rear_actual / 1000.0) +
            " kN (expected " + std::to_string(Wr_expected / 1000.0) + " kN)");

    r.check(subsystem.priority() == 105, "Priority = 105 (after Drive=100)");

    r.check(s.dynamic_model_enabled == true, "dynamic_model_enabled = true");
}

// ============================================================================
// Test 6: WheelSubsystem Dynamic Mode Step
// ============================================================================

static void test_wheel_subsystem_dynamic_step(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 6: WheelSubsystem Dynamic Step ===" << RESET << "\n";

    WheelSubsystemParams params;
    params.mass_kg = 218000.0;
    params.wheelbase_m = 6.3;
    params.track_m = 7.2;
    params.cg_height_m = 3.2;
    params.cg_to_front_m = 3.78;
    params.wheel.radius_m = 1.93;
    params.wheel.inertia_kgm2 = 1000.0;

    params.tyre_params.mu_peak = 0.72;
    params.tyre_params.Cx_base = 280000.0;
    params.tyre_params.Fz_ref = 100000.0;

    params.mu_peak = 0.72;
    params.mu_slide = 0.65;
    params.dynamic_mode_enabled = true;

    WheelSubsystem subsystem(params);

    PlantState s;
    s.v_mps = 10.0;
    s.a_long_mps2 = 0.0;

    // Simulate DriveSubsystem output: drive torque on rear wheels
    s.tau_drive_rl_nm = 50000.0;
    s.tau_drive_rr_nm = 50000.0;

    s.tau_brake_fl_nm = 0.0;
    s.tau_brake_fr_nm = 0.0;
    s.tau_brake_rl_nm = 0.0;
    s.tau_brake_rr_nm = 0.0;

    subsystem.initialize(s);

    sim::ActuatorCmd cmd;
    const double dt = 0.001;

    const double omega_before = s.omega_rl_radps;

    // Run multiple steps to allow slip to develop
    // (First step: free-rolling → slip=0, Fx≈0)
    // (Subsequent steps: slip builds up → Fx > 0)
    for (int i = 0; i < 5; i++) {
        subsystem.step(s, cmd, dt);
    }

    const double omega_after = s.omega_rl_radps;

    r.check(omega_after > omega_before,
            "Wheel accelerates: ω increased from " + std::to_string(omega_before) +
            " to " + std::to_string(omega_after) + " rad/s");

    r.check(s.Fx_rl != 0.0,
            "Tire force Fx_rl=" + std::to_string(s.Fx_rl / 1000.0) + " kN");

    r.check(s.sigma_x_rl > 0.0,
            "Drive slip: σx_rl=" + std::to_string(s.sigma_x_rl));
}

// ============================================================================
// Test 7: Load Transfer During Acceleration
// ============================================================================

static void test_load_transfer(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 7: Longitudinal Load Transfer ===" << RESET << "\n";

    const double mass_kg = 218000.0;
    const double wheelbase_m = 6.3;
    const double cg_height_m = 2.8;
    const double cg_to_front_m = 3.78;

    WheelSubsystemParams params;
    params.mass_kg = mass_kg;
    params.wheelbase_m = wheelbase_m;
    params.track_m = 7.2;
    params.cg_height_m = cg_height_m;
    params.cg_to_front_m = cg_to_front_m;

    WheelSubsystem subsystem(params);

    PlantState s;
    s.v_mps = 10.0;

    // Static case
    s.a_long_mps2 = 0.0;
    subsystem.initialize(s);

    const double Fz_front_static = s.Fz_fl + s.Fz_fr;
    const double Fz_rear_static  = s.Fz_rl + s.Fz_rr;

    // Acceleration case: use step() to recompute loads inside the subsystem path
    s.a_long_mps2 = 2.0;

    sim::ActuatorCmd cmd;
    subsystem.step(s, cmd, 0.01);

    const double Fz_front_accel = s.Fz_fl + s.Fz_fr;
    const double Fz_rear_accel  = s.Fz_rl + s.Fz_rr;

    r.check(Fz_front_accel < Fz_front_static,
            "Accel: Front load decreased from " + std::to_string(Fz_front_static / 1000.0) +
            " to " + std::to_string(Fz_front_accel / 1000.0) + " kN");

    r.check(Fz_rear_accel > Fz_rear_static,
            "Accel: Rear load increased from " + std::to_string(Fz_rear_static / 1000.0) +
            " to " + std::to_string(Fz_rear_accel / 1000.0) + " kN");

    // Expected ΔFz = m·a·h/L (rear gain, front loss)
    const double expected_dFz = mass_kg * 2.0 * cg_height_m / wheelbase_m;
    const double actual_dFz   = Fz_rear_accel - Fz_rear_static;

    r.check(is_close(actual_dFz, expected_dFz, 2000.0),
            "Load transfer: ΔFz=" + std::to_string(actual_dFz / 1000.0) +
            " kN (expected " + std::to_string(expected_dFz / 1000.0) + " kN)");

    // Also sanity-check the split used in expectations
    (void)cg_to_front_m;
}

// ============================================================================
// Test 8: Kinematic vs Dynamic Mode
// ============================================================================

static void test_kinematic_vs_dynamic(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 8: Kinematic vs Dynamic Mode ===" << RESET << "\n";

    WheelSubsystemParams params;
    params.mass_kg = 218000.0;
    params.wheelbase_m = 6.3;
    params.track_m = 7.2;
    params.cg_height_m = 3.2;
    params.cg_to_front_m = 3.78;
    params.wheel.radius_m = 1.93;

    // KINEMATIC mode
    params.dynamic_mode_enabled = false;
    WheelSubsystem kin_subsystem(params);

    PlantState s_kin;
    s_kin.v_mps = 15.0;
    kin_subsystem.initialize(s_kin);

    sim::ActuatorCmd cmd;
    kin_subsystem.step(s_kin, cmd, 0.01);

    const double expected_omega = s_kin.v_mps / params.wheel.radius_m;
    r.check(is_close(s_kin.omega_rl_radps, expected_omega, 0.001),
            "KINEMATIC: ω_rl=" + std::to_string(s_kin.omega_rl_radps) +
            " = V/R=" + std::to_string(expected_omega));

    // DYNAMIC mode
    params.dynamic_mode_enabled = true;
    WheelSubsystem dyn_subsystem(params);

    PlantState s_dyn;
    s_dyn.v_mps = 15.0;
    s_dyn.tau_drive_rl_nm = 100000.0;
    s_dyn.tau_drive_rr_nm = 100000.0;
    dyn_subsystem.initialize(s_dyn);

    for (int i = 0; i < 10; i++) {
        dyn_subsystem.step(s_dyn, cmd, 0.001);
    }

    const double omega_no_slip = s_dyn.v_mps / params.wheel.radius_m;
    const double slip = (s_dyn.omega_rl_radps * params.wheel.radius_m - s_dyn.v_mps) /
                        std::max(1e-6, std::abs(s_dyn.v_mps));

    r.check(s_dyn.omega_rl_radps > omega_no_slip,
            "DYNAMIC: ω_rl=" + std::to_string(s_dyn.omega_rl_radps) +
            " > V/R (slip=" + std::to_string(slip * 100.0) + "%)");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║            Wheel Dynamics Unit Tests                          ║\n";
    std::cout << "║  Reference: Vehicle_Dynamics_with_Dugoff_Tire_Model.pdf       ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";

    TestResult r;

    test_tyre_dugoff_slip(r);
    test_tyre_dugoff_friction_circle(r);
    test_wheel_dynamics_integration(r);
    test_wheel_stability_criterion(r);
    test_wheel_subsystem_init(r);
    test_wheel_subsystem_dynamic_step(r);
    test_load_transfer(r);
    test_kinematic_vs_dynamic(r);

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
