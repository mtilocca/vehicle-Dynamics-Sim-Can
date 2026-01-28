// test/test_steer_ackermann.cpp
//
// Unit tests for steering with Ackermann geometry
// Verifies: Ackermann condition, inner/outer wheel angles, percentage blending

#include <iostream>
#include <string>
#include <cmath>

// Standalone test - minimal dependencies
// In real build, include the actual headers

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

constexpr double PI = 3.14159265358979323846;
double deg2rad(double d) { return d * PI / 180.0; }
double rad2deg(double r) { return r * 180.0 / PI; }

// ============================================================================
// Ackermann Map Function (Copy from steer_plant.cpp for testing)
// ============================================================================

void ackermann_map(
    double steer_virtual_rad,
    double L,
    double W,
    double ackermann_pct,
    double& delta_fl_rad,
    double& delta_fr_rad)
{
    if (std::abs(steer_virtual_rad) < 1e-6) {
        delta_fl_rad = 0.0;
        delta_fr_rad = 0.0;
        return;
    }

    const double R = L / std::tan(steer_virtual_rad);
    const double R_FL = R + W / 2.0;
    const double R_FR = R - W / 2.0;
    
    double delta_fl_ackermann = std::atan(L / R_FL);
    double delta_fr_ackermann = std::atan(L / R_FR);

    double ack = std::max(0.0, std::min(1.0, ackermann_pct));
    
    delta_fl_rad = ack * delta_fl_ackermann + (1.0 - ack) * steer_virtual_rad;
    delta_fr_rad = ack * delta_fr_ackermann + (1.0 - ack) * steer_virtual_rad;
}

// ============================================================================
// Test 1: Straight Ahead
// ============================================================================

void test_straight_ahead(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 1: Straight Ahead ===" << RESET << "\n";
    
    double delta_fl, delta_fr;
    ackermann_map(0.0, 6.3, 4.0, 1.0, delta_fl, delta_fr);
    
    r.check(std::abs(delta_fl) < 1e-6, "FL = 0 for straight ahead");
    r.check(std::abs(delta_fr) < 1e-6, "FR = 0 for straight ahead");
}

// ============================================================================
// Test 2: Right Turn - Inner Wheel Turns More
// ============================================================================

void test_right_turn_inner_outer(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 2: Right Turn - Inner/Outer ===" << RESET << "\n";
    
    const double L = 6.3;
    const double W = 4.0;
    const double delta_virtual = deg2rad(20.0);  // 20° right turn
    
    double delta_fl, delta_fr;
    ackermann_map(delta_virtual, L, W, 1.0, delta_fl, delta_fr);  // 100% Ackermann
    
    // For right turn (δ > 0):
    // - FR is INNER (closer to turn center) → should turn MORE
    // - FL is OUTER (farther from turn center) → should turn LESS
    
    r.check(delta_fr > delta_fl,
            "Inner (FR=" + std::to_string(rad2deg(delta_fr)) + "°) > Outer (FL=" + 
            std::to_string(rad2deg(delta_fl)) + "°) for right turn");
    
    r.check(delta_fr > delta_virtual,
            "Inner wheel turns MORE than virtual: FR=" + 
            std::to_string(rad2deg(delta_fr)) + "° > " + std::to_string(rad2deg(delta_virtual)) + "°");
    
    r.check(delta_fl < delta_virtual,
            "Outer wheel turns LESS than virtual: FL=" + 
            std::to_string(rad2deg(delta_fl)) + "° < " + std::to_string(rad2deg(delta_virtual)) + "°");
}

// ============================================================================
// Test 3: Left Turn - Inner Wheel Turns More
// ============================================================================

void test_left_turn_inner_outer(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 3: Left Turn - Inner/Outer ===" << RESET << "\n";
    
    const double L = 6.3;
    const double W = 4.0;
    const double delta_virtual = deg2rad(-20.0);  // 20° left turn
    
    double delta_fl, delta_fr;
    ackermann_map(delta_virtual, L, W, 1.0, delta_fl, delta_fr);
    
    // For left turn (δ < 0):
    // - FL is INNER (closer to turn center) → should turn MORE (more negative)
    // - FR is OUTER (farther from turn center) → should turn LESS (less negative)
    
    r.check(delta_fl < delta_fr,  // More negative means turned more
            "Inner (FL=" + std::to_string(rad2deg(delta_fl)) + "°) more negative than Outer (FR=" + 
            std::to_string(rad2deg(delta_fr)) + "°) for left turn");
    
    r.check(std::abs(delta_fl) > std::abs(delta_fr),
            "|Inner| > |Outer|: |FL|=" + std::to_string(std::abs(rad2deg(delta_fl))) + 
            "° > |FR|=" + std::to_string(std::abs(rad2deg(delta_fr))) + "°");
}

// ============================================================================
// Test 4: Ackermann Condition: cot(δ_outer) - cot(δ_inner) = W/L
// ============================================================================

void test_ackermann_condition(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 4: Ackermann Condition ===" << RESET << "\n";
    
    const double L = 6.3;
    const double W = 4.0;
    const double expected_cot_diff = W / L;  // 4.0 / 6.3 ≈ 0.635
    
    // Test at various steering angles
    for (double delta_deg : {10.0, 20.0, 30.0}) {
        double delta_virtual = deg2rad(delta_deg);
        double delta_fl, delta_fr;
        ackermann_map(delta_virtual, L, W, 1.0, delta_fl, delta_fr);
        
        // For right turn: FL is outer, FR is inner
        double cot_outer = 1.0 / std::tan(delta_fl);
        double cot_inner = 1.0 / std::tan(delta_fr);
        double cot_diff = cot_outer - cot_inner;
        
        r.check(is_close(cot_diff, expected_cot_diff, 0.01),
                "At δ=" + std::to_string(delta_deg) + "°: cot(outer)-cot(inner)=" + 
                std::to_string(cot_diff) + " ≈ W/L=" + std::to_string(expected_cot_diff));
    }
}

// ============================================================================
// Test 5: Ackermann Percentage - Blending with Parallel
// ============================================================================

void test_ackermann_percentage(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 5: Ackermann Percentage ===" << RESET << "\n";
    
    const double L = 6.3;
    const double W = 4.0;
    const double delta_virtual = deg2rad(20.0);
    
    // 0% Ackermann (parallel steering)
    double delta_fl_0, delta_fr_0;
    ackermann_map(delta_virtual, L, W, 0.0, delta_fl_0, delta_fr_0);
    
    r.check(is_close(delta_fl_0, delta_virtual, 0.001) && 
            is_close(delta_fr_0, delta_virtual, 0.001),
            "0% Ackermann: FL=FR=virtual=" + std::to_string(rad2deg(delta_virtual)) + "°");
    
    // 100% Ackermann
    double delta_fl_100, delta_fr_100;
    ackermann_map(delta_virtual, L, W, 1.0, delta_fl_100, delta_fr_100);
    
    double diff_100 = std::abs(delta_fr_100 - delta_fl_100);
    r.check(diff_100 > 0.01,
            "100% Ackermann: |FR-FL|=" + std::to_string(rad2deg(diff_100)) + "° (distinct angles)");
    
    // 50% Ackermann (halfway)
    double delta_fl_50, delta_fr_50;
    ackermann_map(delta_virtual, L, W, 0.5, delta_fl_50, delta_fr_50);
    
    double diff_50 = std::abs(delta_fr_50 - delta_fl_50);
    r.check(diff_50 > 0.0 && diff_50 < diff_100,
            "50% Ackermann: |FR-FL|=" + std::to_string(rad2deg(diff_50)) + 
            "° (between 0 and " + std::to_string(rad2deg(diff_100)) + "°)");
    
    // Verify it's approximately half
    r.check(is_close(diff_50, diff_100 * 0.5, 0.01),
            "50% Ackermann difference ≈ half of 100%");
}

// ============================================================================
// Test 6: Symmetry - Left and Right Turns Mirror Each Other
// ============================================================================

void test_symmetry(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 6: Left/Right Symmetry ===" << RESET << "\n";
    
    const double L = 6.3;
    const double W = 4.0;
    const double delta_virtual_right = deg2rad(20.0);
    const double delta_virtual_left = deg2rad(-20.0);
    
    double delta_fl_right, delta_fr_right;
    double delta_fl_left, delta_fr_left;
    
    ackermann_map(delta_virtual_right, L, W, 1.0, delta_fl_right, delta_fr_right);
    ackermann_map(delta_virtual_left, L, W, 1.0, delta_fl_left, delta_fr_left);
    
    // For symmetric geometry, right turn angles should mirror left turn angles
    // δ_FL(right) = -δ_FR(left)
    // δ_FR(right) = -δ_FL(left)
    
    r.check(is_close(delta_fl_right, -delta_fr_left, 0.001),
            "FL(right)=" + std::to_string(rad2deg(delta_fl_right)) + 
            "° ≈ -FR(left)=" + std::to_string(rad2deg(-delta_fr_left)) + "°");
    
    r.check(is_close(delta_fr_right, -delta_fl_left, 0.001),
            "FR(right)=" + std::to_string(rad2deg(delta_fr_right)) + 
            "° ≈ -FL(left)=" + std::to_string(rad2deg(-delta_fl_left)) + "°");
}

// ============================================================================
// Test 7: XCMG XDE320 Mining Truck Parameters
// ============================================================================

void test_xcmg_parameters(TestResult& r) {
    std::cout << "\n" << YELLOW << "=== Test 7: XCMG XDE320 Parameters ===" << RESET << "\n";
    
    // XCMG XDE320 (Table 4)
    const double L = 6.3;           // Wheelbase
    const double W = 4.0;           // Track width (estimated)
    const double delta_max = deg2rad(35.0);  // Max steering
    
    double delta_fl, delta_fr;
    ackermann_map(delta_max, L, W, 1.0, delta_fl, delta_fr);
    
    // At max steering, compute turn radius
    double R = L / std::tan(delta_max);
    double R_inner = R - W / 2.0;
    
    std::cout << "  Turn radius at max steering: R=" << R << " m\n";
    std::cout << "  Inner wheel radius: R_inner=" << R_inner << " m\n";
    
    r.check(R > 0, "Turn radius positive at max right steering: R=" + std::to_string(R) + " m");
    r.check(R_inner > 0, "Inner wheel radius positive: R_inner=" + std::to_string(R_inner) + " m");
    
    // Mining truck minimum turn radius (inner wheel)
    // Typical mining trucks: 10-15m minimum turn radius
    r.check(R_inner > 5.0 && R_inner < 20.0,
            "Realistic mining truck turn radius: " + std::to_string(R_inner) + " m (5-20m typical)");
    
    // Inner wheel angle should be larger than outer
    r.check(delta_fr > delta_fl,
            "Inner (FR=" + std::to_string(rad2deg(delta_fr)) + "°) > Outer (FL=" + 
            std::to_string(rad2deg(delta_fl)) + "°)");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    std::cout << "╔═══════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          Ackermann Steering Geometry Tests                    ║\n";
    std::cout << "║  Verifies: Inner/outer angles, Ackermann condition, blending  ║\n";
    std::cout << "╚═══════════════════════════════════════════════════════════════╝\n";
    
    TestResult r;
    
    test_straight_ahead(r);
    test_right_turn_inner_outer(r);
    test_left_turn_inner_outer(r);
    test_ackermann_condition(r);
    test_ackermann_percentage(r);
    test_symmetry(r);
    test_xcmg_parameters(r);
    
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