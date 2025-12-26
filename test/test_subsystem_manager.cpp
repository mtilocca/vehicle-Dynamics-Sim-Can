// test/test_subsystem_manager.cpp
#include "plant/subsystem_manager.hpp"
#include "plant/steer_subsystem.hpp"
#include "plant/drive_subsystem.hpp"
#include "plant/battery_subsystem.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

// ANSI color codes
#define COLOR_GREEN "\033[32m"
#define COLOR_RED "\033[31m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET "\033[0m"

bool test_subsystem_registration() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 1: Subsystem Registration ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    
    // Register subsystems
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());
    
    bool pass = mgr.subsystem_count() == 3;
    
    std::cout << "  Registered subsystems: " << mgr.subsystem_count() << "\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_priority_ordering() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 2: Priority Ordering ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    
    // Register in reverse priority order (should auto-sort)
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>()); // Priority 150
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());   // Priority 100
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());   // Priority 50
    
    // Check sorted order
    auto* s0 = mgr.get_subsystem(0);
    auto* s1 = mgr.get_subsystem(1);
    auto* s2 = mgr.get_subsystem(2);
    
    bool pass = (s0->priority() < s1->priority()) && (s1->priority() < s2->priority());
    
    std::cout << "  Execution order:\n";
    std::cout << "    1. " << s0->name() << " (priority " << s0->priority() << ")\n";
    std::cout << "    2. " << s1->name() << " (priority " << s1->priority() << ")\n";
    std::cout << "    3. " << s2->name() << " (priority " << s2->priority() << ")\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_initialize_all() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 3: Initialize All ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());
    
    // Initialize should set default values
    mgr.initialize_all(state);
    
    bool pass = (state.steer_virtual_rad == 0.0) && 
                (state.batt_soc_pct > 0.0);
    
    std::cout << "  Steer initialized: " << state.steer_virtual_rad << " rad\n";
    std::cout << "  Battery SOC: " << state.batt_soc_pct << "%\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_enable_disable() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 4: Enable/Disable ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd;
    
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());
    
    // Disable battery
    auto* battery = mgr.find_subsystem("Battery");
    battery->set_enabled(false);
    
    bool pass = (mgr.subsystem_count() == 2) && 
                (mgr.enabled_count() == 1);
    
    std::cout << "  Total subsystems: " << mgr.subsystem_count() << "\n";
    std::cout << "  Enabled subsystems: " << mgr.enabled_count() << "\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_find_subsystem() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 5: Find Subsystem ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());
    
    auto* steer = mgr.find_subsystem("Steer");
    auto* drive = mgr.find_subsystem("Drive");
    auto* missing = mgr.find_subsystem("NotFound");
    
    bool pass = (steer != nullptr) && (drive != nullptr) && (missing == nullptr);
    
    std::cout << "  Found 'Steer': " << (steer ? "Yes" : "No") << "\n";
    std::cout << "  Found 'Drive': " << (drive ? "Yes" : "No") << "\n";
    std::cout << "  Found 'NotFound': " << (missing ? "Yes" : "No") << "\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_step_all_phases() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 6: Step All Phases ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd;
    
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);
    
    // CRITICAL: Enable system and set steering command
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    
    // Step should execute pre/step/post
    mgr.step_all(state, cmd, 0.01);
    
    // Steering should have changed (rate-limited, so not full 10 deg in one step)
    bool pass = std::abs(state.steer_virtual_rad) > 0.0;
    
    std::cout << "  Steer command: " << cmd.steer_cmd_deg << " deg\n";
    std::cout << "  Steer angle after step: " << (state.steer_virtual_rad * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

bool test_reset_all() {
    std::cout << "\n" << COLOR_YELLOW << "=== Test 7: Reset All ===" << COLOR_RESET << "\n";
    
    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd;
    
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);
    
    // CRITICAL: Enable system and change state
    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    
    // Step multiple times to build up steering angle
    for (int i = 0; i < 10; ++i) {
        mgr.step_all(state, cmd, 0.01);
    }
    
    double steer_before_reset = state.steer_virtual_rad;
    
    // Reset
    mgr.reset_all(state);
    
    bool pass = (std::abs(steer_before_reset) > 0.01) && (std::abs(state.steer_virtual_rad) < 0.001);
    
    std::cout << "  Steer before reset: " << (steer_before_reset * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Steer after reset: " << (state.steer_virtual_rad * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET << "\n";
    
    return pass;
}

int main() {
    std::cout << COLOR_YELLOW << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║        SubsystemManager Validation Tests                  ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝" << COLOR_RESET << "\n";
    
    int passed = 0;
    int total = 0;
    
    passed += test_subsystem_registration(); total++;
    passed += test_priority_ordering(); total++;
    passed += test_initialize_all(); total++;
    passed += test_enable_disable(); total++;
    passed += test_find_subsystem(); total++;
    passed += test_step_all_phases(); total++;
    passed += test_reset_all(); total++;
    
    std::cout << "\n" << COLOR_YELLOW << "═══════════════════════════════════════════════════════════" << COLOR_RESET << "\n";
    std::cout << "  " << COLOR_YELLOW << "Summary: " << COLOR_RESET;
    
    if (passed == total) {
        std::cout << COLOR_GREEN << passed << "/" << total << " tests passed ✓" << COLOR_RESET << "\n";
    } else {
        std::cout << COLOR_RED << passed << "/" << total << " tests passed ✗" << COLOR_RESET << "\n";
    }
    
    std::cout << COLOR_YELLOW << "═══════════════════════════════════════════════════════════" << COLOR_RESET << "\n\n";
    
    return (passed == total) ? 0 : 1;
}