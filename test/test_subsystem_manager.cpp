// test/test_subsystem_manager.cpp
// Tests SubsystemManager lifecycle and priority ordering (no Battery/Wheel subsystems).
#include "plant/subsystem_manager/subsystem_manager.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/vehicle_subsystem/vehicle_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

#define COLOR_GREEN  "\033[32m"
#define COLOR_RED    "\033[31m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET  "\033[0m"

static bool test_registration() {
    std::cout << "\n" COLOR_YELLOW "=== Test 1: Subsystem Registration ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    plant::PlantState state;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::VehicleSubsystem>());

    bool pass = (mgr.subsystem_count() == 3);
    std::cout << "  Registered: " << mgr.subsystem_count() << " subsystems\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_priority_ordering() {
    std::cout << "\n" COLOR_YELLOW "=== Test 2: Priority Ordering ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;

    // Register in reverse priority order — should auto-sort
    mgr.register_subsystem(std::make_unique<plant::VehicleSubsystem>()); // 110
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());   // 100
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());   //  50

    auto* s0 = mgr.get_subsystem(0);
    auto* s1 = mgr.get_subsystem(1);
    auto* s2 = mgr.get_subsystem(2);

    bool pass = (s0 && s1 && s2) &&
                (s0->priority() < s1->priority()) &&
                (s1->priority() < s2->priority());

    std::cout << "  Execution order:\n";
    if (s0) std::cout << "    1. " << s0->name() << " (priority " << s0->priority() << ")\n";
    if (s1) std::cout << "    2. " << s1->name() << " (priority " << s1->priority() << ")\n";
    if (s2) std::cout << "    3. " << s2->name() << " (priority " << s2->priority() << ")\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_initialize_all() {
    std::cout << "\n" COLOR_YELLOW "=== Test 3: Initialize All ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    plant::PlantState state;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::VehicleSubsystem>());
    mgr.initialize_all(state);

    bool pass = (state.steer_virtual_rad == 0.0) && (state.v_mps == 0.0);
    std::cout << "  steer_virtual_rad = " << state.steer_virtual_rad << " rad\n";
    std::cout << "  v_mps = " << state.v_mps << " m/s\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_enable_disable() {
    std::cout << "\n" COLOR_YELLOW "=== Test 4: Enable/Disable ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    plant::PlantState state;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());

    // Disable Drive
    auto* drive = mgr.find_subsystem("Drive");
    if (drive) drive->set_enabled(false);

    bool pass = (mgr.subsystem_count() == 2) && (mgr.enabled_count() == 1);
    std::cout << "  Total: " << mgr.subsystem_count() << "  Enabled: " << mgr.enabled_count() << "\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_find_subsystem() {
    std::cout << "\n" COLOR_YELLOW "=== Test 5: Find Subsystem ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());

    auto* steer   = mgr.find_subsystem("Steer");
    auto* drive   = mgr.find_subsystem("Drive");
    auto* missing = mgr.find_subsystem("NotFound");

    bool pass = (steer != nullptr) && (drive != nullptr) && (missing == nullptr);
    std::cout << "  Found 'Steer':    " << (steer   ? "Yes" : "No") << "\n";
    std::cout << "  Found 'Drive':    " << (drive   ? "Yes" : "No") << "\n";
    std::cout << "  Found 'NotFound': " << (missing ? "Yes" : "No") << "\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_step_all() {
    std::cout << "\n" COLOR_YELLOW "=== Test 6: Step All ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd{};

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);

    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    mgr.step_all(state, cmd, 0.01);

    bool pass = (std::abs(state.steer_virtual_rad) > 0.0);
    std::cout << "  steer_cmd=10 deg → steer_actual="
              << (state.steer_virtual_rad * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

static bool test_reset_all() {
    std::cout << "\n" COLOR_YELLOW "=== Test 7: Reset All ===" COLOR_RESET "\n";

    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd{};

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);

    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    for (int i = 0; i < 10; ++i) {
        mgr.step_all(state, cmd, 0.01);
    }

    double before = state.steer_virtual_rad;
    mgr.reset_all(state);

    bool pass = (std::abs(before) > 0.01) && (std::abs(state.steer_virtual_rad) < 0.001);
    std::cout << "  Steer before reset: " << (before * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Steer after reset:  " << (state.steer_virtual_rad * 180.0 / 3.14159) << " deg\n";
    std::cout << "  Result: " << (pass ? COLOR_GREEN "PASS" : COLOR_RED "FAIL") << COLOR_RESET "\n";
    return pass;
}

int main() {
    std::cout << COLOR_YELLOW
              << "╔════════════════════════════════════════════════════╗\n"
              << "║   SubsystemManager Validation Tests               ║\n"
              << "╚════════════════════════════════════════════════════╝"
              << COLOR_RESET "\n";

    int passed = 0, total = 0;
    passed += test_registration();      total++;
    passed += test_priority_ordering(); total++;
    passed += test_initialize_all();    total++;
    passed += test_enable_disable();    total++;
    passed += test_find_subsystem();    total++;
    passed += test_step_all();          total++;
    passed += test_reset_all();         total++;

    std::cout << "\n" COLOR_YELLOW "═══════════════════════════════════════════════════════" COLOR_RESET "\n";
    std::cout << "  Summary: ";
    if (passed == total)
        std::cout << COLOR_GREEN << passed << "/" << total << " tests passed" << COLOR_RESET "\n";
    else
        std::cout << COLOR_RED << passed << "/" << total << " tests passed" << COLOR_RESET "\n";
    std::cout << COLOR_YELLOW "═══════════════════════════════════════════════════════" COLOR_RESET "\n\n";

    return (passed == total) ? 0 : 1;
}
