// test/test_subsystem_manager.cpp — GoogleTest migration

#include "plant/subsystem_manager/subsystem_manager.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/drive_subsystem/drive_subsystem.hpp"
#include "plant/battery_subsystem/battery_subsystem.hpp"
#include <gtest/gtest.h>
#include <cmath>

TEST(SubsystemManager, Registration) {
    plant::SubsystemManager mgr;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());

    EXPECT_EQ(mgr.subsystem_count(), 3u);
}

TEST(SubsystemManager, PriorityOrdering) {
    plant::SubsystemManager mgr;

    // Register in reverse priority order — must auto-sort
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());  // 150
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());    // 100
    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());    //  50

    auto* s0 = mgr.get_subsystem(0);
    auto* s1 = mgr.get_subsystem(1);
    auto* s2 = mgr.get_subsystem(2);

    EXPECT_LT(s0->priority(), s1->priority());
    EXPECT_LT(s1->priority(), s2->priority());
}

TEST(SubsystemManager, InitializeAll) {
    plant::SubsystemManager mgr;
    plant::PlantState state;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());
    mgr.initialize_all(state);

    EXPECT_EQ(state.steer_virtual_rad, 0.0);
    EXPECT_GT(state.batt_soc_pct, 0.0);
}

TEST(SubsystemManager, EnableDisable) {
    plant::SubsystemManager mgr;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::BatterySubsystem>());

    auto* battery = mgr.find_subsystem("Battery");
    ASSERT_NE(battery, nullptr);
    battery->set_enabled(false);

    EXPECT_EQ(mgr.subsystem_count(), 2u);
    EXPECT_EQ(mgr.enabled_count(), 1u);
}

TEST(SubsystemManager, FindByName) {
    plant::SubsystemManager mgr;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.register_subsystem(std::make_unique<plant::DriveSubsystem>());

    EXPECT_NE(mgr.find_subsystem("Steer"),    nullptr);
    EXPECT_NE(mgr.find_subsystem("Drive"),    nullptr);
    EXPECT_EQ(mgr.find_subsystem("NotFound"), nullptr);
}

TEST(SubsystemManager, StepAllPhases) {
    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);

    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    mgr.step_all(state, cmd, 0.01);

    EXPECT_GT(std::abs(state.steer_virtual_rad), 0.0)
        << "steer_virtual_rad should change after a step with steer command";
}

TEST(SubsystemManager, ResetAll) {
    plant::SubsystemManager mgr;
    plant::PlantState state;
    sim::ActuatorCmd cmd;

    mgr.register_subsystem(std::make_unique<plant::SteerSubsystem>());
    mgr.initialize_all(state);

    cmd.system_enable = true;
    cmd.steer_cmd_deg = 10.0;
    for (int i = 0; i < 10; ++i) mgr.step_all(state, cmd, 0.01);

    double steer_before = state.steer_virtual_rad;
    EXPECT_GT(std::abs(steer_before), 0.01) << "steering should have built up";

    mgr.reset_all(state);
    EXPECT_NEAR(state.steer_virtual_rad, 0.0, 0.001) << "reset should zero steering";
}
