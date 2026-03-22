// test/test_steer_dynamics.cpp — GoogleTest migration

#include "plant/steer_subsystem/steer_plant.hpp"
#include "plant/steer_subsystem/steer_subsystem.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sim/actuator_cmd.hpp"
#include <gtest/gtest.h>
#include <cmath>

using namespace plant;

static double deg2rad(double d) { return d * M_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / M_PI; }

TEST(SteerDynamics, StraightAhead) {
    SteerParams p;
    p.wheelbase_m = 6.3; p.track_width_m = 4.0;
    p.ackermann_pct = 1.0;
    SteerPlant steer(p);

    PlantState s; s.v_mps = 10.0;
    sim::ActuatorCmd cmd; cmd.system_enable = true; cmd.steer_cmd_deg = 0.0;
    steer.step(s, cmd, 0.01);

    EXPECT_NEAR(s.steer_virtual_rad, 0.0, 1e-6);
    EXPECT_NEAR(s.delta_fl_rad,      0.0, 1e-6);
    EXPECT_NEAR(s.delta_fr_rad,      0.0, 1e-6);
}

TEST(SteerDynamics, RightTurnInnerOuter) {
    SteerParams p;
    p.wheelbase_m = 6.3; p.track_width_m = 4.0;
    p.delta_max_deg = 30.0; p.ackermann_pct = 1.0;
    p.steer_rate_dps = 1000.0;
    SteerPlant steer(p);

    PlantState s; s.v_mps = 5.0;
    sim::ActuatorCmd cmd; cmd.system_enable = true; cmd.steer_cmd_deg = 20.0;
    for (int i = 0; i < 10; ++i) steer.step(s, cmd, 0.01);

    EXPECT_GT(s.steer_virtual_rad, 0.0) << "virtual steering must be positive";
    EXPECT_GT(s.delta_fr_rad, s.delta_fl_rad) << "inner (FR) > outer (FL)";

    // Verify Ackermann relation: cot(outer) - cot(inner) = W/L
    double cot_diff = 1.0 / std::tan(s.delta_fl_rad) - 1.0 / std::tan(s.delta_fr_rad);
    EXPECT_NEAR(cot_diff, p.track_width_m / p.wheelbase_m, 0.01);
}

TEST(SteerDynamics, SpeedDependentLimit) {
    SteerParams p;
    p.delta_max_deg = 30.0;
    p.v_steer_limit_start_mps = 8.0;
    p.v_steer_limit_end_mps   = 25.0;
    p.steer_limit_ratio_highv = 0.40;
    p.steer_rate_dps = 1000.0;
    SteerPlant steer(p);

    sim::ActuatorCmd cmd; cmd.system_enable = true; cmd.steer_cmd_deg = 30.0;

    // Low speed — no limit
    PlantState s_low; s_low.v_mps = 5.0;
    for (int i = 0; i < 20; ++i) steer.step(s_low, cmd, 0.01);
    EXPECT_NEAR(rad2deg(s_low.steer_virtual_rad), 30.0, 1.0);

    // High speed — limited to 40% (fresh instance)
    SteerPlant steer2(p);
    PlantState s_high; s_high.v_mps = 30.0;
    for (int i = 0; i < 20; ++i) steer2.step(s_high, cmd, 0.01);
    EXPECT_NEAR(rad2deg(s_high.steer_virtual_rad), 30.0 * 0.40, 1.0);
}

TEST(SteerDynamics, RateLimiting) {
    SteerParams p;
    p.steer_rate_dps = 45.0;
    SteerPlant steer(p);

    PlantState s; s.v_mps = 5.0; s.steer_virtual_rad = 0.0;
    sim::ActuatorCmd cmd; cmd.system_enable = true; cmd.steer_cmd_deg = 30.0;
    steer.step(s, cmd, 0.01);  // one 10 ms step

    // Expected: 45°/s × 0.01 s = 0.45°
    EXPECT_NEAR(rad2deg(s.steer_virtual_rad), 0.45, 0.1);
    EXPECT_NEAR(rad2deg(s.steer_rate_radps),  45.0, 1.0);
}

TEST(SteerDynamics, SubsystemPriority) {
    SteerParams p;
    SteerSubsystem sub(p);
    EXPECT_EQ(sub.priority(), 50);
    EXPECT_EQ(std::string(sub.name()), "Steer");
}

TEST(SteerDynamics, AckermannPctBlend) {
    auto make_steer = [](double ack) {
        SteerParams p;
        p.wheelbase_m = 6.3; p.track_width_m = 4.0;
        p.ackermann_pct = ack;
        p.steer_rate_dps = 1000.0;
        return SteerPlant(p);
    };

    auto steer_full    = make_steer(1.0);
    auto steer_partial = make_steer(0.5);

    PlantState sf, sp; sf.v_mps = sp.v_mps = 5.0;
    sim::ActuatorCmd cmd; cmd.system_enable = true; cmd.steer_cmd_deg = 20.0;

    for (int i = 0; i < 20; ++i) {
        steer_full.step(sf, cmd, 0.01);
        steer_partial.step(sp, cmd, 0.01);
    }

    double diff_full    = std::abs(rad2deg(sf.delta_fr_rad) - rad2deg(sf.delta_fl_rad));
    double diff_partial = std::abs(rad2deg(sp.delta_fr_rad) - rad2deg(sp.delta_fl_rad));

    EXPECT_GT(diff_full, diff_partial) << "100% Ackermann has larger FL/FR spread";
    EXPECT_NEAR(diff_partial, diff_full * 0.5, 0.5);
}
