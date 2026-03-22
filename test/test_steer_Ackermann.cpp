// test/test_steer_Ackermann.cpp — GoogleTest migration
// Standalone Ackermann geometry tests (no project headers needed)

#include <gtest/gtest.h>
#include <cmath>

static constexpr double PI = 3.14159265358979323846;
static double deg2rad(double d) { return d * PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / PI; }

// Copy of ackermann_map from steer_plant.cpp (tested in isolation)
static void ackermann_map(double steer_virtual_rad, double L, double W,
                           double ackermann_pct,
                           double& delta_fl_rad, double& delta_fr_rad) {
    if (std::abs(steer_virtual_rad) < 1e-6) {
        delta_fl_rad = delta_fr_rad = 0.0;
        return;
    }
    const double R    = L / std::tan(steer_virtual_rad);
    const double R_FL = R + W / 2.0;
    const double R_FR = R - W / 2.0;
    double fl_ack = std::atan(L / R_FL);
    double fr_ack = std::atan(L / R_FR);
    double ack = std::max(0.0, std::min(1.0, ackermann_pct));
    delta_fl_rad = ack * fl_ack + (1.0 - ack) * steer_virtual_rad;
    delta_fr_rad = ack * fr_ack + (1.0 - ack) * steer_virtual_rad;
}

TEST(Ackermann, StraightAhead) {
    double fl, fr;
    ackermann_map(0.0, 6.3, 4.0, 1.0, fl, fr);
    EXPECT_NEAR(fl, 0.0, 1e-6);
    EXPECT_NEAR(fr, 0.0, 1e-6);
}

TEST(Ackermann, RightTurnInnerOuterAngles) {
    const double delta = deg2rad(20.0);
    double fl, fr;
    ackermann_map(delta, 6.3, 4.0, 1.0, fl, fr);

    // For right turn: FR is inner → larger angle; FL is outer → smaller
    EXPECT_GT(fr, fl) << "FR(inner) must exceed FL(outer)";
    EXPECT_GT(fr, delta) << "Inner wheel turns more than virtual";
    EXPECT_LT(fl, delta) << "Outer wheel turns less than virtual";
}

TEST(Ackermann, LeftTurnInnerOuterAngles) {
    const double delta = deg2rad(-20.0);
    double fl, fr;
    ackermann_map(delta, 6.3, 4.0, 1.0, fl, fr);

    // For left turn: FL is inner → more negative
    EXPECT_LT(fl, fr) << "FL(inner) must be more negative than FR(outer)";
    EXPECT_GT(std::abs(fl), std::abs(fr)) << "|inner| > |outer|";
}

TEST(Ackermann, AckermannConditionAt10deg) {
    double fl, fr;
    ackermann_map(deg2rad(10.0), 6.3, 4.0, 1.0, fl, fr);
    double cot_diff = 1.0 / std::tan(fl) - 1.0 / std::tan(fr);
    EXPECT_NEAR(cot_diff, 4.0 / 6.3, 0.01);
}

TEST(Ackermann, AckermannConditionAt20deg) {
    double fl, fr;
    ackermann_map(deg2rad(20.0), 6.3, 4.0, 1.0, fl, fr);
    double cot_diff = 1.0 / std::tan(fl) - 1.0 / std::tan(fr);
    EXPECT_NEAR(cot_diff, 4.0 / 6.3, 0.01);
}

TEST(Ackermann, AckermannConditionAt30deg) {
    double fl, fr;
    ackermann_map(deg2rad(30.0), 6.3, 4.0, 1.0, fl, fr);
    double cot_diff = 1.0 / std::tan(fl) - 1.0 / std::tan(fr);
    EXPECT_NEAR(cot_diff, 4.0 / 6.3, 0.01);
}

TEST(Ackermann, ZeroPercentIsParallel) {
    const double delta = deg2rad(20.0);
    double fl, fr;
    ackermann_map(delta, 6.3, 4.0, 0.0, fl, fr);
    EXPECT_NEAR(fl, delta, 0.001);
    EXPECT_NEAR(fr, delta, 0.001);
}

TEST(Ackermann, HundredPercentHasDistinctAngles) {
    double fl, fr;
    ackermann_map(deg2rad(20.0), 6.3, 4.0, 1.0, fl, fr);
    EXPECT_GT(std::abs(fr - fl), 0.01);
}

TEST(Ackermann, FiftyPercentHalfDifference) {
    double fl_100, fr_100, fl_50, fr_50;
    ackermann_map(deg2rad(20.0), 6.3, 4.0, 1.0, fl_100, fr_100);
    ackermann_map(deg2rad(20.0), 6.3, 4.0, 0.5, fl_50,  fr_50);
    double diff_100 = std::abs(fr_100 - fl_100);
    double diff_50  = std::abs(fr_50  - fl_50);
    EXPECT_GT(diff_50, 0.0);
    EXPECT_LT(diff_50, diff_100);
    EXPECT_NEAR(diff_50, diff_100 * 0.5, 0.01);
}

TEST(Ackermann, LeftRightSymmetry) {
    double fl_r, fr_r, fl_l, fr_l;
    ackermann_map(deg2rad( 20.0), 6.3, 4.0, 1.0, fl_r, fr_r);
    ackermann_map(deg2rad(-20.0), 6.3, 4.0, 1.0, fl_l, fr_l);
    EXPECT_NEAR(fl_r, -fr_l, 0.001);
    EXPECT_NEAR(fr_r, -fl_l, 0.001);
}

TEST(Ackermann, XcmgMaxSteeringRealisticRadius) {
    double fl, fr;
    ackermann_map(deg2rad(35.0), 6.3, 4.0, 1.0, fl, fr);

    double R       = 6.3 / std::tan(deg2rad(35.0));
    double R_inner = R - 4.0 / 2.0;

    EXPECT_GT(R_inner, 5.0)  << "inner wheel radius should be > 5 m";
    EXPECT_LT(R_inner, 20.0) << "inner wheel radius should be < 20 m";
    EXPECT_GT(fr, fl)        << "inner (FR) > outer (FL)";
}
