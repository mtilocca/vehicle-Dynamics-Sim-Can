// test/test_vehicle_config.cpp
// Tests for hardcoded Heavy-Duty Electric Vehicle vehicle configuration.
// YAML loading is disabled — all tests use get_default().

#include "config/vehicle_config.hpp"
#include <gtest/gtest.h>
#include <cmath>

TEST(VehicleConfig, DefaultIsHDV) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_EQ(cfg.name, "Heavy-Duty Electric Vehicle");
    EXPECT_EQ(cfg.manufacturer, "HDV");
}

TEST(VehicleConfig, HDVMassIsCorrect) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_NEAR(cfg.params.drive.mass_kg, 218000.0, 1.0);
}

TEST(VehicleConfig, HDVGeometryIsCorrect) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_NEAR(cfg.params.wheelbase_m,          6.30, 0.01);
    EXPECT_NEAR(cfg.params.track_width_m,         7.20, 0.01);
    EXPECT_NEAR(cfg.params.drive.wheel_radius_m,  1.93, 0.01);
    EXPECT_NEAR(cfg.params.geometry.cg_height_m,  3.20, 0.01);
    EXPECT_NEAR(cfg.params.geometry.cg_to_front_m, 2.52, 0.01);
    EXPECT_NEAR(cfg.params.geometry.cg_to_rear_m,  3.78, 0.01);
}

TEST(VehicleConfig, HDVDrivetrainIsCorrect) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_NEAR(cfg.params.drive.motor_torque_max_nm, 145000.0, 1.0);
    EXPECT_NEAR(cfg.params.drive.motor_power_max_w,  2013000.0, 1.0);
    EXPECT_NEAR(cfg.params.drive.gear_ratio,              28.0, 0.01);
    EXPECT_NEAR(cfg.params.drive.brake_torque_max_nm, 180000.0, 1.0);
}

TEST(VehicleConfig, HDVBatteryIsCorrect) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_NEAR(cfg.params.battery_params.capacity_kWh,      1650.0, 1.0);
    EXPECT_NEAR(cfg.params.battery_params.nominal_voltage_v, 1200.0, 1.0);
    EXPECT_LT(cfg.params.battery_params.min_soc, cfg.params.battery_params.max_soc);
}

TEST(VehicleConfig, HDVTireIsCorrect) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_EQ(cfg.tire_params.model, "dugoff");
    EXPECT_NEAR(cfg.tire_params.surface.mu_peak,  0.72, 0.001);
    EXPECT_NEAR(cfg.tire_params.surface.mu_slide, 0.65, 0.001);
    EXPECT_GT(cfg.tire_params.Cx_base, 0.0);
    EXPECT_GT(cfg.tire_params.Cy_base, 0.0);
}

TEST(VehicleConfig, DefaultValidationPasses) {
    auto cfg = config::VehicleConfig::get_default();
    EXPECT_NO_THROW(cfg.validate());
}

TEST(VehicleConfig, LoadFallsBackToDefault) {
    // YAML loading is disabled — load() always returns HDV defaults
    auto cfg = config::VehicleConfig::load("/tmp/does_not_matter.yaml");
    EXPECT_EQ(cfg.name, "Heavy-Duty Electric Vehicle");
    EXPECT_NEAR(cfg.params.drive.mass_kg, 218000.0, 1.0);
}
