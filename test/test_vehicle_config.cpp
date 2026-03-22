// test/test_vehicle_config.cpp — GoogleTest migration

#include "config/vehicle_config.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <cmath>

TEST(VehicleConfig, DefaultConfigIsValid) {
    auto cfg = config::VehicleConfig::get_default();

    EXPECT_GT(cfg.params.drive.mass_kg,               0.0);
    EXPECT_GT(cfg.params.drive.motor_power_max_w,     0.0);
    EXPECT_GT(cfg.params.battery_params.capacity_kWh, 0.0);
    EXPECT_GT(cfg.params.wheelbase_m,                 0.0);
    EXPECT_GT(cfg.params.track_width_m,               0.0);
}

TEST(VehicleConfig, ValidYamlLoads) {
    const char* path = "/tmp/gtest_vehicle_valid.yaml";
    {
        std::ofstream f(path);
        f << R"(
vehicle:
  name: "Test Vehicle"
  geometry:
    mass_kg: 2000.0
    wheelbase_m: 3.0
    track_width_m: 1.7
    wheel_radius_m: 0.35
  drivetrain:
    motor_power_max_w: 200000.0
    motor_torque_max_nm: 2500.0
    gear_ratio: 10.0
    drivetrain_eff: 0.95
  battery:
    capacity_kwh: 100.0
    nominal_voltage: 400.0
    initial_soc: 0.80
    max_power_kw: 150.0
    efficiency_charge: 0.95
    efficiency_discharge: 0.95
    min_soc: 0.05
    max_soc: 0.95
  resistance:
    drag_coefficient: 0.30
    rolling_resistance: 35.0
  limits:
    v_max_mps: 50.0
    v_stop_eps: 0.3
)";
    }

    ASSERT_NO_THROW({
        auto cfg = config::VehicleConfig::load(path);
        EXPECT_EQ(cfg.name, "Test Vehicle");
        EXPECT_NEAR(cfg.params.drive.mass_kg,               2000.0,  1.0);
        EXPECT_NEAR(cfg.params.drive.motor_power_max_w, 200000.0, 100.0);
        EXPECT_NEAR(cfg.params.battery_params.capacity_kWh, 100.0,    1.0);
        EXPECT_NEAR(cfg.params.wheelbase_m,                   3.0,  0.01);
    });

    std::remove(path);
}

TEST(VehicleConfig, MissingFileFallsBackToDefaults) {
    auto cfg = config::VehicleConfig::load("/tmp/definitely_does_not_exist.yaml");
    EXPECT_GT(cfg.params.drive.mass_kg,               0.0);
    EXPECT_GT(cfg.params.battery_params.capacity_kWh, 0.0);
}

TEST(VehicleConfig, NegativeMassThrows) {
    const char* path = "/tmp/gtest_vehicle_neg_mass.yaml";
    {
        std::ofstream f(path);
        f << "vehicle:\n  geometry:\n    mass_kg: -1000.0\n"
             "  drivetrain:\n    motor_power_max_w: 200000.0\n"
             "  battery:\n    capacity_kwh: 100.0\n";
    }

    EXPECT_THROW(config::VehicleConfig::load(path), std::exception);
    std::remove(path);
}

TEST(VehicleConfig, InvalidSocRangeThrows) {
    const char* path = "/tmp/gtest_vehicle_soc.yaml";
    {
        std::ofstream f(path);
        f << "vehicle:\n  geometry:\n    mass_kg: 2000.0\n"
             "  drivetrain:\n    motor_power_max_w: 200000.0\n"
             "  battery:\n    capacity_kWh: 100.0\n"
             "    min_soc: 0.80\n    max_soc: 0.60\n";
    }

    EXPECT_THROW(config::VehicleConfig::load(path), std::exception);
    std::remove(path);
}

TEST(VehicleConfig, ZeroMotorPowerThrows) {
    const char* path = "/tmp/gtest_vehicle_zero_power.yaml";
    {
        std::ofstream f(path);
        f << "vehicle:\n  geometry:\n    mass_kg: 2000.0\n"
             "  drivetrain:\n    motor_power_max_w: 0.0\n"
             "  battery:\n    capacity_kWh: 100.0\n";
    }

    EXPECT_THROW(config::VehicleConfig::load(path), std::exception);
    std::remove(path);
}
