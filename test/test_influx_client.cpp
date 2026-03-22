// test/test_influx_client.cpp — GoogleTest migration

#include "utils/influx.hpp"
#include "plant/plant_main/plant_state.hpp"
#include "sensors/sensor_out.hpp"
#include "sim/actuator_cmd.hpp"
#include <gtest/gtest.h>
#include <cmath>

// ── Test helpers ─────────────────────────────────────────────────────────────

static plant::PlantState make_state() {
    plant::PlantState s;
    s.x_m = 100.0; s.y_m = 50.0; s.yaw_rad = 0.5; s.v_mps = 10.0;
    s.steer_virtual_rad = 0.1; s.delta_fl_rad = 0.1; s.delta_fr_rad = 0.1;
    s.motor_power_kW = 150.0; s.regen_power_kW = 0.0; s.brake_force_kN = 0.0;
    s.batt_soc_pct = 80.0; s.batt_v = 800.0; s.batt_i = 200.0;
    s.wheel_fl_rps = 5.0; s.wheel_fr_rps = 5.0;
    s.wheel_rl_rps = 5.0; s.wheel_rr_rps = 5.0;
    return s;
}

static sensors::SensorOut make_sensors() {
    sensors::SensorOut s;
    s.batt_soc_meas = 80.0; s.batt_v_meas = 800.0;
    s.batt_i_meas = 200.0; s.batt_temp_meas = 25.0;
    s.wheel_fl_rps_meas = 5.0; s.wheel_fr_rps_meas = 5.0;
    s.wheel_rl_rps_meas = 5.0; s.wheel_rr_rps_meas = 5.0;
    s.imu_valid = true; s.imu_gz_rps = 0.03;
    s.imu_ax_mps2 = 1.0; s.imu_ay_mps2 = 0.5; s.imu_az_mps2 = 9.81;
    s.gnss_valid = true; s.gnss_lat_deg = -31.9505; s.gnss_lon_deg = 115.8605;
    s.radar_valid = true; s.radar_target_range_m = 100.0;
    return s;
}

static sim::ActuatorCmd make_cmd() {
    sim::ActuatorCmd c;
    c.drive_torque_cmd_nm = 5000.0; c.brake_cmd_pct = 0.0;
    c.steer_cmd_deg = 5.0; c.system_enable = true;
    return c;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

TEST(InfluxClient, CreationDisabled) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = false;
    utils::InfluxClient client(cfg);
    EXPECT_FALSE(client.is_enabled());
}

TEST(InfluxClient, CreationEnabled) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url    = "http://localhost:8086";
    cfg.org    = "Autonomy";
    cfg.bucket = "test-bucket";
    cfg.write_interval_s = 0.1;
    utils::InfluxClient client(cfg);
    EXPECT_TRUE(client.is_enabled());
}

TEST(InfluxClient, WriteDisabledReturnsFalse) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = false;
    utils::InfluxClient client(cfg);
    EXPECT_FALSE(client.write_data_point(make_state(), make_sensors(), make_cmd(), 0.0));
}

TEST(InfluxClient, RateLimiting) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url    = "http://localhost:8086";
    cfg.org    = "Autonomy";
    cfg.bucket = "test";
    cfg.write_interval_s = 1.0;
    cfg.token  = "dummy-token";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();

    client.write_data_point(s, sn, c, 0.0);   // first attempt

    // 0.5 s later — within the 1 s interval → must skip
    bool skipped = !client.write_data_point(s, sn, c, 0.5);
    EXPECT_TRUE(skipped) << "write at 0.5 s should be skipped (interval=1 s)";

    // After 1 s interval — attempt again
    client.write_data_point(s, sn, c, 1.0);

    bool skipped2 = !client.write_data_point(s, sn, c, 1.5);
    EXPECT_TRUE(skipped2) << "write at 1.5 s should be skipped";
}

TEST(InfluxClient, ConfigDefaults) {
    utils::InfluxClient::Config cfg;
    EXPECT_FALSE(cfg.enabled);
    EXPECT_EQ(cfg.url, "http://localhost:8086");
    EXPECT_TRUE(cfg.token.empty());
    EXPECT_EQ(cfg.org, "Autonomy");
    EXPECT_EQ(cfg.bucket, "vehicle-sim");
    EXPECT_NEAR(cfg.write_interval_s, 0.25, 0.001);
}

TEST(InfluxClient, ConfigCustom) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true;
    cfg.url    = "http://192.168.1.100:8086";
    cfg.token  = "my-secret";
    cfg.org    = "MyOrg";
    cfg.bucket = "my-bucket";
    cfg.write_interval_s = 0.5;

    EXPECT_TRUE(cfg.enabled);
    EXPECT_EQ(cfg.url, "http://192.168.1.100:8086");
    EXPECT_EQ(cfg.token, "my-secret");
    EXPECT_EQ(cfg.org, "MyOrg");
    EXPECT_EQ(cfg.bucket, "my-bucket");
    EXPECT_NEAR(cfg.write_interval_s, 0.5, 0.001);
}

TEST(InfluxClient, MultipleWritesDoNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test";
    cfg.write_interval_s = 0.25; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();
    for (double t : {0.0, 0.1, 0.25, 0.3, 0.5, 0.75, 1.0})
        client.write_data_point(s, sn, c, t);
    // no crash = pass
    SUCCEED();
}

TEST(InfluxClient, FlushDoesNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test"; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    client.flush();
    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();
    client.write_data_point(s, sn, c, 0.0);
    client.flush();
    SUCCEED();
}

TEST(InfluxClient, StateChangesDoNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test";
    cfg.write_interval_s = 0.1; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();
    client.write_data_point(s, sn, c, 0.0);
    s.x_m = 200.0; s.v_mps = 20.0;
    client.write_data_point(s, sn, c, 0.1);
    SUCCEED();
}

TEST(InfluxClient, SensorValidityFlagsDoNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test";
    cfg.write_interval_s = 0.1; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto c = make_cmd();
    auto sn = make_sensors();

    sn.imu_valid = true;  sn.gnss_valid = true;  sn.radar_valid = true;
    client.write_data_point(s, sn, c, 0.0);

    sn.imu_valid = false; sn.radar_valid = false;
    client.write_data_point(s, sn, c, 0.1);

    sn.gnss_valid = false;
    client.write_data_point(s, sn, c, 0.2);
    SUCCEED();
}

TEST(InfluxClient, ExtremeValuesDoNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test";
    cfg.write_interval_s = 0.1; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();
    s.x_m = 1e6; s.y_m = -1e6; s.v_mps = 100.0; s.batt_soc_pct = 0.0;
    client.write_data_point(s, sn, c, 0.0);
    SUCCEED();
}

TEST(InfluxClient, HighFrequencyDoNotCrash) {
    utils::InfluxClient::Config cfg;
    cfg.enabled = true; cfg.url = "http://localhost:8086";
    cfg.org = "Autonomy"; cfg.bucket = "test";
    cfg.write_interval_s = 0.01; cfg.token = "dummy";
    utils::InfluxClient client(cfg);

    auto s = make_state(); auto sn = make_sensors(); auto c = make_cmd();
    for (int i = 0; i < 100; ++i) {
        s.x_m += 1.0;
        client.write_data_point(s, sn, c, i * 0.01);
    }
    SUCCEED();
}
