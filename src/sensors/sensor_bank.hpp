// src/sensors/sensor_bank.hpp
#pragma once

#include "sensors/sensor_out.hpp"
#include "sensors/battery_sensor.hpp"
#include "sensors/wheel_sensor.hpp"
#include "sensors/imu_sensor.hpp"
#include "sensors/gnss_sensor.hpp"
#include "sensors/radar_sensor.hpp"
#include <memory>

namespace sensors {

/**
 * Configuration for sensor bank
 */
struct SensorBankConfig {
    bool enable_battery_sensor = true;
    bool enable_wheel_sensor = true;
    bool enable_imu_sensor = false;
    bool enable_gnss_sensor = false;
    bool enable_radar_sensor = false;
    
    BatterySensorParams battery_params{};
    WheelSensorParams wheel_params{};
    IMUSensorParams imu_params{};
    GNSSSensorParams gnss_params{};
    RadarSensorParams radar_params{};
};

/**
 * SensorBank - Manages all vehicle sensors
 */
class SensorBank {
public:
    SensorBank(const SensorBankConfig& cfg)
        : config_(cfg)
    {
        // Create enabled sensors
        if (config_.enable_battery_sensor) {
            battery_sensor_ = std::make_unique<BatterySensor>(config_.battery_params);
        }
        if (config_.enable_wheel_sensor) {
            wheel_sensor_ = std::make_unique<WheelSensor>(config_.wheel_params);
        }
        if (config_.enable_imu_sensor) {
            imu_sensor_ = std::make_unique<IMUSensor>(config_.imu_params);
        }
        if (config_.enable_gnss_sensor) {
            gnss_sensor_ = std::make_unique<GNSSSensor>(config_.gnss_params);
        }
        if (config_.enable_radar_sensor) {
            radar_sensor_ = std::make_unique<RadarSensor>(config_.radar_params);
        }
    }

    /**
     * Step all sensors with current truth state
     */
    void step(double t, const plant::PlantState& truth, double dt) {
        if (battery_sensor_) battery_sensor_->step(t, truth, dt);
        if (wheel_sensor_) wheel_sensor_->step(t, truth, dt);
        if (imu_sensor_) imu_sensor_->step(t, truth, dt);
        if (gnss_sensor_) gnss_sensor_->step(t, truth, dt);
        if (radar_sensor_) radar_sensor_->step(t, truth, dt);
    }

    /**
     * Get aggregated sensor outputs
     * Merges outputs from all enabled sensors into one SensorOut
     */
    SensorOut get_output(double t) const {
        SensorOut out;
        out.t_s = t;

        // Merge battery sensor output
        if (battery_sensor_) {
            SensorOut batt_out = battery_sensor_->get_output();
            out.batt_soc_meas = batt_out.batt_soc_meas;
            out.batt_v_meas = batt_out.batt_v_meas;
            out.batt_i_meas = batt_out.batt_i_meas;
            out.batt_temp_meas = batt_out.batt_temp_meas;
            out.batt_valid = batt_out.batt_valid;
        }

        // Merge wheel sensor output
        if (wheel_sensor_) {
            SensorOut wheel_out = wheel_sensor_->get_output();
            out.wheel_fl_rps_meas = wheel_out.wheel_fl_rps_meas;
            out.wheel_fr_rps_meas = wheel_out.wheel_fr_rps_meas;
            out.wheel_rl_rps_meas = wheel_out.wheel_rl_rps_meas;
            out.wheel_rr_rps_meas = wheel_out.wheel_rr_rps_meas;
            out.wheel_valid = wheel_out.wheel_valid;
        }

        // Merge IMU sensor output
        if (imu_sensor_) {
            SensorOut imu_out = imu_sensor_->get_output();
            out.imu_gx_rps = imu_out.imu_gx_rps;
            out.imu_gy_rps = imu_out.imu_gy_rps;
            out.imu_gz_rps = imu_out.imu_gz_rps;
            out.imu_ax_mps2 = imu_out.imu_ax_mps2;
            out.imu_ay_mps2 = imu_out.imu_ay_mps2;
            out.imu_az_mps2 = imu_out.imu_az_mps2;
            out.imu_temp_c = imu_out.imu_temp_c;
            out.imu_status = imu_out.imu_status;
            out.imu_valid = imu_out.imu_valid;
        }

        // Merge GNSS sensor output
        if (gnss_sensor_) {
            SensorOut gnss_out = gnss_sensor_->get_output();
            out.gnss_lat_deg = gnss_out.gnss_lat_deg;
            out.gnss_lon_deg = gnss_out.gnss_lon_deg;
            out.gnss_alt_m = gnss_out.gnss_alt_m;
            out.gnss_vn_mps = gnss_out.gnss_vn_mps;
            out.gnss_ve_mps = gnss_out.gnss_ve_mps;
            out.gnss_fix_type = gnss_out.gnss_fix_type;
            out.gnss_sat_count = gnss_out.gnss_sat_count;
            out.gnss_valid = gnss_out.gnss_valid;
        }

        // Merge radar sensor output
        if (radar_sensor_) {
            SensorOut radar_out = radar_sensor_->get_output();
            out.radar_target_range_m = radar_out.radar_target_range_m;
            out.radar_target_rel_vel_mps = radar_out.radar_target_rel_vel_mps;
            out.radar_target_angle_deg = radar_out.radar_target_angle_deg;
            out.radar_status = radar_out.radar_status;
            out.radar_valid = radar_out.radar_valid;
        }

        return out;
    }

    /**
     * Reset all sensors
     */
    void reset() {
        if (battery_sensor_) battery_sensor_->reset();
        if (wheel_sensor_) wheel_sensor_->reset();
        if (imu_sensor_) imu_sensor_->reset();
        if (gnss_sensor_) gnss_sensor_->reset();
        if (radar_sensor_) radar_sensor_->reset();
    }

    /**
     * Get count of enabled sensors
     */
    size_t sensor_count() const {
        size_t count = 0;
        if (battery_sensor_) count++;
        if (wheel_sensor_) count++;
        if (imu_sensor_) count++;
        if (gnss_sensor_) count++;
        if (radar_sensor_) count++;
        return count;
    }

    /**
     * Set radar weather condition
     */
    void set_radar_weather(RadarWeatherCondition weather) {
        if (radar_sensor_) {
            radar_sensor_->set_weather(weather);
        }
    }

private:
    SensorBankConfig config_;
    
    std::unique_ptr<BatterySensor> battery_sensor_;
    std::unique_ptr<WheelSensor> wheel_sensor_;
    std::unique_ptr<IMUSensor> imu_sensor_;
    std::unique_ptr<GNSSSensor> gnss_sensor_;
    std::unique_ptr<RadarSensor> radar_sensor_;
};

} // namespace sensors