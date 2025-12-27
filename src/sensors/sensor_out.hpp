// src/sensors/sensor_out.hpp
#pragma once

#include <cstdint>

namespace sensors {

/**
 * SensorOut - Container for all sensor measurements
 * 
 * This is the "measured" counterpart to PlantState (truth).
 * All values include sensor noise, bias, quantization, etc.
 */
struct SensorOut {
    // --- Time ---
    double t_s = 0.0;

    // --- Battery Sensors ---
    double batt_v_meas = 0.0;           // Measured voltage (V)
    double batt_i_meas = 0.0;           // Measured current (A)
    double batt_soc_meas = 0.0;         // Measured SOC (0-100%)
    double batt_temp_meas = 25.0;       // Measured temperature (°C)
    bool batt_valid = false;

    // --- Wheel Speed Sensors (encoder ticks/sec or rad/s) ---
    double wheel_fl_rps_meas = 0.0;
    double wheel_fr_rps_meas = 0.0;
    double wheel_rl_rps_meas = 0.0;
    double wheel_rr_rps_meas = 0.0;
    bool wheel_valid = false;

    // --- IMU/GNSS (future) ---
    double accel_x_meas = 0.0;          // m/s²
    double accel_y_meas = 0.0;
    double accel_z_meas = 0.0;
    double gyro_x_meas = 0.0;           // rad/s
    double gyro_y_meas = 0.0;
    double gyro_z_meas = 0.0;
    double gnss_lat_meas = 0.0;         // degrees
    double gnss_lon_meas = 0.0;
    double gnss_alt_meas = 0.0;         // meters
    bool imu_valid = false;
    bool gnss_valid = false;

    // --- Radar (future) ---
    double radar_range_meas = 0.0;      // meters
    double radar_azimuth_meas = 0.0;    // radians
    bool radar_valid = false;

    // --- Status ---
    uint32_t sensor_status_flags = 0;
};

} // namespace sensors