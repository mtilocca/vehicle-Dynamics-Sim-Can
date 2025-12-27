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

    // --- IMU Sensor ---
    double imu_gyro_yaw_rate_dps = 0.0; // Gyro yaw rate (deg/s)
    double imu_accel_x_mps2 = 0.0;      // Longitudinal acceleration (m/s²)
    double imu_accel_y_mps2 = 0.0;      // Lateral acceleration (m/s²)
    bool imu_valid = false;

    // --- GNSS Sensor ---
    double gnss_pos_x_m = 0.0;          // Position X / Easting (m)
    double gnss_pos_y_m = 0.0;          // Position Y / Northing (m)
    double gnss_altitude_m = 0.0;       // Altitude (m)
    double gnss_velocity_mps = 0.0;     // Ground speed (m/s)
    double gnss_heading_deg = 0.0;      // Course over ground (deg)
    bool gnss_valid = false;

    // --- Radar Sensor ---
    double radar_range_m = 0.0;         // Range to target (m)
    double radar_range_rate_mps = 0.0;  // Closing velocity (m/s)
    double radar_angle_deg = 0.0;       // Angle to target (deg)
    bool radar_valid_target = false;    // Target detection valid

    // --- Status ---
    uint32_t sensor_status_flags = 0;
};

} // namespace sensors