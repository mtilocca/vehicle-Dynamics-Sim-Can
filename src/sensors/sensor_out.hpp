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

    // === IMU Sensor (6-DOF: 3-axis gyro + 3-axis accel) ===
    // Gyroscope (angular velocity in rad/s)
    double imu_gx_rps = 0.0;            // Roll rate (rad/s)
    double imu_gy_rps = 0.0;            // Pitch rate (rad/s)
    double imu_gz_rps = 0.0;            // Yaw rate (rad/s)
    
    // Accelerometer (linear acceleration in m/s²)
    double imu_ax_mps2 = 0.0;           // Longitudinal / X-axis (m/s²)
    double imu_ay_mps2 = 0.0;           // Lateral / Y-axis (m/s²)
    double imu_az_mps2 = 0.0;           // Vertical / Z-axis (m/s²)
    
    // IMU metadata
    double imu_temp_c = 25.0;           // Temperature (°C)
    uint8_t imu_status = 0;             // Status flags (0=OK)
    bool imu_valid = false;

    // === GNSS Sensor (GPS) ===
    // Position (WGS84 geodetic coordinates)
    double gnss_lat_deg = 0.0;          // Latitude (deg, -90 to +90)
    double gnss_lon_deg = 0.0;          // Longitude (deg, -180 to +180)
    double gnss_alt_m = 0.0;            // Altitude MSL (m)
    
    // Velocity (NED frame components)
    double gnss_vn_mps = 0.0;           // Velocity North (m/s)
    double gnss_ve_mps = 0.0;           // Velocity East (m/s)
    
    // GNSS quality indicators
    uint8_t gnss_fix_type = 0;          // Fix type (0=no fix, 3=3D fix)
    uint8_t gnss_sat_count = 0;         // Satellites in use
    bool gnss_valid = false;

    // === Radar Sensor ===
    double radar_target_range_m = 0.0;      // Range to target (m)
    double radar_target_rel_vel_mps = 0.0;  // Relative velocity (m/s, + = approaching)
    double radar_target_angle_deg = 0.0;    // Azimuth angle (deg)
    uint8_t radar_status = 0;               // Status flags (bit 0 = target valid)
    bool radar_valid = false;

    // --- Status ---
    uint32_t sensor_status_flags = 0;
};

} // namespace sensors