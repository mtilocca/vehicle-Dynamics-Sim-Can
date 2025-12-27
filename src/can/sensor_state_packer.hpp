// src/can/sensor_state_packer.hpp
#pragma once

#include "../sensors/sensor_out.hpp"
#include <cstdint>
#include <cstring>

namespace can {

/**
 * SensorStatePacker - Packs sensor measurements into CAN frames
 * 
 * Similar to PlantStatePacker but uses SensorOut (measured values)
 * instead of PlantState (truth values).
 * 
 * This allows transmission of realistic sensor data with noise,
 * bias, quantization, etc. over the CAN bus.
 */
class SensorStatePacker {
public:
    /**
     * Pack battery sensor data
     * CAN ID: 0x200
     * 8 bytes: SOC(2) | Voltage(2) | Current(2) | Temp(1) | Valid(1)
     */
    static void pack_battery(const sensors::SensorOut& sens, uint8_t* data) {
        // SOC: 0-100% -> 0-10000 (0.01% resolution)
        uint16_t soc = static_cast<uint16_t>(sens.batt_soc_meas * 100.0);
        
        // Voltage: 0-600V -> 0-6000 (0.1V resolution)
        uint16_t voltage = static_cast<uint16_t>(sens.batt_v_meas * 10.0);
        
        // Current: -500 to +500A -> 0-10000 (offset binary, 0.1A resolution)
        int16_t current_raw = static_cast<int16_t>(sens.batt_i_meas * 10.0);
        uint16_t current = static_cast<uint16_t>(current_raw + 5000);
        
        // Temperature: -40 to +125°C -> 0-165 (1°C resolution)
        uint8_t temp = static_cast<uint8_t>(sens.batt_temp_meas + 40.0);
        
        uint8_t valid = sens.batt_valid ? 1 : 0;

        data[0] = (soc >> 8) & 0xFF;
        data[1] = soc & 0xFF;
        data[2] = (voltage >> 8) & 0xFF;
        data[3] = voltage & 0xFF;
        data[4] = (current >> 8) & 0xFF;
        data[5] = current & 0xFF;
        data[6] = temp;
        data[7] = valid;
    }

    /**
     * Pack wheel speed sensor data
     * CAN ID: 0x201
     * 8 bytes: FL(2) | FR(2) | RL(2) | RR(2)
     */
    static void pack_wheel_speeds(const sensors::SensorOut& sens, uint8_t* data) {
        // Wheel speeds: 0-200 rps -> 0-20000 (0.01 rps resolution)
        uint16_t fl = static_cast<uint16_t>(sens.wheel_fl_rps_meas * 100.0);
        uint16_t fr = static_cast<uint16_t>(sens.wheel_fr_rps_meas * 100.0);
        uint16_t rl = static_cast<uint16_t>(sens.wheel_rl_rps_meas * 100.0);
        uint16_t rr = static_cast<uint16_t>(sens.wheel_rr_rps_meas * 100.0);

        data[0] = (fl >> 8) & 0xFF;
        data[1] = fl & 0xFF;
        data[2] = (fr >> 8) & 0xFF;
        data[3] = fr & 0xFF;
        data[4] = (rl >> 8) & 0xFF;
        data[5] = rl & 0xFF;
        data[6] = (rr >> 8) & 0xFF;
        data[7] = rr & 0xFF;
    }

    /**
     * Pack IMU sensor data
     * CAN ID: 0x202
     * 8 bytes: Gyro_Yaw(2) | Accel_X(2) | Accel_Y(2) | Valid(1) | Reserved(1)
     */
    static void pack_imu(const sensors::SensorOut& sens, uint8_t* data) {
        // Gyro: -500 to +500 deg/s -> 0-10000 (offset binary, 0.1 deg/s resolution)
        int16_t gyro_raw = static_cast<int16_t>(sens.imu_gyro_yaw_rate_dps * 10.0);
        uint16_t gyro = static_cast<uint16_t>(gyro_raw + 5000);
        
        // Accel X: -20 to +20 m/s² -> 0-4000 (offset binary, 0.01 m/s² resolution)
        int16_t accel_x_raw = static_cast<int16_t>(sens.imu_accel_x_mps2 * 100.0);
        uint16_t accel_x = static_cast<uint16_t>(accel_x_raw + 2000);
        
        // Accel Y: -20 to +20 m/s² -> 0-4000
        int16_t accel_y_raw = static_cast<int16_t>(sens.imu_accel_y_mps2 * 100.0);
        uint16_t accel_y = static_cast<uint16_t>(accel_y_raw + 2000);
        
        uint8_t valid = sens.imu_valid ? 1 : 0;

        data[0] = (gyro >> 8) & 0xFF;
        data[1] = gyro & 0xFF;
        data[2] = (accel_x >> 8) & 0xFF;
        data[3] = accel_x & 0xFF;
        data[4] = (accel_y >> 8) & 0xFF;
        data[5] = accel_y & 0xFF;
        data[6] = valid;
        data[7] = 0; // Reserved
    }

    /**
     * Pack GNSS position data
     * CAN ID: 0x203
     * 8 bytes: Pos_X(3) | Pos_Y(3) | Valid(1) | Reserved(1)
     */
    static void pack_gnss_position(const sensors::SensorOut& sens, uint8_t* data) {
        // Position: -100000 to +100000 m -> 24-bit signed (0.01m resolution)
        int32_t pos_x = static_cast<int32_t>(sens.gnss_pos_x_m * 100.0);
        int32_t pos_y = static_cast<int32_t>(sens.gnss_pos_y_m * 100.0);
        
        uint8_t valid = sens.gnss_valid ? 1 : 0;

        data[0] = (pos_x >> 16) & 0xFF;
        data[1] = (pos_x >> 8) & 0xFF;
        data[2] = pos_x & 0xFF;
        data[3] = (pos_y >> 16) & 0xFF;
        data[4] = (pos_y >> 8) & 0xFF;
        data[5] = pos_y & 0xFF;
        data[6] = valid;
        data[7] = 0; // Reserved
    }

    /**
     * Pack GNSS velocity/heading data
     * CAN ID: 0x204
     * 8 bytes: Velocity(2) | Heading(2) | Altitude(2) | Valid(1) | Reserved(1)
     */
    static void pack_gnss_velocity(const sensors::SensorOut& sens, uint8_t* data) {
        // Velocity: 0-200 m/s -> 0-20000 (0.01 m/s resolution)
        uint16_t velocity = static_cast<uint16_t>(sens.gnss_velocity_mps * 100.0);
        
        // Heading: 0-360 deg -> 0-36000 (0.01 deg resolution)
        uint16_t heading = static_cast<uint16_t>(sens.gnss_heading_deg * 100.0);
        
        // Altitude: -1000 to +10000 m -> 0-11000 (1m resolution)
        uint16_t altitude = static_cast<uint16_t>(sens.gnss_altitude_m + 1000.0);
        
        uint8_t valid = sens.gnss_valid ? 1 : 0;

        data[0] = (velocity >> 8) & 0xFF;
        data[1] = velocity & 0xFF;
        data[2] = (heading >> 8) & 0xFF;
        data[3] = heading & 0xFF;
        data[4] = (altitude >> 8) & 0xFF;
        data[5] = altitude & 0xFF;
        data[6] = valid;
        data[7] = 0; // Reserved
    }

    /**
     * Pack radar sensor data
     * CAN ID: 0x205
     * 8 bytes: Range(2) | RangeRate(2) | Angle(2) | Valid(1) | Reserved(1)
     */
    static void pack_radar(const sensors::SensorOut& sens, uint8_t* data) {
        // Range: 0-300m -> 0-30000 (0.01m resolution)
        uint16_t range = static_cast<uint16_t>(sens.radar_range_m * 100.0);
        
        // Range rate: -50 to +50 m/s -> 0-10000 (offset binary, 0.01 m/s resolution)
        int16_t rate_raw = static_cast<int16_t>(sens.radar_range_rate_mps * 100.0);
        uint16_t rate = static_cast<uint16_t>(rate_raw + 5000);
        
        // Angle: -90 to +90 deg -> 0-18000 (offset binary, 0.01 deg resolution)
        int16_t angle_raw = static_cast<int16_t>(sens.radar_angle_deg * 100.0);
        uint16_t angle = static_cast<uint16_t>(angle_raw + 9000);
        
        uint8_t valid = sens.radar_valid_target ? 1 : 0;

        data[0] = (range >> 8) & 0xFF;
        data[1] = range & 0xFF;
        data[2] = (rate >> 8) & 0xFF;
        data[3] = rate & 0xFF;
        data[4] = (angle >> 8) & 0xFF;
        data[5] = angle & 0xFF;
        data[6] = valid;
        data[7] = 0; // Reserved
    }
};

} // namespace can