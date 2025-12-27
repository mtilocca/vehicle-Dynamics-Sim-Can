// src/can/sensor_state_packer.hpp
#pragma once

#include "../sensors/sensor_out.hpp"
#include <cstdint>
#include <cstring>

namespace can {

/**
 * SensorStatePacker - Packs sensor measurements into CAN frames
 * 
 * Maps SensorOut fields to CAN frames according to can_map.csv:
 * - 0x200: IMU_ACC (accelerometer 3-axis + temp)
 * - 0x201: IMU_GYR (gyroscope 3-axis + status)
 * - 0x210: GNSS_LL (latitude + longitude)
 * - 0x211: GNSS_AV (altitude + velocity north/east + fix + sats)
 * - 0x220: WHEELS_1 (4 wheel speeds) - handled by existing code
 * - 0x230: BATT_STATE (battery) - handled by existing code
 * - 0x240: RADAR_1 (range + velocity + angle + status)
 */
class SensorStatePacker {
public:
    /**
     * Pack IMU accelerometer data
     * CAN ID: 0x200 (IMU_ACC)
     * Signal layout per can_map.csv:
     *   imu_ax_mps2  : start_bit=0,  bit_length=16, signed, factor=0.01
     *   imu_ay_mps2  : start_bit=16, bit_length=16, signed, factor=0.01
     *   imu_az_mps2  : start_bit=32, bit_length=16, signed, factor=0.01
     *   imu_temp_c   : start_bit=48, bit_length=16, signed, factor=0.01
     */
    static void pack_imu_acc(const sensors::SensorOut& sens, uint8_t* data) {
        // Accel X: -50 to +50 m/s² with 0.01 resolution
        int16_t ax = static_cast<int16_t>(sens.imu_ax_mps2 / 0.01);
        
        // Accel Y: -50 to +50 m/s² with 0.01 resolution
        int16_t ay = static_cast<int16_t>(sens.imu_ay_mps2 / 0.01);
        
        // Accel Z: -50 to +50 m/s² with 0.01 resolution
        int16_t az = static_cast<int16_t>(sens.imu_az_mps2 / 0.01);
        
        // Temperature: -40 to +125°C with 0.01 resolution
        int16_t temp = static_cast<int16_t>(sens.imu_temp_c / 0.01);
        
        // Pack little-endian
        data[0] = ax & 0xFF;
        data[1] = (ax >> 8) & 0xFF;
        data[2] = ay & 0xFF;
        data[3] = (ay >> 8) & 0xFF;
        data[4] = az & 0xFF;
        data[5] = (az >> 8) & 0xFF;
        data[6] = temp & 0xFF;
        data[7] = (temp >> 8) & 0xFF;
    }

    /**
     * Pack IMU gyroscope data
     * CAN ID: 0x201 (IMU_GYR)
     * Signal layout per can_map.csv:
     *   imu_gx_rps   : start_bit=0,  bit_length=16, signed, factor=0.001
     *   imu_gy_rps   : start_bit=16, bit_length=16, signed, factor=0.001
     *   imu_gz_rps   : start_bit=32, bit_length=16, signed, factor=0.001
     *   imu_status   : start_bit=48, bit_length=8,  unsigned
     */
    static void pack_imu_gyr(const sensors::SensorOut& sens, uint8_t* data) {
        // Gyro X: -10 to +10 rad/s with 0.001 resolution
        int16_t gx = static_cast<int16_t>(sens.imu_gx_rps / 0.001);
        
        // Gyro Y: -10 to +10 rad/s with 0.001 resolution
        int16_t gy = static_cast<int16_t>(sens.imu_gy_rps / 0.001);
        
        // Gyro Z: -10 to +10 rad/s with 0.001 resolution
        int16_t gz = static_cast<int16_t>(sens.imu_gz_rps / 0.001);
        
        // Status flags
        uint8_t status = sens.imu_status;
        
        // Pack little-endian
        data[0] = gx & 0xFF;
        data[1] = (gx >> 8) & 0xFF;
        data[2] = gy & 0xFF;
        data[3] = (gy >> 8) & 0xFF;
        data[4] = gz & 0xFF;
        data[5] = (gz >> 8) & 0xFF;
        data[6] = status;
        data[7] = 0;  // Reserved
    }

    /**
     * Pack GNSS lat/lon data
     * CAN ID: 0x210 (GNSS_LL)
     * Signal layout per can_map.csv:
     *   gnss_lat_deg : start_bit=0,  bit_length=32, signed, factor=1e-7
     *   gnss_lon_deg : start_bit=32, bit_length=32, signed, factor=1e-7
     */
    static void pack_gnss_ll(const sensors::SensorOut& sens, uint8_t* data) {
        // Latitude: -90 to +90 deg with 1e-7 resolution (~1.1 cm precision)
        int32_t lat = static_cast<int32_t>(sens.gnss_lat_deg / 1e-7);
        
        // Longitude: -180 to +180 deg with 1e-7 resolution
        int32_t lon = static_cast<int32_t>(sens.gnss_lon_deg / 1e-7);
        
        // Pack little-endian (4 bytes each)
        data[0] = lat & 0xFF;
        data[1] = (lat >> 8) & 0xFF;
        data[2] = (lat >> 16) & 0xFF;
        data[3] = (lat >> 24) & 0xFF;
        data[4] = lon & 0xFF;
        data[5] = (lon >> 8) & 0xFF;
        data[6] = (lon >> 16) & 0xFF;
        data[7] = (lon >> 24) & 0xFF;
    }

    /**
     * Pack GNSS altitude/velocity data
     * CAN ID: 0x211 (GNSS_AV)
     * Signal layout per can_map.csv:
     *   gnss_alt_m      : start_bit=0,  bit_length=16, signed, factor=0.1, offset=-1000
     *   gnss_vn_mps     : start_bit=16, bit_length=16, signed, factor=0.01
     *   gnss_ve_mps     : start_bit=32, bit_length=16, signed, factor=0.01
     *   gnss_fix_type   : start_bit=48, bit_length=8,  unsigned
     *   gnss_sat_count  : start_bit=56, bit_length=8,  unsigned
     */
    static void pack_gnss_av(const sensors::SensorOut& sens, uint8_t* data) {
        // Altitude: -1000 to +8000m with 0.1m resolution, offset=-1000
        // raw_value = (phys_value - offset) / factor
        int16_t alt = static_cast<int16_t>((sens.gnss_alt_m - (-1000.0)) / 0.1);
        
        // Velocity North: -200 to +200 m/s with 0.01 resolution
        int16_t vn = static_cast<int16_t>(sens.gnss_vn_mps / 0.01);
        
        // Velocity East: -200 to +200 m/s with 0.01 resolution
        int16_t ve = static_cast<int16_t>(sens.gnss_ve_mps / 0.01);
        
        // Fix type and satellite count
        uint8_t fix_type = sens.gnss_fix_type;
        uint8_t sat_count = sens.gnss_sat_count;
        
        // Pack little-endian
        data[0] = alt & 0xFF;
        data[1] = (alt >> 8) & 0xFF;
        data[2] = vn & 0xFF;
        data[3] = (vn >> 8) & 0xFF;
        data[4] = ve & 0xFF;
        data[5] = (ve >> 8) & 0xFF;
        data[6] = fix_type;
        data[7] = sat_count;
    }

    /**
     * Pack radar sensor data
     * CAN ID: 0x240 (RADAR_1)
     * Signal layout per can_map.csv:
     *   radar_target_range_m     : start_bit=0,  bit_length=16, unsigned, factor=0.1
     *   radar_target_rel_vel_mps : start_bit=16, bit_length=16, signed,   factor=0.01
     *   radar_target_angle_deg   : start_bit=32, bit_length=16, signed,   factor=0.1
     *   radar_status             : start_bit=48, bit_length=8,  unsigned
     */
    static void pack_radar(const sensors::SensorOut& sens, uint8_t* data) {
        // Range: 0 to 1000m with 0.1m resolution
        uint16_t range = static_cast<uint16_t>(sens.radar_target_range_m / 0.1);
        
        // Relative velocity: -200 to +200 m/s with 0.01 resolution
        int16_t rel_vel = static_cast<int16_t>(sens.radar_target_rel_vel_mps / 0.01);
        
        // Angle: -90 to +90 deg with 0.1 resolution
        int16_t angle = static_cast<int16_t>(sens.radar_target_angle_deg / 0.1);
        
        // Status flags
        uint8_t status = sens.radar_status;
        
        // Pack little-endian
        data[0] = range & 0xFF;
        data[1] = (range >> 8) & 0xFF;
        data[2] = rel_vel & 0xFF;
        data[3] = (rel_vel >> 8) & 0xFF;
        data[4] = angle & 0xFF;
        data[5] = (angle >> 8) & 0xFF;
        data[6] = status;
        data[7] = 0;  // Reserved
    }

    /**
     * Pack battery sensor data
     * CAN ID: 0x230 (BATT_STATE)
     * This is handled by PlantStatePacker in existing code,
     * but included here for completeness if sensor measurements are preferred.
     * 
     * Signal layout per can_map.csv:
     *   batt_v          : start_bit=0,  bit_length=16, unsigned, factor=0.1
     *   batt_i          : start_bit=16, bit_length=16, signed,   factor=0.1
     *   batt_soc_pct    : start_bit=32, bit_length=8,  unsigned, factor=0.5
     *   batt_temp_c     : start_bit=40, bit_length=8,  unsigned, factor=1, offset=-40
     *   batt_power_kw   : start_bit=48, bit_length=16, signed,   factor=0.1
     */
    static void pack_battery(const sensors::SensorOut& sens, uint8_t* data) {
        // Voltage: 0-1000V with 0.1V resolution
        uint16_t voltage = static_cast<uint16_t>(sens.batt_v_meas / 0.1);
        
        // Current: -2000 to +2000A with 0.1A resolution
        int16_t current = static_cast<int16_t>(sens.batt_i_meas / 0.1);
        
        // SOC: 0-100% with 0.5% resolution
        uint8_t soc = static_cast<uint8_t>(sens.batt_soc_meas / 0.5);
        
        // Temperature: -40 to +125°C with 1°C resolution, offset=-40
        uint8_t temp = static_cast<uint8_t>(sens.batt_temp_meas - (-40.0));
        
        // Power: V * I (calculated)
        double power_kw = (sens.batt_v_meas * sens.batt_i_meas) / 1000.0;
        int16_t power = static_cast<int16_t>(power_kw / 0.1);
        
        // Pack little-endian
        data[0] = voltage & 0xFF;
        data[1] = (voltage >> 8) & 0xFF;
        data[2] = current & 0xFF;
        data[3] = (current >> 8) & 0xFF;
        data[4] = soc;
        data[5] = temp;
        data[6] = power & 0xFF;
        data[7] = (power >> 8) & 0xFF;
    }

    /**
     * Pack wheel speed sensor data
     * CAN ID: 0x220 (WHEELS_1)
     * 
     * Signal layout per can_map.csv:
     *   wheel_fl_rps : start_bit=0,  bit_length=16, signed, factor=0.01
     *   wheel_fr_rps : start_bit=16, bit_length=16, signed, factor=0.01
     *   wheel_rl_rps : start_bit=32, bit_length=16, signed, factor=0.01
     *   wheel_rr_rps : start_bit=48, bit_length=16, signed, factor=0.01
     */
    static void pack_wheel_speeds(const sensors::SensorOut& sens, uint8_t* data) {
        // All wheels: -300 to +300 rad/s with 0.01 resolution
        int16_t fl = static_cast<int16_t>(sens.wheel_fl_rps_meas / 0.01);
        int16_t fr = static_cast<int16_t>(sens.wheel_fr_rps_meas / 0.01);
        int16_t rl = static_cast<int16_t>(sens.wheel_rl_rps_meas / 0.01);
        int16_t rr = static_cast<int16_t>(sens.wheel_rr_rps_meas / 0.01);
        
        // Pack little-endian
        data[0] = fl & 0xFF;
        data[1] = (fl >> 8) & 0xFF;
        data[2] = fr & 0xFF;
        data[3] = (fr >> 8) & 0xFF;
        data[4] = rl & 0xFF;
        data[5] = (rl >> 8) & 0xFF;
        data[6] = rr & 0xFF;
        data[7] = (rr >> 8) & 0xFF;
    }
};

} // namespace can