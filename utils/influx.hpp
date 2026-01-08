// utils/influx.hpp
#pragma once

#include "plant/plant_state.hpp"
#include "sensors/sensor_out.hpp"
#include "sim/actuator_cmd.hpp"
#include <string>
#include <memory>

namespace utils {

/**
 * InfluxDB Client for time-series logging of vehicle dynamics simulation
 * 
 * Logs same data as CSV but to InfluxDB for real-time monitoring and analysis.
 * Only enabled when --influx flag is set AND simulation is in real-time mode.
 * 
 * Write interval: 250ms (4Hz) - balances granularity with network overhead
 * 
 * Organization: Autonomy
 * Bucket: vehicle-sim
 * 
 * Measurement schema:
 *   - vehicle_truth: Ground truth plant state
 *   - battery_sensors: Battery sensor measurements
 *   - wheel_sensors: Wheel speed sensor measurements
 *   - imu_sensors: IMU 6-DOF measurements
 *   - gnss_sensors: GNSS position/velocity measurements
 *   - radar_sensors: Radar target tracking measurements
 */
class InfluxClient {
public:
    struct Config {
        std::string url = "http://localhost:8086";  // InfluxDB server URL
        std::string token = "";                      // Authentication token (optional for local)
        std::string org = "Autonomy";                // Organization name
        std::string bucket = "vehicle-sim";          // Bucket name
        double write_interval_s = 0.25;              // 250ms = 4Hz
        bool enabled = false;                        // Only enabled with --influx flag
    };
    
    /**
     * Initialize InfluxDB client
     * 
     * @param config InfluxDB configuration
     * @throws std::runtime_error if connection fails
     */
    explicit InfluxClient(const Config& config);
    
    /**
     * Destructor - flushes any pending writes
     */
    ~InfluxClient();
    
    /**
     * Write simulation data point to InfluxDB
     * 
     * Only writes if:
     *   - enabled flag is true
     *   - simulation is in real-time mode
     *   - sufficient time has elapsed since last write (write_interval_s)
     * 
     * @param state Ground truth plant state
     * @param sensor_out All sensor measurements
     * @param cmd Current actuator commands
     * @param sim_time Current simulation time
     * @return true if data was written, false if skipped (rate limiting)
     */
    bool write_data_point(const plant::PlantState& state,
                         const sensors::SensorOut& sensor_out,
                         const sim::ActuatorCmd& cmd,
                         double sim_time);
    
    /**
     * Flush any buffered writes immediately
     */
    void flush();
    
    /**
     * Check if client is enabled and connected
     */
    bool is_enabled() const { return config_.enabled; }
    
    /**
     * Get current configuration
     */
    const Config& get_config() const { return config_; }

private:
    Config config_;
    double last_write_time_;
    
    // Implementation details hidden (pimpl pattern)
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    /**
     * Build line protocol string for vehicle truth data
     * 
     * Measurement: vehicle_truth
     * Tags: none (single vehicle simulation)
     * Fields: All ground truth state variables
     */
    std::string build_vehicle_truth_line(const plant::PlantState& state,
                                         const sim::ActuatorCmd& cmd,
                                         int64_t timestamp_ns);
    
    /**
     * Build line protocol string for battery sensor data
     * 
     * Measurement: battery_sensors
     * Fields: SOC, voltage, current, temperature (truth + measured)
     */
    std::string build_battery_sensors_line(const plant::PlantState& state,
                                           const sensors::SensorOut& sensor_out,
                                           int64_t timestamp_ns);
    
    /**
     * Build line protocol string for wheel sensor data
     * 
     * Measurement: wheel_sensors
     * Fields: All 4 wheel speeds (truth + measured)
     */
    std::string build_wheel_sensors_line(const plant::PlantState& state,
                                         const sensors::SensorOut& sensor_out,
                                         int64_t timestamp_ns);
    
    /**
     * Build line protocol string for IMU sensor data
     * 
     * Measurement: imu_sensors
     * Fields: 3-axis gyro, 3-axis accel, temperature
     */
    std::string build_imu_sensors_line(const sensors::SensorOut& sensor_out,
                                       int64_t timestamp_ns);
    
    /**
     * Build line protocol string for GNSS sensor data
     * 
     * Measurement: gnss_sensors
     * Fields: lat/lon/alt, velocity NE, fix type, sat count
     */
    std::string build_gnss_sensors_line(const sensors::SensorOut& sensor_out,
                                        int64_t timestamp_ns);
    
    /**
     * Build line protocol string for radar sensor data
     * 
     * Measurement: radar_sensors
     * Fields: target range, relative velocity, angle
     */
    std::string build_radar_sensors_line(const sensors::SensorOut& sensor_out,
                                         int64_t timestamp_ns);
    
    /**
     * Send line protocol data to InfluxDB
     * 
     * @param line_protocol Concatenated line protocol strings
     * @return true if write succeeded, false otherwise
     */
    bool send_to_influx(const std::string& line_protocol);
    
    /**
     * Convert simulation time to nanosecond timestamp
     */
    int64_t sim_time_to_ns(double sim_time_s);
};

} // namespace utils