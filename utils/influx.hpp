// utils/influx.hpp
#pragma once

#include <string>
#include <memory>
#include <cstdint>
#include "plant/plant_state.hpp"
#include "sensors/sensor_out.hpp"
#include "sim/actuator_cmd.hpp"

namespace utils {

/**
 * InfluxDB Line Protocol Client for Time-Series Logging
 * 
 * Writes simulation data to InfluxDB v2 using the Line Protocol format over HTTP.
 * Supports authentication, configurable write intervals, and multiple measurements.
 * 
 * Features:
 * - Real-time data logging (wall clock timestamps)
 * - Rate-limited writes (configurable interval)
 * - Multiple measurements: vehicle_truth, battery_sensors, wheel_sensors, 
 *   imu_sensors, gnss_sensors, radar_sensors
 * - HTTP authentication with bearer tokens
 * - Matching field names with CSV logger
 * 
 * Usage:
 *   InfluxClient::Config config;
 *   config.enabled = true;
 *   config.url = "http://localhost:8086";
 *   config.token = "your-token";
 *   config.org = "Autonomy";
 *   config.bucket = "vehicle-sim";
 *   config.write_interval_s = 0.25;  // 4Hz
 *   
 *   InfluxClient client(config);
 *   
 *   // In simulation loop
 *   client.write_data_point(state, sensor_out, cmd, sim_time);
 *   
 *   // At end
 *   client.flush();
 */
class InfluxClient {
public:
    struct Config {
        bool enabled = false;
        std::string url = "http://localhost:8086";
        std::string token = "";  // Empty = no authentication (local only)
        std::string org = "Autonomy";
        std::string bucket = "vehicle-sim";
        double write_interval_s = 0.25;  // 250ms = 4Hz
    };
    
    explicit InfluxClient(const Config& config);
    ~InfluxClient();
    
    // Delete copy/move to ensure single ownership of CURL handle
    InfluxClient(const InfluxClient&) = delete;
    InfluxClient& operator=(const InfluxClient&) = delete;
    InfluxClient(InfluxClient&&) = delete;
    InfluxClient& operator=(InfluxClient&&) = delete;
    
    /**
     * Write data point to InfluxDB (rate-limited by write_interval_s)
     * 
     * @param state Current plant state
     * @param sensor_out Sensor measurements
     * @param cmd Actuator commands
     * @param sim_time Current simulation time (used for rate limiting only)
     * @return true if write succeeded, false if skipped or failed
     */
    bool write_data_point(const plant::PlantState& state,
                         const sensors::SensorOut& sensor_out,
                         const sim::ActuatorCmd& cmd,
                         double sim_time);
    
    /**
     * Flush any buffered data (no-op in current implementation)
     */
    void flush();
    
    /**
     * Check if InfluxDB logging is enabled
     */
    bool is_enabled() const { return config_.enabled; }
    
private:
    Config config_;
    double last_write_time_;
    
    // Pimpl idiom to hide CURL implementation details
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    // Line protocol builders for each measurement
    std::string build_vehicle_truth_line(const plant::PlantState& state,
                                        const sim::ActuatorCmd& cmd,
                                        int64_t timestamp_ns);
    
    std::string build_battery_sensors_line(const plant::PlantState& state,
                                          const sensors::SensorOut& sensor_out,
                                          int64_t timestamp_ns);
    
    std::string build_wheel_sensors_line(const plant::PlantState& state,
                                        const sensors::SensorOut& sensor_out,
                                        int64_t timestamp_ns);
    
    std::string build_imu_sensors_line(const sensors::SensorOut& sensor_out,
                                       int64_t timestamp_ns);
    
    std::string build_gnss_sensors_line(const sensors::SensorOut& sensor_out,
                                        int64_t timestamp_ns);
    
    std::string build_radar_sensors_line(const sensors::SensorOut& sensor_out,
                                         int64_t timestamp_ns);
    
    // HTTP communication
    bool send_to_influx(const std::string& line_protocol);
    
    // Time conversion utilities
    int64_t wall_clock_time_ns();      // Get current wall clock time in nanoseconds
    int64_t sim_time_to_ns(double sim_time_s);  // Convert sim time to ns (deprecated)
};

} // namespace utils