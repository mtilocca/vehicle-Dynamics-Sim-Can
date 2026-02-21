// utils/influx.hpp
#pragma once

#include <string>
#include <memory>
#include <cstdint>
#include "plant/plant_main/plant_state.hpp"

namespace utils {

/**
 * InfluxDB Line Protocol Client for Time-Series Logging
 *
 * Writes vehicle truth data to InfluxDB v2 using the Line Protocol format over HTTP.
 * Logs core vehicle dynamics: pose, kinematics, steering, motor/brake outputs.
 *
 * Usage:
 *   InfluxClient::Config config;
 *   config.enabled = true;
 *   config.url = "http://localhost:8086";
 *   config.token = "your-token";
 *   config.org = "Autonomy";
 *   config.bucket = "vehicle-sim";
 *   config.write_interval_s = 0.25;  // 4 Hz
 *
 *   InfluxClient client(config);
 *
 *   // In simulation loop
 *   client.write_vehicle_truth(state, sim_time);
 *
 *   // At end
 *   client.flush();
 */
class InfluxClient {
public:
    struct Config {
        bool enabled = false;
        std::string url = "http://localhost:8086";
        std::string token = "";
        std::string org = "Autonomy";
        std::string bucket = "vehicle-sim";
        double write_interval_s = 0.25;  // 250 ms = 4 Hz
    };

    explicit InfluxClient(const Config& config);
    ~InfluxClient();

    InfluxClient(const InfluxClient&) = delete;
    InfluxClient& operator=(const InfluxClient&) = delete;
    InfluxClient(InfluxClient&&) = delete;
    InfluxClient& operator=(InfluxClient&&) = delete;

    /**
     * Write vehicle truth data to InfluxDB (rate-limited by write_interval_s).
     *
     * Logs: x_m, y_m, yaw_deg, v_mps, vy_mps, yaw_rate_radps,
     *       a_long_mps2, a_lat_mps2, steer_deg, delta_fl_deg, delta_fr_deg,
     *       motor_torque_nm, brake_force_kN, wheel_{fl,fr,rl,rr}_rps
     *
     * @param state  Current plant state
     * @param sim_time  Current simulation time [s] (used for rate limiting)
     * @return true if write succeeded, false if rate-limited or failed
     */
    bool write_vehicle_truth(const plant::PlantState& state, double sim_time);

    void flush();

    bool is_enabled() const { return config_.enabled; }

private:
    Config config_;
    double last_write_time_;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    std::string build_vehicle_truth_line(const plant::PlantState& state,
                                         int64_t timestamp_ns);

    bool send_to_influx(const std::string& line_protocol);

    int64_t wall_clock_time_ns();
};

} // namespace utils
