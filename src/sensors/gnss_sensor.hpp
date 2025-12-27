// src/sensors/gnss_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"
#include <cmath>

namespace sensors {

struct GNSSSensorParams {
    double position_noise_stddev = 2.0;    // meters (CEP)
    double velocity_noise_stddev = 0.1;    // m/s
    double heading_noise_stddev = 1.0;     // degrees
    double altitude_noise_stddev = 5.0;    // meters
    double drift_sigma = 0.5;              // random walk
    double update_hz = 10.0;
    uint64_t random_seed = 0;
};

/**
 * GNSS Sensor (GPS)
 * 
 * Measures global position, velocity, and heading
 */
class GNSSSensor : public SensorBase {
public:
    explicit GNSSSensor(const GNSSSensorParams& params = {})
        : params_(params)
        , pos_noise_(params.random_seed)
        , vel_noise_(params.random_seed ? params.random_seed + 1 : 0)
        , hdg_noise_(params.random_seed ? params.random_seed + 2 : 0)
        , alt_noise_(params.random_seed ? params.random_seed + 3 : 0)
        , drift_noise_(params.random_seed ? params.random_seed + 4 : 0)
        , rate_limiter_(params.update_hz)
    {
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // Update position drift (random walk)
        double drift_step_x = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        double drift_step_y = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        double drift_step_alt = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        
        pos_x_drift_ += drift_step_x;
        pos_y_drift_ += drift_step_y;
        alt_drift_ += drift_step_alt;
        
        // Position (with drift + noise)
        double pos_x_noise = pos_noise_.gaussian(params_.position_noise_stddev);
        double pos_y_noise = pos_noise_.gaussian(params_.position_noise_stddev);
        double pos_x_raw = truth.x_m + pos_x_drift_ + pos_x_noise;
        double pos_y_raw = truth.y_m + pos_y_drift_ + pos_y_noise;
        
        // Altitude (assume ground level)
        double alt_noise_val = alt_noise_.gaussian(params_.altitude_noise_stddev);
        double alt_raw = 0.0 + alt_drift_ + alt_noise_val;
        
        // Velocity
        double vel_noise_val = vel_noise_.gaussian(params_.velocity_noise_stddev);
        double vel_raw = truth.v_mps + vel_noise_val;
        
        // Heading (convert yaw_rad to degrees and add noise)
        double heading_truth_deg = truth.yaw_rad * (180.0 / M_PI);
        double hdg_noise_val = hdg_noise_.gaussian(params_.heading_noise_stddev);
        double hdg_raw = heading_truth_deg + hdg_noise_val;
        
        // Rate limiting
        auto [pos_x_meas, x_updated] = rate_limiter_.update(t, pos_x_raw);
        auto [pos_y_meas, y_updated] = rate_limiter_.update(t, pos_y_raw);
        auto [alt_meas, alt_updated] = rate_limiter_.update(t, alt_raw);
        auto [vel_meas, vel_updated] = rate_limiter_.update(t, vel_raw);
        auto [hdg_meas, hdg_updated] = rate_limiter_.update(t, hdg_raw);
        
        out_.gnss_pos_x_m = pos_x_meas;
        out_.gnss_pos_y_m = pos_y_meas;
        out_.gnss_altitude_m = alt_meas;
        out_.gnss_velocity_mps = vel_meas;
        out_.gnss_heading_deg = hdg_meas;
        out_.gnss_valid = true;
        out_.t_s = t;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        pos_x_drift_ = 0.0;
        pos_y_drift_ = 0.0;
        alt_drift_ = 0.0;
        rate_limiter_.reset();
    }

    std::string name() const override {
        return "GNSSSensor";
    }

private:
    GNSSSensorParams params_;
    SensorOut out_;
    
    // Noise generators
    utils::NoiseGenerator pos_noise_;
    utils::NoiseGenerator vel_noise_;
    utils::NoiseGenerator hdg_noise_;
    utils::NoiseGenerator alt_noise_;
    utils::NoiseGenerator drift_noise_;
    
    // Drift states
    double pos_x_drift_ = 0.0;
    double pos_y_drift_ = 0.0;
    double alt_drift_ = 0.0;
    
    // Rate limiter
    utils::RateLimiter rate_limiter_;
};

} // namespace sensors