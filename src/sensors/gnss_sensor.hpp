// src/sensors/gnss_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"
#include <cmath>

namespace sensors {

struct GNSSSensorParams {
    // Position noise (CEP = Circular Error Probable)
    double position_noise_stddev = 2.0;    // meters (CEP)
    double altitude_noise_stddev = 5.0;    // meters
    
    // Velocity noise
    double velocity_noise_stddev = 0.1;    // m/s
    
    // Random walk drift
    double drift_sigma = 0.5;              // m/√s
    
    // Origin for local XY → lat/lon conversion
    double origin_lat_deg = 0.0;           // Reference latitude (degrees)
    double origin_lon_deg = 0.0;           // Reference longitude (degrees)
    
    // Update rate
    double update_hz = 10.0;               // Hz
    
    // Simulated quality
    uint8_t nominal_fix_type = 3;          // 3 = 3D fix
    uint8_t nominal_sat_count = 12;        // satellites
    
    uint64_t random_seed = 0;
};

/**
 * GNSS Sensor (GPS)
 * 
 * Converts local XY coordinates to WGS84 lat/lon
 * Provides velocity in NED (North/East/Down) frame
 * Reports fix quality and satellite count
 */
class GNSSSensor : public SensorBase {
public:
    explicit GNSSSensor(const GNSSSensorParams& params = {})
        : params_(params)
        , pos_noise_(params.random_seed)
        , alt_noise_(params.random_seed ? params.random_seed + 1 : 0)
        , vel_noise_(params.random_seed ? params.random_seed + 2 : 0)
        , drift_noise_(params.random_seed ? params.random_seed + 3 : 0)
        , rate_limiter_(params.update_hz)
    {
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // ====================================================================
        // POSITION DRIFT (random walk)
        // ====================================================================
        double drift_step_x = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        double drift_step_y = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        double drift_step_alt = drift_noise_.gaussian(params_.drift_sigma * std::sqrt(dt));
        
        pos_x_drift_ += drift_step_x;
        pos_y_drift_ += drift_step_y;
        alt_drift_ += drift_step_alt;
        
        // ====================================================================
        // POSITION (local XY → lat/lon conversion)
        // ====================================================================
        
        // Add noise and drift to local position
        double pos_x_noise = pos_noise_.gaussian(params_.position_noise_stddev);
        double pos_y_noise = pos_noise_.gaussian(params_.position_noise_stddev);
        double pos_x_noisy = truth.x_m + pos_x_drift_ + pos_x_noise;
        double pos_y_noisy = truth.y_m + pos_y_drift_ + pos_y_noise;
        
        // Convert local XY (meters) to lat/lon (degrees)
        // Simple approximation: 1 degree latitude ≈ 111,320 meters
        // 1 degree longitude ≈ 111,320 * cos(latitude) meters
        const double METERS_PER_DEG_LAT = 111320.0;
        double meters_per_deg_lon = METERS_PER_DEG_LAT * std::cos(params_.origin_lat_deg * M_PI / 180.0);
        
        double lat_raw = params_.origin_lat_deg + (pos_y_noisy / METERS_PER_DEG_LAT);
        double lon_raw = params_.origin_lon_deg + (pos_x_noisy / meters_per_deg_lon);
        
        // Altitude (assume ground level = 0m MSL)
        double alt_noise = alt_noise_.gaussian(params_.altitude_noise_stddev);
        double alt_raw = 0.0 + alt_drift_ + alt_noise;
        
        // ====================================================================
        // VELOCITY (convert from vehicle frame to NED frame)
        // ====================================================================
        
        // Vehicle velocity in NED frame
        // North = v * sin(yaw), East = v * cos(yaw) in standard convention
        double vn_truth = truth.v_mps * std::sin(truth.yaw_rad);  // North
        double ve_truth = truth.v_mps * std::cos(truth.yaw_rad);  // East
        
        // Add noise
        double vn_noise = vel_noise_.gaussian(params_.velocity_noise_stddev);
        double ve_noise = vel_noise_.gaussian(params_.velocity_noise_stddev);
        
        double vn_raw = vn_truth + vn_noise;
        double ve_raw = ve_truth + ve_noise;
        
        // ====================================================================
        // FIX QUALITY (simulate good fix with occasional degradation)
        // ====================================================================
        
        // Simulate occasional fix loss (1% chance per update)
        double fix_random = drift_noise_.uniform(1.0);
        uint8_t fix_type = (fix_random > 0.99) ? 0 : params_.nominal_fix_type;
        
        // Satellite count varies slightly
        double sat_variation = drift_noise_.gaussian(2.0);
        uint8_t sat_count = static_cast<uint8_t>(
            std::max(0.0, std::min(255.0, params_.nominal_sat_count + sat_variation))
        );
        
        // ====================================================================
        // RATE LIMITING
        // ====================================================================
        auto [lat_meas, lat_updated] = rate_limiter_.update(t, lat_raw);
        auto [lon_meas, lon_updated] = rate_limiter_.update(t, lon_raw);
        auto [alt_meas, alt_updated] = rate_limiter_.update(t, alt_raw);
        auto [vn_meas, vn_updated] = rate_limiter_.update(t, vn_raw);
        auto [ve_meas, ve_updated] = rate_limiter_.update(t, ve_raw);
        
        // ====================================================================
        // OUTPUT
        // ====================================================================
        out_.gnss_lat_deg = lat_meas;
        out_.gnss_lon_deg = lon_meas;
        out_.gnss_alt_m = alt_meas;
        out_.gnss_vn_mps = vn_meas;
        out_.gnss_ve_mps = ve_meas;
        out_.gnss_fix_type = fix_type;
        out_.gnss_sat_count = sat_count;
        out_.gnss_valid = (fix_type >= 2);  // Valid if 2D or 3D fix
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
    utils::NoiseGenerator alt_noise_;
    utils::NoiseGenerator vel_noise_;
    utils::NoiseGenerator drift_noise_;
    
    // Drift states
    double pos_x_drift_ = 0.0;
    double pos_y_drift_ = 0.0;
    double alt_drift_ = 0.0;
    
    // Rate limiter
    utils::RateLimiter rate_limiter_;
};

} // namespace sensors