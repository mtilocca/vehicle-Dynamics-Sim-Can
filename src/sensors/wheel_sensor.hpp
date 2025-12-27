// src/sensors/wheel_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"

namespace sensors {

/**
 * WheelSensorParams - Configuration for wheel speed sensor
 */
struct WheelSensorParams {
    // Encoder parameters
    int ticks_per_revolution = 48;          // Encoder resolution
    double noise_stddev_pct = 0.5;          // % of measurement (multiplicative noise)
    double bias_sigma_pct = 0.1;            // % (drift strength)
    double bias_tau = 1800.0;               // seconds (drift time constant)
    double update_hz = 100.0;               // Hz (sample rate)
    
    // Quantization is inherent in tick counting
    // Resolution = 2π / ticks_per_rev rad/s at 1 Hz
    
    // Random seed (0 = random)
    uint64_t random_seed = 0;
};

/**
 * WheelSensor - Simulates 4-wheel ABS encoder sensors
 * 
 * Models:
 * - Discrete tick counting (quantization)
 * - White noise (sensor jitter)
 * - Multiplicative bias (calibration error, bearing friction)
 * - Finite update rate
 */
class WheelSensor : public SensorBase {
public:
    explicit WheelSensor(const WheelSensorParams& params = {})
        : params_(params),
          noise_fl_(params.random_seed),
          noise_fr_(params.random_seed ? params.random_seed + 1 : 0),
          noise_rl_(params.random_seed ? params.random_seed + 2 : 0),
          noise_rr_(params.random_seed ? params.random_seed + 3 : 0),
          bias_fl_(params.bias_tau, params.bias_sigma_pct / 100.0, params.random_seed),
          bias_fr_(params.bias_tau, params.bias_sigma_pct / 100.0, params.random_seed ? params.random_seed + 1 : 0),
          bias_rl_(params.bias_tau, params.bias_sigma_pct / 100.0, params.random_seed ? params.random_seed + 2 : 0),
          bias_rr_(params.bias_tau, params.bias_sigma_pct / 100.0, params.random_seed ? params.random_seed + 3 : 0),
          limiter_fl_(params.update_hz),
          limiter_fr_(params.update_hz),
          limiter_rl_(params.update_hz),
          limiter_rr_(params.update_hz)
    {
        // Calculate quantization resolution (rad/s)
        // At 1 RPS, encoder generates ticks_per_rev ticks/sec
        // Minimum resolvable speed = 2π / ticks_per_rev rad/s
        quant_resolution_ = (2.0 * M_PI) / params_.ticks_per_revolution;
        
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // Process each wheel independently
        out_.wheel_fl_rps_meas = process_wheel(t, dt, truth.wheel_fl_rps, 
                                                noise_fl_, bias_fl_, limiter_fl_);
        out_.wheel_fr_rps_meas = process_wheel(t, dt, truth.wheel_fr_rps,
                                                noise_fr_, bias_fr_, limiter_fr_);
        out_.wheel_rl_rps_meas = process_wheel(t, dt, truth.wheel_rl_rps,
                                                noise_rl_, bias_rl_, limiter_rl_);
        out_.wheel_rr_rps_meas = process_wheel(t, dt, truth.wheel_rr_rps,
                                                noise_rr_, bias_rr_, limiter_rr_);

        out_.t_s = t;
        out_.wheel_valid = true;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        bias_fl_.reset();
        bias_fr_.reset();
        bias_rl_.reset();
        bias_rr_.reset();
        limiter_fl_.reset();
        limiter_fr_.reset();
        limiter_rl_.reset();
        limiter_rr_.reset();
    }

    std::string name() const override { return "WheelSensor"; }

private:
    double process_wheel(double t, double dt, double truth_rps,
                        utils::NoiseGenerator& noise,
                        utils::BiasModel& bias,
                        utils::RateLimiter& limiter) {
        // ====================================================================
        // 1. Apply multiplicative bias (calibration error, ~0.1-1%)
        // ====================================================================
        double bias_factor = 1.0 + bias.step(dt);
        double speed_with_bias = truth_rps * bias_factor;

        // ====================================================================
        // 2. Add white noise (proportional to speed)
        // ====================================================================
        double noise_stddev = std::abs(speed_with_bias) * (params_.noise_stddev_pct / 100.0);
        double noise_val = noise.gaussian(noise_stddev);
        double speed_noisy = speed_with_bias + noise_val;

        // ====================================================================
        // 3. Quantize to encoder ticks (discrete measurement)
        // ====================================================================
        // Round to nearest quantum
        double speed_quant = std::round(speed_noisy / quant_resolution_) * quant_resolution_;

        // ====================================================================
        // 4. Apply sample rate limiting
        // ====================================================================
        auto [speed_meas, updated] = limiter.update(t, speed_quant);

        return speed_meas;
    }

    WheelSensorParams params_;
    SensorOut out_;

    double quant_resolution_;  // rad/s per tick

    // Per-wheel noise generators
    utils::NoiseGenerator noise_fl_;
    utils::NoiseGenerator noise_fr_;
    utils::NoiseGenerator noise_rl_;
    utils::NoiseGenerator noise_rr_;

    // Per-wheel bias models (multiplicative calibration error)
    utils::BiasModel bias_fl_;
    utils::BiasModel bias_fr_;
    utils::BiasModel bias_rl_;
    utils::BiasModel bias_rr_;

    // Per-wheel rate limiters
    utils::RateLimiter limiter_fl_;
    utils::RateLimiter limiter_fr_;
    utils::RateLimiter limiter_rl_;
    utils::RateLimiter limiter_rr_;
};

} // namespace sensors