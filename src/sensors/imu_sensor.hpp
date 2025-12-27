// src/sensors/imu_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"

namespace sensors {

struct IMUSensorParams {
    double gyro_noise_stddev = 0.1;        // deg/s
    double accel_noise_stddev = 0.05;      // m/s^2
    double gyro_bias_sigma = 0.05;         // deg/s
    double gyro_bias_tau = 1800.0;         // 30 min
    double accel_bias_sigma = 0.02;        // m/s^2
    double accel_bias_tau = 3600.0;        // 1 hour
    double update_hz = 100.0;
    uint64_t random_seed = 0;
};

/**
 * IMU Sensor - Gyroscope and Accelerometer
 * 
 * Measures angular velocity (yaw rate) and linear acceleration
 */
class IMUSensor : public SensorBase {
public:
    explicit IMUSensor(const IMUSensorParams& params = {})
        : params_(params)
        , gyro_noise_(params.random_seed)
        , accel_x_noise_(params.random_seed ? params.random_seed + 1 : 0)
        , accel_y_noise_(params.random_seed ? params.random_seed + 2 : 0)
        , gyro_bias_(params.gyro_bias_tau, params.gyro_bias_sigma, params.random_seed)
        , accel_x_bias_(params.accel_bias_tau, params.accel_bias_sigma, params.random_seed ? params.random_seed + 1 : 0)
        , accel_y_bias_(params.accel_bias_tau, params.accel_bias_sigma, params.random_seed ? params.random_seed + 2 : 0)
        , rate_limiter_(params.update_hz)
    {
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // Update biases
        double gyro_bias_val = gyro_bias_.step(dt);
        double accel_x_bias_val = accel_x_bias_.step(dt);
        double accel_y_bias_val = accel_y_bias_.step(dt);
        
        // Gyroscope - yaw rate (convert rad/s to deg/s)
        // PlantState has steer_rate_radps, use as proxy for yaw rate
        double yaw_rate_truth_dps = truth.steer_rate_radps * (180.0 / M_PI);
        double gyro_noise = gyro_noise_.gaussian(params_.gyro_noise_stddev);
        double gyro_raw = yaw_rate_truth_dps + gyro_noise + gyro_bias_val;
        
        // Accelerometer X - longitudinal
        // PlantState has a_long_mps2
        double accel_x_noise = accel_x_noise_.gaussian(params_.accel_noise_stddev);
        double accel_x_raw = truth.a_long_mps2 + accel_x_noise + accel_x_bias_val;
        
        // Accelerometer Y - lateral (from turning)
        // Lateral accel ≈ v * yaw_rate = v * steer_rate (simplified)
        double lateral_accel_truth = truth.v_mps * truth.steer_rate_radps;
        double accel_y_noise = accel_y_noise_.gaussian(params_.accel_noise_stddev);
        double accel_y_raw = lateral_accel_truth + accel_y_noise + accel_y_bias_val;
        
        // Rate limiting
        auto [gyro_meas, gyro_updated] = rate_limiter_.update(t, gyro_raw);
        auto [accel_x_meas, accel_x_updated] = rate_limiter_.update(t, accel_x_raw);
        auto [accel_y_meas, accel_y_updated] = rate_limiter_.update(t, accel_y_raw);
        
        out_.imu_gyro_yaw_rate_dps = gyro_meas;
        out_.imu_accel_x_mps2 = accel_x_meas;
        out_.imu_accel_y_mps2 = accel_y_meas;
        out_.imu_valid = true;
        out_.t_s = t;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        gyro_bias_.reset();
        accel_x_bias_.reset();
        accel_y_bias_.reset();
        rate_limiter_.reset();
    }

    std::string name() const override {
        return "IMUSensor";
    }

private:
    IMUSensorParams params_;
    SensorOut out_;
    
    // Noise generators
    utils::NoiseGenerator gyro_noise_;
    utils::NoiseGenerator accel_x_noise_;
    utils::NoiseGenerator accel_y_noise_;
    
    // Bias models
    utils::BiasModel gyro_bias_;
    utils::BiasModel accel_x_bias_;
    utils::BiasModel accel_y_bias_;
    
    // Rate limiter
    utils::RateLimiter rate_limiter_;
};

} // namespace sensors