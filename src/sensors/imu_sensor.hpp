// src/sensors/imu_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"

namespace sensors {

struct IMUSensorParams {
    // Gyroscope noise parameters (all 3 axes)
    double gyro_noise_stddev = 0.001;      // rad/s (white noise)
    double gyro_bias_sigma = 0.0005;       // rad/s (drift strength)
    double gyro_bias_tau = 1800.0;         // seconds (30 min)
    
    // Accelerometer noise parameters (all 3 axes)
    double accel_noise_stddev = 0.05;      // m/s² (white noise)
    double accel_bias_sigma = 0.02;        // m/s² (drift strength)
    double accel_bias_tau = 3600.0;        // seconds (1 hour)
    
    // Temperature sensor
    double temp_noise_stddev = 0.5;        // °C
    double temp_nominal = 25.0;            // °C (nominal operating temp)
    
    // Sample rate
    double update_hz = 100.0;              // Hz
    
    uint64_t random_seed = 0;
};

/**
 * IMU Sensor - 6-DOF Inertial Measurement Unit
 * 
 * Measures:
 * - 3-axis gyroscope (angular velocity in rad/s)
 * - 3-axis accelerometer (linear acceleration in m/s²)
 * - Temperature
 * - Status flags
 */
class IMUSensor : public SensorBase {
public:
    explicit IMUSensor(const IMUSensorParams& params = {})
        : params_(params)
        // Gyro noise generators (one per axis)
        , gyro_x_noise_(params.random_seed)
        , gyro_y_noise_(params.random_seed ? params.random_seed + 1 : 0)
        , gyro_z_noise_(params.random_seed ? params.random_seed + 2 : 0)
        // Accel noise generators (one per axis)
        , accel_x_noise_(params.random_seed ? params.random_seed + 3 : 0)
        , accel_y_noise_(params.random_seed ? params.random_seed + 4 : 0)
        , accel_z_noise_(params.random_seed ? params.random_seed + 5 : 0)
        // Gyro bias models
        , gyro_x_bias_(params.gyro_bias_tau, params.gyro_bias_sigma, params.random_seed)
        , gyro_y_bias_(params.gyro_bias_tau, params.gyro_bias_sigma, params.random_seed ? params.random_seed + 1 : 0)
        , gyro_z_bias_(params.gyro_bias_tau, params.gyro_bias_sigma, params.random_seed ? params.random_seed + 2 : 0)
        // Accel bias models
        , accel_x_bias_(params.accel_bias_tau, params.accel_bias_sigma, params.random_seed ? params.random_seed + 3 : 0)
        , accel_y_bias_(params.accel_bias_tau, params.accel_bias_sigma, params.random_seed ? params.random_seed + 4 : 0)
        , accel_z_bias_(params.accel_bias_tau, params.accel_bias_sigma, params.random_seed ? params.random_seed + 5 : 0)
        // Temperature noise
        , temp_noise_(params.random_seed ? params.random_seed + 6 : 0)
        // Rate limiter
        , rate_limiter_(params.update_hz)
    {
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // ====================================================================
        // GYROSCOPE - 3-axis angular velocity (rad/s)
        // ====================================================================
        
        // Update biases
        double gyro_x_bias_val = gyro_x_bias_.step(dt);
        double gyro_y_bias_val = gyro_y_bias_.step(dt);
        double gyro_z_bias_val = gyro_z_bias_.step(dt);
        
        // X-axis (roll rate) - assume zero for ground vehicle
        double gyro_x_noise = gyro_x_noise_.gaussian(params_.gyro_noise_stddev);
        double gyro_x_raw = 0.0 + gyro_x_noise + gyro_x_bias_val;
        
        // Y-axis (pitch rate) - assume zero for ground vehicle
        double gyro_y_noise = gyro_y_noise_.gaussian(params_.gyro_noise_stddev);
        double gyro_y_raw = 0.0 + gyro_y_noise + gyro_y_bias_val;
        
        // Z-axis (yaw rate) - use steer_rate_radps as proxy
        double yaw_rate_truth = truth.steer_rate_radps;  // rad/s
        double gyro_z_noise = gyro_z_noise_.gaussian(params_.gyro_noise_stddev);
        double gyro_z_raw = yaw_rate_truth + gyro_z_noise + gyro_z_bias_val;
        
        // ====================================================================
        // ACCELEROMETER - 3-axis linear acceleration (m/s²)
        // ====================================================================
        
        // Update biases
        double accel_x_bias_val = accel_x_bias_.step(dt);
        double accel_y_bias_val = accel_y_bias_.step(dt);
        double accel_z_bias_val = accel_z_bias_.step(dt);
        
        // X-axis (longitudinal)
        double accel_x_truth = truth.a_long_mps2;
        double accel_x_noise = accel_x_noise_.gaussian(params_.accel_noise_stddev);
        double accel_x_raw = accel_x_truth + accel_x_noise + accel_x_bias_val;
        
        // Y-axis (lateral) - approximate from v * yaw_rate
        double accel_y_truth = truth.v_mps * truth.steer_rate_radps;
        double accel_y_noise = accel_y_noise_.gaussian(params_.accel_noise_stddev);
        double accel_y_raw = accel_y_truth + accel_y_noise + accel_y_bias_val;
        
        // Z-axis (vertical) - gravity + small noise (assume flat ground)
        double accel_z_truth = -9.81;  // Gravity pointing down
        double accel_z_noise = accel_z_noise_.gaussian(params_.accel_noise_stddev);
        double accel_z_raw = accel_z_truth + accel_z_noise + accel_z_bias_val;
        
        // ====================================================================
        // TEMPERATURE
        // ====================================================================
        double temp_noise = temp_noise_.gaussian(params_.temp_noise_stddev);
        double temp_raw = params_.temp_nominal + temp_noise;
        
        // ====================================================================
        // RATE LIMITING (all measurements updated at same rate)
        // ====================================================================
        auto [gyro_x_meas, gyro_x_updated] = rate_limiter_.update(t, gyro_x_raw);
        auto [gyro_y_meas, gyro_y_updated] = rate_limiter_.update(t, gyro_y_raw);
        auto [gyro_z_meas, gyro_z_updated] = rate_limiter_.update(t, gyro_z_raw);
        auto [accel_x_meas, accel_x_updated] = rate_limiter_.update(t, accel_x_raw);
        auto [accel_y_meas, accel_y_updated] = rate_limiter_.update(t, accel_y_raw);
        auto [accel_z_meas, accel_z_updated] = rate_limiter_.update(t, accel_z_raw);
        auto [temp_meas, temp_updated] = rate_limiter_.update(t, temp_raw);
        
        // ====================================================================
        // OUTPUT
        // ====================================================================
        out_.imu_gx_rps = gyro_x_meas;
        out_.imu_gy_rps = gyro_y_meas;
        out_.imu_gz_rps = gyro_z_meas;
        out_.imu_ax_mps2 = accel_x_meas;
        out_.imu_ay_mps2 = accel_y_meas;
        out_.imu_az_mps2 = accel_z_meas;
        out_.imu_temp_c = temp_meas;
        out_.imu_status = 0;  // 0 = OK (could add health checks)
        out_.imu_valid = true;
        out_.t_s = t;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        gyro_x_bias_.reset();
        gyro_y_bias_.reset();
        gyro_z_bias_.reset();
        accel_x_bias_.reset();
        accel_y_bias_.reset();
        accel_z_bias_.reset();
        rate_limiter_.reset();
    }

    std::string name() const override {
        return "IMUSensor";
    }

private:
    IMUSensorParams params_;
    SensorOut out_;
    
    // Noise generators (one per axis)
    utils::NoiseGenerator gyro_x_noise_;
    utils::NoiseGenerator gyro_y_noise_;
    utils::NoiseGenerator gyro_z_noise_;
    utils::NoiseGenerator accel_x_noise_;
    utils::NoiseGenerator accel_y_noise_;
    utils::NoiseGenerator accel_z_noise_;
    utils::NoiseGenerator temp_noise_;
    
    // Bias models (one per axis)
    utils::BiasModel gyro_x_bias_;
    utils::BiasModel gyro_y_bias_;
    utils::BiasModel gyro_z_bias_;
    utils::BiasModel accel_x_bias_;
    utils::BiasModel accel_y_bias_;
    utils::BiasModel accel_z_bias_;
    
    // Rate limiter (shared for all measurements)
    utils::RateLimiter rate_limiter_;
};

} // namespace sensors