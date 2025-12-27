// src/sensors/radar_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"

namespace sensors {

enum class RadarWeatherCondition {
    CLEAR,
    LIGHT_RAIN,
    HEAVY_RAIN,
    FOG
};

struct RadarSensorParams {
    double range_noise_stddev = 0.2;       // meters
    double doppler_noise_stddev = 0.1;     // m/s
    double angle_noise_stddev = 0.5;       // degrees
    double update_hz = 20.0;
    RadarWeatherCondition weather_condition = RadarWeatherCondition::CLEAR;
    uint64_t random_seed = 0;
};

/**
 * Automotive Radar Sensor
 * 
 * Measures range, range-rate (doppler), and angle to targets
 * Weather effects degrade performance
 */
class RadarSensor : public SensorBase {
public:
    explicit RadarSensor(const RadarSensorParams& params = {})
        : params_(params)
        , range_noise_(params.random_seed)
        , doppler_noise_(params.random_seed ? params.random_seed + 1 : 0)
        , angle_noise_(params.random_seed ? params.random_seed + 2 : 0)
        , uniform_noise_(params.random_seed ? params.random_seed + 3 : 0)
        , rate_limiter_(params.update_hz)
    {
        update_weather_effects();
        reset();
    }

    void set_weather(RadarWeatherCondition weather) {
        params_.weather_condition = weather;
        update_weather_effects();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        (void)dt; // Unused parameter
        
        // Simulate a virtual target at fixed distance ahead
        double target_range_truth = 50.0;        // 50m ahead
        double target_velocity_truth = 0.0;      // Stationary
        double target_angle_truth = 0.0;         // Directly ahead
        
        // Relative velocity (closing rate)
        double relative_velocity = truth.v_mps - target_velocity_truth;
        
        // Apply weather-dependent noise
        double range_noise = range_noise_.gaussian(params_.range_noise_stddev * weather_range_factor_);
        double doppler_noise = doppler_noise_.gaussian(params_.doppler_noise_stddev * weather_doppler_factor_);
        double angle_noise = angle_noise_.gaussian(params_.angle_noise_stddev * weather_angle_factor_);
        
        double range_raw = target_range_truth + range_noise;
        double doppler_raw = relative_velocity + doppler_noise;
        double angle_raw = target_angle_truth + angle_noise;
        
        // Simulate false detection probability in fog
        bool valid = true;
        if (params_.weather_condition == RadarWeatherCondition::FOG) {
            // 30% chance of false detection in fog
            double rand_val = uniform_noise_.uniform(0.0, 1.0);
            if (rand_val > 0.7) {
                valid = false;
            }
        }
        
        // Rate limiting
        auto [range_meas, range_updated] = rate_limiter_.update(t, range_raw);
        auto [doppler_meas, doppler_updated] = rate_limiter_.update(t, doppler_raw);
        auto [angle_meas, angle_updated] = rate_limiter_.update(t, angle_raw);
        
        out_.radar_range_m = range_meas;
        out_.radar_range_rate_mps = doppler_meas;
        out_.radar_angle_deg = angle_meas;
        out_.radar_valid_target = valid;
        out_.t_s = t;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        rate_limiter_.reset();
    }

    std::string name() const override {
        return "RadarSensor";
    }

private:
    void update_weather_effects() {
        switch (params_.weather_condition) {
            case RadarWeatherCondition::CLEAR:
                weather_range_factor_ = 1.0;
                weather_doppler_factor_ = 1.0;
                weather_angle_factor_ = 1.0;
                break;
            case RadarWeatherCondition::LIGHT_RAIN:
                weather_range_factor_ = 1.5;
                weather_doppler_factor_ = 1.2;
                weather_angle_factor_ = 1.3;
                break;
            case RadarWeatherCondition::HEAVY_RAIN:
                weather_range_factor_ = 3.0;
                weather_doppler_factor_ = 2.0;
                weather_angle_factor_ = 2.5;
                break;
            case RadarWeatherCondition::FOG:
                weather_range_factor_ = 5.0;
                weather_doppler_factor_ = 1.5;
                weather_angle_factor_ = 4.0;
                break;
        }
    }

    RadarSensorParams params_;
    SensorOut out_;
    
    // Noise generators
    utils::NoiseGenerator range_noise_;
    utils::NoiseGenerator doppler_noise_;
    utils::NoiseGenerator angle_noise_;
    utils::NoiseGenerator uniform_noise_;
    
    // Weather effects
    double weather_range_factor_ = 1.0;
    double weather_doppler_factor_ = 1.0;
    double weather_angle_factor_ = 1.0;
    
    // Rate limiter
    utils::RateLimiter rate_limiter_;
};

} // namespace sensors