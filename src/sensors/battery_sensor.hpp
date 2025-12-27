// src/sensors/battery_sensor.hpp
#pragma once

#include "sensors/sensor_base.hpp"
#include "sensors/sensor_out.hpp"
#include "utils/noise.hpp"

namespace sensors {

/**
 * BatterySensorParams - Configuration for battery sensor noise models
 */
struct BatterySensorParams {
    // Voltage sensor
    double voltage_noise_stddev = 0.5;      // V (white noise)
    double voltage_bias_sigma = 0.01;       // V (drift strength)
    double voltage_bias_tau = 3600.0;       // seconds (drift time constant)
    double voltage_resolution = 0.1;        // V (ADC quantization)
    double voltage_update_hz = 100.0;       // Hz (sample rate)

    // Current sensor
    double current_noise_stddev = 0.2;      // A (white noise)
    double current_bias_sigma = 0.05;       // A (drift strength)
    double current_bias_tau = 1800.0;       // seconds
    double current_resolution = 0.1;        // A
    double current_update_hz = 100.0;       // Hz

    // SOC sensor (typically estimated, not measured)
    double soc_noise_stddev = 0.5;          // % (estimation error)
    double soc_bias_sigma = 0.1;            // % (drift strength)
    double soc_bias_tau = 7200.0;           // seconds
    double soc_resolution = 0.1;            // %
    double soc_update_hz = 1.0;             // Hz (slow update)

    // Temperature sensor
    double temp_noise_stddev = 0.5;         // °C
    double temp_update_hz = 10.0;           // Hz

    // Random seed (0 = random)
    uint64_t random_seed = 0;
};

/**
 * BatterySensor - Simulates battery voltage, current, SOC, temperature sensors
 */
class BatterySensor : public SensorBase {
public:
    explicit BatterySensor(const BatterySensorParams& params = {})
        : params_(params),
          voltage_noise_(params.random_seed),
          current_noise_(params.random_seed ? params.random_seed + 1 : 0),
          soc_noise_(params.random_seed ? params.random_seed + 2 : 0),
          temp_noise_(params.random_seed ? params.random_seed + 3 : 0),
          voltage_bias_(params.voltage_bias_tau, params.voltage_bias_sigma, params.random_seed),
          current_bias_(params.current_bias_tau, params.current_bias_sigma, params.random_seed ? params.random_seed + 1 : 0),
          soc_bias_(params.soc_bias_tau, params.soc_bias_sigma, params.random_seed ? params.random_seed + 2 : 0),
          voltage_quant_(params.voltage_resolution),
          current_quant_(params.current_resolution),
          soc_quant_(params.soc_resolution),
          voltage_limiter_(params.voltage_update_hz),
          current_limiter_(params.current_update_hz),
          soc_limiter_(params.soc_update_hz),
          temp_limiter_(params.temp_update_hz)
    {
        reset();
    }

    void step(double t, const plant::PlantState& truth, double dt) override {
        // ====================================================================
        // Voltage Sensor
        // ====================================================================
        double v_bias = voltage_bias_.step(dt);
        double v_noise = voltage_noise_.gaussian(params_.voltage_noise_stddev);
        double v_raw = truth.batt_v + v_bias + v_noise;
        double v_quant = voltage_quant_.quantize(v_raw);
        
        auto [v_meas, v_updated] = voltage_limiter_.update(t, v_quant);
        out_.batt_v_meas = v_meas;

        // ====================================================================
        // Current Sensor
        // ====================================================================
        double i_bias = current_bias_.step(dt);
        double i_noise = current_noise_.gaussian(params_.current_noise_stddev);
        double i_raw = truth.batt_i + i_bias + i_noise;
        double i_quant = current_quant_.quantize(i_raw);
        
        auto [i_meas, i_updated] = current_limiter_.update(t, i_quant);
        out_.batt_i_meas = i_meas;

        // ====================================================================
        // SOC Sensor (modeled as coulomb counting with drift)
        // ====================================================================
        double soc_bias = soc_bias_.step(dt);
        double soc_noise = soc_noise_.gaussian(params_.soc_noise_stddev);
        double soc_raw = truth.batt_soc_pct + soc_bias + soc_noise;
        double soc_quant = soc_quant_.quantize(soc_raw);
        
        // Clamp to [0, 100]
        soc_quant = std::max(0.0, std::min(100.0, soc_quant));
        
        auto [soc_meas, soc_updated] = soc_limiter_.update(t, soc_quant);
        out_.batt_soc_meas = soc_meas;

        // ====================================================================
        // Temperature Sensor (simple white noise)
        // ====================================================================
        double temp_noise = temp_noise_.gaussian(params_.temp_noise_stddev);
        double temp_raw = truth.batt_temp_c + temp_noise;
        
        auto [temp_meas, temp_updated] = temp_limiter_.update(t, temp_raw);
        out_.batt_temp_meas = temp_meas;

        out_.t_s = t;
        out_.batt_valid = true;
    }

    SensorOut get_output() const override {
        return out_;
    }

    void reset() override {
        out_ = SensorOut{};
        voltage_bias_.reset();
        current_bias_.reset();
        soc_bias_.reset();
        voltage_limiter_.reset();
        current_limiter_.reset();
        soc_limiter_.reset();
        temp_limiter_.reset();
    }

    std::string name() const override { return "BatterySensor"; }

private:
    BatterySensorParams params_;
    SensorOut out_;

    // Noise generators
    utils::NoiseGenerator voltage_noise_;
    utils::NoiseGenerator current_noise_;
    utils::NoiseGenerator soc_noise_;
    utils::NoiseGenerator temp_noise_;

    // Bias models (slow drift)
    utils::BiasModel voltage_bias_;
    utils::BiasModel current_bias_;
    utils::BiasModel soc_bias_;

    // Quantizers (ADC resolution)
    utils::Quantizer voltage_quant_;
    utils::Quantizer current_quant_;
    utils::Quantizer soc_quant_;

    // Rate limiters (sample rate)
    utils::RateLimiter voltage_limiter_;
    utils::RateLimiter current_limiter_;
    utils::RateLimiter soc_limiter_;
    utils::RateLimiter temp_limiter_;
};

} // namespace sensors