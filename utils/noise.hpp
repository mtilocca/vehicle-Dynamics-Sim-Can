// src/utils/noise.hpp
#pragma once

#include <random>
#include <cmath>

namespace utils {

/**
 * NoiseGenerator - Stateful random number generator for sensor noise
 * 
 * Thread-safe per-instance (each sensor gets its own generator)
 */
class NoiseGenerator {
public:
    explicit NoiseGenerator(uint64_t seed = 0)
        : gen_(seed == 0 ? std::random_device{}() : seed),
          dist_(0.0, 1.0)
    {}

    /**
     * Gaussian white noise: N(0, stddev)
     */
    double gaussian(double stddev) {
        if (stddev <= 0.0) return 0.0;
        return dist_(gen_) * stddev;
    }

    /**
     * Uniform random in [-range, +range]
     */
    double uniform(double range) {
        if (range <= 0.0) return 0.0;
        return (dist_(gen_) - 0.5) * 2.0 * range;
    }

    /**
     * Random walk (integrated white noise)
     * Returns incremental change, caller accumulates
     */
    double random_walk(double sigma, double dt) {
        return gaussian(sigma * std::sqrt(dt));
    }

    /**
     * Exponential decay with time constant tau
     * For bias drift: bias += decay(bias, tau, dt) + white_noise
     */
    double decay(double current_value, double tau, double dt) {
        if (tau <= 0.0) return current_value;
        double alpha = std::exp(-dt / tau);
        return current_value * alpha;
    }

private:
    std::mt19937_64 gen_;
    std::normal_distribution<double> dist_;
};

/**
 * BiasModel - First-order Gauss-Markov bias drift
 * 
 * Models slowly varying bias (e.g., temperature drift, aging)
 * 
 * db/dt = -bias/tau + white_noise(sigma)
 */
class BiasModel {
public:
    BiasModel(double tau_s = 3600.0, double sigma = 0.001, uint64_t seed = 0)
        : tau_(tau_s),
          sigma_(sigma),
          bias_(0.0),
          noise_(seed)
    {}

    double step(double dt) {
        // Exponential decay toward zero
        bias_ = noise_.decay(bias_, tau_, dt);
        
        // Add random walk component
        bias_ += noise_.random_walk(sigma_, dt);
        
        return bias_;
    }

    double get() const { return bias_; }
    void reset() { bias_ = 0.0; }
    void set(double value) { bias_ = value; }

private:
    double tau_;    // Time constant (seconds)
    double sigma_;  // Noise strength
    double bias_;
    NoiseGenerator noise_;
};

/**
 * Quantizer - Simulates ADC/encoder quantization
 */
class Quantizer {
public:
    explicit Quantizer(double resolution = 0.0)
        : resolution_(resolution)
    {}

    double quantize(double value) const {
        if (resolution_ <= 0.0) return value;
        
        // Round to nearest quantum
        return std::round(value / resolution_) * resolution_;
    }

    void set_resolution(double res) { resolution_ = res; }
    double get_resolution() const { return resolution_; }

private:
    double resolution_;
};

/**
 * RateLimiter - Simulates finite sensor update rate
 */
class RateLimiter {
public:
    explicit RateLimiter(double update_hz = 0.0)
        : period_(update_hz > 0.0 ? 1.0 / update_hz : 0.0),
          next_update_(0.0),
          last_value_(0.0)
    {}

    /**
     * Update sensor value at specified rate
     * Returns: (value, updated_this_step)
     */
    std::pair<double, bool> update(double t, double new_value) {
        if (period_ <= 0.0) {
            // No rate limiting
            last_value_ = new_value;
            return {new_value, true};
        }

        if (t >= next_update_) {
            last_value_ = new_value;
            next_update_ = t + period_;
            return {last_value_, true};
        }

        // Hold previous value
        return {last_value_, false};
    }

    void reset(double t = 0.0) {
        next_update_ = t;
        last_value_ = 0.0;
    }

private:
    double period_;
    double next_update_;
    double last_value_;
};

} // namespace utils