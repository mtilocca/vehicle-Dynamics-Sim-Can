// src/sim/timing_controller.hpp
#pragma once

#include <chrono>
#include <thread>
#include <cmath>

namespace sim {

/**
 * TimingController - Cross-platform high-precision timing
 * 
 * Features:
 * - Adaptive sleep compensation (handles OS scheduler jitter)
 * - Deadline miss detection and statistics
 * - Busy-wait final microseconds for precision
 * - Works on Linux, macOS, Windows
 */
class TimingController {
public:
    struct Stats {
        size_t total_steps = 0;
        size_t deadline_misses = 0;
        double max_lateness_us = 0.0;
        double avg_lateness_us = 0.0;
        double max_loop_time_us = 0.0;
    };

    explicit TimingController(double dt_s)
        : dt_ns_(static_cast<int64_t>(dt_s * 1e9)),
          spin_threshold_ns_(50000)  // Busy-wait last 50us for precision
    {
        reset();
    }

    void reset() {
        step_count_ = 0;
        total_lateness_us_ = 0.0;
        stats_ = Stats{};
        epoch_ = std::chrono::steady_clock::now();
        last_step_end_ = epoch_;
    }

    /**
     * Wait until next timestep deadline
     * 
     * Returns false if deadline was missed (loop too slow)
     */
    bool wait_for_next_step() {
        using namespace std::chrono;
        
        auto now = steady_clock::now();
        step_count_++;
        
        // Calculate absolute deadline for this step
        auto deadline = epoch_ + nanoseconds(step_count_ * dt_ns_);
        
        // Check if we missed the deadline
        auto time_until_deadline = deadline - now;
        int64_t remaining_ns = duration_cast<nanoseconds>(time_until_deadline).count();
        
        bool missed_deadline = false;
        if (remaining_ns < 0) {
            missed_deadline = true;
            stats_.deadline_misses++;
            
            double lateness_us = -remaining_ns / 1000.0;
            stats_.max_lateness_us = std::max(stats_.max_lateness_us, lateness_us);
            total_lateness_us_ += lateness_us;
            stats_.avg_lateness_us = total_lateness_us_ / stats_.deadline_misses;
        }
        
        // Two-phase sleep for precision:
        // 1. Coarse sleep until close to deadline (OS scheduler)
        // 2. Busy-wait final microseconds (tight loop)
        
        if (remaining_ns > spin_threshold_ns_) {
            // Sleep most of the time (lets OS schedule other work)
            auto sleep_until = deadline - nanoseconds(spin_threshold_ns_);
            std::this_thread::sleep_until(sleep_until);
        }
        
        // Busy-wait for final precision
        while (steady_clock::now() < deadline) {
            // Tight loop - burns CPU but guarantees precision
            std::this_thread::yield();  // Hint to OS, but keep spinning
        }
        
        last_step_end_ = steady_clock::now();
        return !missed_deadline;
    }

    /**
     * Get current simulation time (in seconds)
     */
    double get_sim_time() const {
        return step_count_ * (dt_ns_ / 1e9);
    }

    /**
     * Get wall-clock time since reset (in seconds)
     */
    double get_wall_time() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - epoch_);
        return elapsed.count();
    }

    /**
     * Get time drift: wall_time - sim_time (positive = running slow)
     */
    double get_time_drift() const {
        return get_wall_time() - get_sim_time();
    }

    /**
     * Get last loop execution time (excluding sleep)
     */
    double get_last_loop_time_us() const {
        auto duration = last_step_end_ - last_loop_start_;
        return std::chrono::duration<double, std::micro>(duration).count();
    }

    /**
     * Mark start of computation (for loop time measurement)
     */
    void mark_loop_start() {
        last_loop_start_ = std::chrono::steady_clock::now();
    }

    /**
     * Update loop time statistics
     */
    void update_loop_stats() {
        double loop_us = get_last_loop_time_us();
        stats_.max_loop_time_us = std::max(stats_.max_loop_time_us, loop_us);
        stats_.total_steps++;
    }

    const Stats& get_stats() const { return stats_; }

    /**
     * Set busy-wait threshold (microseconds before deadline to start spinning)
     * 
     * Larger = more precision, more CPU usage
     * Smaller = less CPU usage, less precision
     * 
     * Typical values: 20-100 us
     */
    void set_spin_threshold_us(double us) {
        spin_threshold_ns_ = static_cast<int64_t>(us * 1000.0);
    }

private:
    int64_t dt_ns_;                  // Timestep in nanoseconds
    int64_t spin_threshold_ns_;      // When to start busy-waiting
    
    size_t step_count_ = 0;
    double total_lateness_us_ = 0.0;
    
    std::chrono::steady_clock::time_point epoch_;
    std::chrono::steady_clock::time_point last_loop_start_;
    std::chrono::steady_clock::time_point last_step_end_;
    
    Stats stats_;
};

} // namespace sim